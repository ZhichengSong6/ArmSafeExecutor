#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetModelState.h>

#include <arm_msgs/JointReference.h>

#include <urdf/model.h>
#include <XmlRpcValue.h>

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

class ArmSimNode
{
public:
    ArmSimNode()
        : nh_(),
          pnh_("~"),
          has_command_(false),
          gazebo_model_ready_(false),
          has_warned_waiting_for_gazebo_(false)
    {
        pnh_.param<std::string>("robot_description_param", robot_description_param_, "/robot_description");
        pnh_.param<std::string>("urdf_path", urdf_path_, "");

        pnh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
        pnh_.param<std::string>("command_topic", command_topic_, "/arm_controller/joint_ref");

        pnh_.param<double>("publish_rate", publish_rate_, 100.0);
        pnh_.param<double>("kp", kp_, 5.0);
        pnh_.param<double>("default_velocity_limit", default_velocity_limit_, 0.5);
        pnh_.param<double>("command_timeout", command_timeout_, 0.5);

        pnh_.param<bool>("use_command_gains", use_command_gains_, false);
        pnh_.param<double>("command_kp_scale", command_kp_scale_, 1.0);
        pnh_.param<double>("max_tracking_kp", max_tracking_kp_, 20.0);

        /*
         * Gazebo is used only as a geometry/sensor renderer in this mode.
         * arm_sim_node still computes q_next using the original kinematic
         * update, publishes /joint_states, and copies the same q_next into
         * Gazebo using /gazebo/set_model_configuration.
         */
        pnh_.param<bool>("sync_to_gazebo", sync_to_gazebo_, false);
        pnh_.param<std::string>("gazebo_model_name", gazebo_model_name_, "Arm");
        pnh_.param<std::string>("gazebo_urdf_param_name", gazebo_urdf_param_name_, "robot_description");
        pnh_.param<std::string>("gazebo_set_configuration_service",
                                gazebo_set_configuration_service_,
                                "/gazebo/set_model_configuration");
        pnh_.param<std::string>("gazebo_get_model_state_service",
                                gazebo_get_model_state_service_,
                                "/gazebo/get_model_state");
        pnh_.param<double>("gazebo_sync_rate", gazebo_sync_rate_, 50.0);

        if (publish_rate_ <= 0.0)
        {
            ROS_FATAL("[arm_sim_node] publish_rate must be positive.");
            ros::shutdown();
            return;
        }

        if (gazebo_sync_rate_ <= 0.0)
        {
            gazebo_sync_rate_ = publish_rate_;
        }

        loadRobotModel();
        loadJointNames();
        initializeJointsFromUrdf();
        loadInitialPositions();

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 10);

        joint_ref_sub_ = nh_.subscribe(
            command_topic_,
            10,
            &ArmSimNode::jointReferenceCallback,
            this);

        if (sync_to_gazebo_)
        {
            gazebo_set_configuration_client_ =
                nh_.serviceClient<gazebo_msgs::SetModelConfiguration>(
                    gazebo_set_configuration_service_);

            gazebo_get_model_state_client_ =
                nh_.serviceClient<gazebo_msgs::GetModelState>(
                    gazebo_get_model_state_service_);
        }

        sim_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &ArmSimNode::timerCallback,
            this);

        printSummary();
    }

private:
    struct JointSimState
    {
        std::string name;
        int type;

        double q = 0.0;
        double dq = 0.0;

        double q_ref = 0.0;
        double dq_ref = 0.0;

        double kp_ref = 0.0;
        double kd_ref = 0.0;
        double tau_ff_ref = 0.0;

        double lower = 0.0;
        double upper = 0.0;
        double velocity_limit = 0.5;

        bool has_position_limit = false;
        bool is_continuous = false;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher joint_state_pub_;
    ros::Subscriber joint_ref_sub_;
    ros::Timer sim_timer_;
    ros::ServiceClient gazebo_set_configuration_client_;
    ros::ServiceClient gazebo_get_model_state_client_;

    urdf::Model model_;

    std::string robot_description_param_;
    std::string urdf_path_;
    std::string joint_states_topic_;
    std::string command_topic_;

    double publish_rate_;
    double kp_;
    double default_velocity_limit_;
    double command_timeout_;

    bool use_command_gains_;
    double command_kp_scale_;
    double max_tracking_kp_;

    bool sync_to_gazebo_;
    std::string gazebo_model_name_;
    std::string gazebo_urdf_param_name_;
    std::string gazebo_set_configuration_service_;
    std::string gazebo_get_model_state_service_;
    double gazebo_sync_rate_;
    ros::WallTime last_gazebo_sync_wall_time_;
    bool gazebo_model_ready_;
    bool has_warned_waiting_for_gazebo_;

    std::vector<std::string> joint_names_;
    std::vector<JointSimState> joints_;
    std::map<std::string, size_t> joint_name_to_index_;

    bool has_command_;
    ros::Time last_command_time_;

    void loadRobotModel()
    {
        bool ok = false;

        if (!urdf_path_.empty())
        {
            ROS_INFO_STREAM("[arm_sim_node] Loading URDF from file: " << urdf_path_);
            ok = model_.initFile(urdf_path_);
        }
        else
        {
            std::string robot_description;
            ROS_INFO_STREAM("[arm_sim_node] Loading URDF from parameter: " << robot_description_param_);

            if (!nh_.getParam(robot_description_param_, robot_description))
            {
                ROS_FATAL_STREAM("[arm_sim_node] Failed to get parameter: " << robot_description_param_);
                ros::shutdown();
                return;
            }

            ok = model_.initString(robot_description);
        }

        if (!ok)
        {
            ROS_FATAL("[arm_sim_node] Failed to load URDF.");
            ros::shutdown();
            return;
        }

        ROS_INFO_STREAM("[arm_sim_node] Loaded robot model: " << model_.getName());
    }

    void loadJointNames()
    {
        joint_names_.clear();

        if (pnh_.hasParam("joint_names"))
        {
            XmlRpc::XmlRpcValue joint_list;
            pnh_.getParam("joint_names", joint_list);

            if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_FATAL("[arm_sim_node] joint_names must be a list.");
                ros::shutdown();
                return;
            }

            for (int i = 0; i < joint_list.size(); ++i)
            {
                joint_names_.push_back(static_cast<std::string>(joint_list[i]));
            }

            ROS_INFO("[arm_sim_node] Loaded joint_names from parameter.");
        }
        else
        {
            ROS_WARN("[arm_sim_node] joint_names not provided. Deriving movable joints from URDF.");
            deriveJointNamesFromUrdfTree();
        }

        if (joint_names_.empty())
        {
            ROS_FATAL("[arm_sim_node] No movable joints found.");
            ros::shutdown();
        }
    }

    void deriveJointNamesFromUrdfTree()
    {
        urdf::LinkConstSharedPtr root = model_.getRoot();
        if (!root)
        {
            ROS_FATAL("[arm_sim_node] URDF has no root link.");
            ros::shutdown();
            return;
        }

        traverseLink(root);
    }

    void traverseLink(const urdf::LinkConstSharedPtr& link)
    {
        if (!link)
        {
            return;
        }

        for (const auto& child_joint : link->child_joints)
        {
            if (!child_joint)
            {
                continue;
            }

            if (isActuatedJoint(child_joint->type))
            {
                joint_names_.push_back(child_joint->name);
            }

            if (!child_joint->child_link_name.empty())
            {
                traverseLink(model_.getLink(child_joint->child_link_name));
            }
        }
    }

    bool isActuatedJoint(int type) const
    {
        return type == urdf::Joint::REVOLUTE ||
               type == urdf::Joint::CONTINUOUS ||
               type == urdf::Joint::PRISMATIC;
    }

    void initializeJointsFromUrdf()
    {
        joints_.clear();
        joint_name_to_index_.clear();

        for (const std::string& name : joint_names_)
        {
            urdf::JointConstSharedPtr joint = model_.getJoint(name);

            if (!joint || !isActuatedJoint(joint->type))
            {
                ROS_FATAL_STREAM("[arm_sim_node] Invalid actuated joint: " << name);
                ros::shutdown();
                return;
            }

            JointSimState js;
            js.name = name;
            js.type = joint->type;
            js.is_continuous = (joint->type == urdf::Joint::CONTINUOUS);
            js.velocity_limit = default_velocity_limit_;

            if (joint->limits)
            {
                if (std::isfinite(joint->limits->velocity) &&
                    joint->limits->velocity > 0.0)
                {
                    js.velocity_limit = joint->limits->velocity;
                }

                if (joint->type == urdf::Joint::REVOLUTE ||
                    joint->type == urdf::Joint::PRISMATIC)
                {
                    js.lower = joint->limits->lower;
                    js.upper = joint->limits->upper;
                    js.has_position_limit =
                        std::isfinite(js.lower) &&
                        std::isfinite(js.upper) &&
                        js.lower < js.upper;
                }
            }

            joint_name_to_index_[js.name] = joints_.size();
            joints_.push_back(js);
        }
    }

    void loadInitialPositions()
    {
        if (!pnh_.hasParam("initial_positions"))
        {
            return;
        }

        XmlRpc::XmlRpcValue initial_positions;
        pnh_.getParam("initial_positions", initial_positions);

        if (initial_positions.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[arm_sim_node] initial_positions must be a dictionary. Ignoring it.");
            return;
        }

        for (auto& js : joints_)
        {
            if (initial_positions.hasMember(js.name))
            {
                const double q0 = static_cast<double>(initial_positions[js.name]);
                js.q = clampPosition(js, q0);
                js.q_ref = js.q;
            }
        }
    }

    void jointReferenceCallback(const arm_msgs::JointReference::ConstPtr& msg)
    {
        if (msg->name.empty() || msg->q_des.size() != msg->name.size())
        {
            ROS_WARN_THROTTLE(1.0, "[arm_sim_node] Invalid JointReference message. Ignoring.");
            return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto it = joint_name_to_index_.find(msg->name[i]);
            if (it == joint_name_to_index_.end())
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "[arm_sim_node] Unknown joint command: " << msg->name[i]);
                continue;
            }

            JointSimState& js = joints_[it->second];
            js.q_ref = clampPosition(js, msg->q_des[i]);
            js.dq_ref = (i < msg->dq_des.size()) ? clampVelocity(js, msg->dq_des[i]) : 0.0;
            js.kp_ref = (i < msg->kp.size()) ? msg->kp[i] : 0.0;
            js.kd_ref = (i < msg->kd.size()) ? msg->kd[i] : 0.0;
            js.tau_ff_ref = (i < msg->tau_ff.size()) ? msg->tau_ff[i] : 0.0;
        }

        has_command_ = true;
        last_command_time_ = ros::Time::now();
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        double dt = (event.current_real - event.last_real).toSec();

        if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1)
        {
            dt = 1.0 / publish_rate_;
        }

        integrate(dt);
        publishJointStates();
        syncGazeboIfDue();
    }

    void integrate(double dt)
    {
        const bool command_fresh =
            has_command_ &&
            ((ros::Time::now() - last_command_time_).toSec() <= command_timeout_);

        for (auto& js : joints_)
        {
            double qdot_cmd = 0.0;

            if (command_fresh)
            {
                double tracking_kp = kp_;
                if (use_command_gains_ && js.kp_ref > 0.0)
                {
                    tracking_kp = std::min(js.kp_ref * command_kp_scale_, max_tracking_kp_);
                }
                qdot_cmd = js.dq_ref + tracking_kp * positionError(js, js.q_ref, js.q);
            }
            else
            {
                qdot_cmd = kp_ * positionError(js, js.q_ref, js.q);
            }

            qdot_cmd = clampVelocity(js, qdot_cmd);

            const double q_next = clampPosition(js, js.q + qdot_cmd * dt);
            js.dq = (q_next - js.q) / dt;
            js.q = js.is_continuous ? normalizeAngle(q_next) : q_next;
        }
    }

    void publishJointStates()
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        for (const auto& js : joints_)
        {
            msg.name.push_back(js.name);
            msg.position.push_back(js.q);
            msg.velocity.push_back(js.dq);
            msg.effort.push_back(0.0);
        }

        joint_state_pub_.publish(msg);
    }

    bool checkGazeboModelReady()
    {
        if (gazebo_model_ready_)
        {
            return true;
        }

        if (!gazebo_get_model_state_client_.exists())
        {
            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "[arm_sim_node] Waiting for Gazebo service: "
                    << gazebo_get_model_state_service_);
            return false;
        }

        gazebo_msgs::GetModelState srv;
        srv.request.model_name = gazebo_model_name_;
        srv.request.relative_entity_name = "world";

        if (!gazebo_get_model_state_client_.call(srv))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[arm_sim_node] Failed to call Gazebo get_model_state."
            );
            return false;
        }

        if (!srv.response.success)
        {
            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "[arm_sim_node] Waiting for Gazebo model '"
                    << gazebo_model_name_ << "' to be spawned."
            );
            return false;
        }

        gazebo_model_ready_ = true;
        has_warned_waiting_for_gazebo_ = false;

        ROS_INFO_STREAM(
            "[arm_sim_node] Gazebo model '" << gazebo_model_name_
            << "' is ready. Starting model-configuration synchronization."
        );

        return true;
    }

    void syncGazeboIfDue()
    {
        if (!sync_to_gazebo_)
        {
            return;
        }

        const ros::WallTime now = ros::WallTime::now();
        if (!last_gazebo_sync_wall_time_.isZero() &&
            (now - last_gazebo_sync_wall_time_).toSec() < (1.0 / gazebo_sync_rate_))
        {
            return;
        }
        last_gazebo_sync_wall_time_ = now;

        if (!gazebo_set_configuration_client_.exists())
        {
            if (!has_warned_waiting_for_gazebo_)
            {
                ROS_WARN_STREAM("[arm_sim_node] Waiting for Gazebo service: "
                                << gazebo_set_configuration_service_);
                has_warned_waiting_for_gazebo_ = true;
            }
            return;
        }

        /*
         * /gazebo/set_model_configuration may already exist before the URDF
         * model has actually been inserted into Gazebo.  Do not send joint
         * configurations until the target model is confirmed to exist.
         */
        if (!checkGazeboModelReady())
        {
            return;
        }

        gazebo_msgs::SetModelConfiguration srv;
        srv.request.model_name = gazebo_model_name_;
        srv.request.urdf_param_name = gazebo_urdf_param_name_;

        for (const auto& js : joints_)
        {
            srv.request.joint_names.push_back(js.name);
            srv.request.joint_positions.push_back(js.q);
        }

        if (!gazebo_set_configuration_client_.call(srv))
        {
            ROS_WARN_THROTTLE(1.0, "[arm_sim_node] Failed to call Gazebo set_model_configuration.");
            gazebo_model_ready_ = false;
            return;
        }

        if (!srv.response.success)
        {
            ROS_WARN_STREAM_THROTTLE(1.0,
                                     "[arm_sim_node] Gazebo rejected joint configuration: "
                                     << srv.response.status_message);
            gazebo_model_ready_ = false;
            return;
        }

        has_warned_waiting_for_gazebo_ = false;
    }

    double clampPosition(const JointSimState& js, double q) const
    {
        if (js.is_continuous)
        {
            return normalizeAngle(q);
        }
        if (js.has_position_limit)
        {
            return std::max(js.lower, std::min(js.upper, q));
        }
        return q;
    }

    double clampVelocity(const JointSimState& js, double dq) const
    {
        const double limit =
            (std::isfinite(js.velocity_limit) && js.velocity_limit > 0.0)
                ? js.velocity_limit
                : default_velocity_limit_;
        return std::max(-limit, std::min(limit, dq));
    }

    double positionError(const JointSimState& js, double target, double current) const
    {
        return js.is_continuous ? shortestAngleDistance(current, target) : (target - current);
    }

    double normalizeAngle(double angle) const
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double shortestAngleDistance(double from, double to) const
    {
        return normalizeAngle(to - from);
    }

    void printSummary()
    {
        ROS_INFO("============================================================");
        ROS_INFO("[arm_sim_node] Simple kinematic arm simulator started.");
        ROS_INFO_STREAM("[arm_sim_node] Command topic: " << command_topic_);
        ROS_INFO_STREAM("[arm_sim_node] Joint states topic: " << joint_states_topic_);
        ROS_INFO_STREAM("[arm_sim_node] publish_rate: " << publish_rate_);
        ROS_INFO_STREAM("[arm_sim_node] Gazebo kinematic synchronization: "
                        << (sync_to_gazebo_ ? "enabled" : "disabled"));

        if (sync_to_gazebo_)
        {
            ROS_INFO_STREAM("[arm_sim_node] Gazebo model: " << gazebo_model_name_);
            ROS_INFO_STREAM("[arm_sim_node] Gazebo sync rate: " << gazebo_sync_rate_);
            ROS_INFO_STREAM("[arm_sim_node] Gazebo set-configuration service: "
                            << gazebo_set_configuration_service_);
            ROS_INFO_STREAM("[arm_sim_node] Gazebo model-state service: "
                            << gazebo_get_model_state_service_);
        }

        for (size_t i = 0; i < joints_.size(); ++i)
        {
            ROS_INFO_STREAM("  [" << i << "] " << joints_[i].name
                            << " q0=" << joints_[i].q
                            << " lower=" << joints_[i].lower
                            << " upper=" << joints_[i].upper
                            << " vel_limit=" << joints_[i].velocity_limit);
        }
        ROS_INFO("============================================================");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_sim_node");
    ArmSimNode node;
    ros::spin();
    return 0;
}
