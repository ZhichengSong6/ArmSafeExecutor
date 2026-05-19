#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
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
          has_command_(false)
    {
        pnh_.param<std::string>("robot_description_param", robot_description_param_, "/robot_description");
        pnh_.param<std::string>("urdf_path", urdf_path_, "");

        pnh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
        pnh_.param<std::string>("command_topic", command_topic_, "/arm_controller/joint_ref");

        pnh_.param<double>("publish_rate", publish_rate_, 100.0);
        pnh_.param<double>("kp", kp_, 5.0);
        pnh_.param<double>("default_velocity_limit", default_velocity_limit_, 0.5);
        pnh_.param<double>("command_timeout", command_timeout_, 0.5);

        // If false, the simulator uses the node parameter kp_ as the tracking gain.
        // If true, it uses msg.kp with scaling/clamping below.
        pnh_.param<bool>("use_command_gains", use_command_gains_, false);
        pnh_.param<double>("command_kp_scale", command_kp_scale_, 1.0);
        pnh_.param<double>("max_tracking_kp", max_tracking_kp_, 20.0);

        loadRobotModel();
        loadJointNames();
        initializeJointsFromUrdf();
        loadInitialPositions();

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 10);

        joint_ref_sub_ = nh_.subscribe(
            command_topic_,
            10,
            &ArmSimNode::jointReferenceCallback,
            this
        );

        sim_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &ArmSimNode::timerCallback,
            this
        );

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

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher joint_state_pub_;
    ros::Subscriber joint_ref_sub_;
    ros::Timer sim_timer_;

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

    std::vector<std::string> joint_names_;
    std::vector<JointSimState> joints_;
    std::map<std::string, size_t> joint_name_to_index_;

    bool has_command_;
    ros::Time last_command_time_;

private:
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
                ROS_FATAL_STREAM("[arm_sim_node] Failed to get parameter: "
                                 << robot_description_param_);
                ros::shutdown();
                return;
            }

            ok = model_.initString(robot_description);
        }

        if (!ok)
        {
            ROS_FATAL("[arm_sim_node] Failed to load URDF. Please check robot_description or urdf_path.");
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
                std::string name = static_cast<std::string>(joint_list[i]);
                joint_names_.push_back(name);
            }

            ROS_INFO("[arm_sim_node] Loaded joint_names from parameter.");
        }
        else
        {
            ROS_WARN("[arm_sim_node] joint_names not provided. Deriving movable joints from URDF tree.");
            deriveJointNamesFromUrdfTree();
        }

        if (joint_names_.empty())
        {
            ROS_FATAL("[arm_sim_node] No movable joints found.");
            ros::shutdown();
            return;
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
                urdf::LinkConstSharedPtr child_link = model_.getLink(child_joint->child_link_name);
                traverseLink(child_link);
            }
        }
    }

    bool isActuatedJoint(int joint_type) const
    {
        return joint_type == urdf::Joint::REVOLUTE ||
               joint_type == urdf::Joint::CONTINUOUS ||
               joint_type == urdf::Joint::PRISMATIC;
    }

    void initializeJointsFromUrdf()
    {
        joints_.clear();
        joint_name_to_index_.clear();

        for (const std::string& joint_name : joint_names_)
        {
            urdf::JointConstSharedPtr urdf_joint = model_.getJoint(joint_name);

            if (!urdf_joint)
            {
                ROS_FATAL_STREAM("[arm_sim_node] Joint not found in URDF: " << joint_name);
                ros::shutdown();
                return;
            }

            if (!isActuatedJoint(urdf_joint->type))
            {
                ROS_FATAL_STREAM("[arm_sim_node] Joint is not actuated: " << joint_name);
                ros::shutdown();
                return;
            }

            JointSimState js;
            js.name = joint_name;
            js.type = urdf_joint->type;
            js.is_continuous = (urdf_joint->type == urdf::Joint::CONTINUOUS);

            js.q = 0.0;
            js.dq = 0.0;
            js.q_ref = 0.0;
            js.dq_ref = 0.0;

            js.velocity_limit = default_velocity_limit_;

            if (urdf_joint->limits)
            {
                if (std::isfinite(urdf_joint->limits->velocity) &&
                    urdf_joint->limits->velocity > 0.0)
                {
                    js.velocity_limit = urdf_joint->limits->velocity;
                }

                if (urdf_joint->type == urdf::Joint::REVOLUTE ||
                    urdf_joint->type == urdf::Joint::PRISMATIC)
                {
                    js.lower = urdf_joint->limits->lower;
                    js.upper = urdf_joint->limits->upper;

                    if (std::isfinite(js.lower) &&
                        std::isfinite(js.upper) &&
                        js.lower < js.upper)
                    {
                        js.has_position_limit = true;
                    }
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
            ROS_WARN("[arm_sim_node] initial_positions should be a dictionary. Ignoring it.");
            return;
        }

        for (auto& js : joints_)
        {
            if (!initial_positions.hasMember(js.name))
            {
                continue;
            }

            double q0 = static_cast<double>(initial_positions[js.name]);
            js.q = clampPosition(js, q0);
            js.q_ref = js.q;
        }
    }

    void jointReferenceCallback(const arm_msgs::JointReference::ConstPtr& msg)
    {
        if (msg->name.empty())
        {
            ROS_WARN_THROTTLE(1.0, "[arm_sim_node] Received empty JointReference.name. Ignoring.");
            return;
        }

        if (msg->q_des.size() != msg->name.size())
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[arm_sim_node] JointReference q_des size mismatch. name.size = "
                    << msg->name.size()
                    << ", q_des.size = " << msg->q_des.size()
                    << ". Ignoring."
            );
            return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = joint_name_to_index_.find(msg->name[i]);
            if (it == joint_name_to_index_.end())
            {
                ROS_WARN_STREAM_THROTTLE(
                    1.0,
                    "[arm_sim_node] Received command for unknown joint: " << msg->name[i]
                );
                continue;
            }

            JointSimState& js = joints_[it->second];

            js.q_ref = clampPosition(js, msg->q_des[i]);

            if (i < msg->dq_des.size())
            {
                js.dq_ref = clampVelocity(js, msg->dq_des[i]);
            }
            else
            {
                js.dq_ref = 0.0;
            }

            if (i < msg->kp.size())
            {
                js.kp_ref = msg->kp[i];
            }

            if (i < msg->kd.size())
            {
                js.kd_ref = msg->kd[i];
            }

            if (i < msg->tau_ff.size())
            {
                js.tau_ff_ref = msg->tau_ff[i];
            }
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
    }

    void integrate(double dt)
    {
        const ros::Time now = ros::Time::now();
        const bool command_fresh =
            has_command_ &&
            ((now - last_command_time_).toSec() <= command_timeout_);

        for (auto& js : joints_)
        {
            double qdot_cmd = 0.0;

            if (command_fresh)
            {
                const double err = positionError(js, js.q_ref, js.q);

                double tracking_kp = kp_;

                if (use_command_gains_ && js.kp_ref > 0.0)
                {
                    tracking_kp = std::min(js.kp_ref * command_kp_scale_, max_tracking_kp_);
                }

                qdot_cmd = js.dq_ref + tracking_kp * err;
            }
            else
            {
                // Hold current target if command stream stops.
                const double err = positionError(js, js.q_ref, js.q);
                qdot_cmd = kp_ * err;
            }

            qdot_cmd = clampVelocity(js, qdot_cmd);

            double q_next = js.q + qdot_cmd * dt;
            q_next = clampPosition(js, q_next);

            js.dq = (q_next - js.q) / dt;
            js.q = q_next;

            if (js.is_continuous)
            {
                js.q = normalizeAngle(js.q);
            }
        }
    }

    void publishJointStates()
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        msg.name.resize(joints_.size());
        msg.position.resize(joints_.size());
        msg.velocity.resize(joints_.size());
        msg.effort.resize(joints_.size());

        for (size_t i = 0; i < joints_.size(); ++i)
        {
            msg.name[i] = joints_[i].name;
            msg.position[i] = joints_[i].q;
            msg.velocity[i] = joints_[i].dq;
            msg.effort[i] = 0.0;
        }

        joint_state_pub_.publish(msg);
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
        const double v_lim =
            (std::isfinite(js.velocity_limit) && js.velocity_limit > 0.0)
                ? js.velocity_limit
                : default_velocity_limit_;

        return std::max(-v_lim, std::min(v_lim, dq));
    }

    double positionError(const JointSimState& js, double q_target, double q_current) const
    {
        if (js.is_continuous)
        {
            return shortestAngleDistance(q_current, q_target);
        }

        return q_target - q_current;
    }

    double normalizeAngle(double angle) const
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    double shortestAngleDistance(double from, double to) const
    {
        return normalizeAngle(to - from);
    }

    std::string jointTypeToString(int type) const
    {
        switch (type)
        {
            case urdf::Joint::REVOLUTE:
                return "REVOLUTE";
            case urdf::Joint::CONTINUOUS:
                return "CONTINUOUS";
            case urdf::Joint::PRISMATIC:
                return "PRISMATIC";
            default:
                return "UNKNOWN";
        }
    }

    void printSummary()
    {
        ROS_INFO("============================================================");
        ROS_INFO("[arm_sim_node] Simple kinematic arm simulator started.");
        ROS_INFO_STREAM("[arm_sim_node] Robot name: " << model_.getName());
        ROS_INFO_STREAM("[arm_sim_node] Number of movable joints: " << joints_.size());
        ROS_INFO_STREAM("[arm_sim_node] Command topic: " << command_topic_);
        ROS_INFO_STREAM("[arm_sim_node] Joint states topic: " << joint_states_topic_);
        ROS_INFO_STREAM("[arm_sim_node] Publish rate: " << publish_rate_);
        ROS_INFO_STREAM("[arm_sim_node] kp: " << kp_);
        ROS_INFO_STREAM("[arm_sim_node] use_command_gains: " << (use_command_gains_ ? "true" : "false"));

        for (size_t i = 0; i < joints_.size(); ++i)
        {
            const auto& js = joints_[i];

            ROS_INFO_STREAM("  [" << i << "] "
                            << js.name
                            << " type=" << jointTypeToString(js.type)
                            << " q0=" << js.q
                            << " lower=" << js.lower
                            << " upper=" << js.upper
                            << " vel_limit=" << js.velocity_limit);
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