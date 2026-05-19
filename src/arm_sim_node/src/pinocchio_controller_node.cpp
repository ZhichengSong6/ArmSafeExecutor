#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <arm_msgs/JointReference.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <XmlRpcValue.h>

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

class PinocchioControllerNode
{
public:
    PinocchioControllerNode()
        : nh_(),
          pnh_("~"),
          has_joint_state_(false),
          trajectory_active_(false)
    {
        pnh_.param<std::string>("robot_description_param", robot_description_param_, "/robot_description");
        pnh_.param<std::string>("urdf_path", urdf_path_, "");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<std::string>("ee_frame", ee_frame_, "EE_link");

        pnh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
        pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "/ee_target_pose");
        pnh_.param<std::string>("command_topic", command_topic_, "/arm_controller/joint_ref");

        pnh_.param<double>("control_rate", control_rate_, 100.0);
        pnh_.param<double>("trajectory_duration", trajectory_duration_, 2.0);

        pnh_.param<int>("ik_max_iters", ik_max_iters_, 100);

        pnh_.param<double>("ik_eps", ik_eps_, 1e-4);
        pnh_.param<double>("ik_position_eps", ik_position_eps_, ik_eps_);
        pnh_.param<double>("ik_orientation_eps", ik_orientation_eps_, 0.01);

        pnh_.param<double>("ik_damping", ik_damping_, 1e-3);
        pnh_.param<double>("ik_step_size", ik_step_size_, 0.5);
        pnh_.param<double>("ik_max_delta_q", ik_max_delta_q_, 0.08);

        pnh_.param<double>("orientation_weight", orientation_weight_, 0.5);

        pnh_.param<double>("default_kp", default_kp_, 20.0);
        pnh_.param<double>("default_kd", default_kd_, 1.0);
        pnh_.param<double>("default_tau_ff", default_tau_ff_, 0.0);

        loadRobotModel();
        loadJointNames();
        initializeVectors();

        joint_state_sub_ = nh_.subscribe(
            joint_states_topic_,
            10,
            &PinocchioControllerNode::jointStateCallback,
            this
        );

        target_pose_sub_ = nh_.subscribe(
            target_pose_topic_,
            10,
            &PinocchioControllerNode::targetPoseCallback,
            this
        );

        joint_ref_pub_ = nh_.advertise<arm_msgs::JointReference>(command_topic_, 10);

        ik_success_pub_ = nh_.advertise<std_msgs::Bool>("/debug/ik_success", 10, true);
        ik_error_pub_ = nh_.advertise<std_msgs::Float64>("/debug/ik_error_norm", 10, true);
        ik_position_error_pub_ = nh_.advertise<std_msgs::Float64>("/debug/ik_position_error_norm", 10, true);
        ik_orientation_error_pub_ = nh_.advertise<std_msgs::Float64>("/debug/ik_orientation_error_norm", 10, true);
        current_ee_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/debug/current_ee_pose", 10, true);

        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / control_rate_),
            &PinocchioControllerNode::controlTimerCallback,
            this
        );

        printSummary();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber joint_state_sub_;
    ros::Subscriber target_pose_sub_;

    ros::Publisher joint_ref_pub_;
    ros::Publisher ik_success_pub_;
    ros::Publisher ik_error_pub_;
    ros::Publisher ik_position_error_pub_;
    ros::Publisher ik_orientation_error_pub_;
    ros::Publisher current_ee_pub_;

    ros::Timer control_timer_;

    pinocchio::Model model_;
    pinocchio::Data data_;

    std::string robot_description_param_;
    std::string urdf_path_;
    std::string base_frame_;
    std::string ee_frame_;

    std::string joint_states_topic_;
    std::string target_pose_topic_;
    std::string command_topic_;

    std::vector<std::string> joint_names_;
    std::map<std::string, int> joint_name_to_index_;

    pinocchio::FrameIndex ee_frame_id_;

    Eigen::VectorXd q_current_;
    Eigen::VectorXd dq_current_;

    Eigen::VectorXd q_start_;
    Eigen::VectorXd q_goal_;

    Eigen::VectorXd q_command_;
    Eigen::VectorXd dq_command_;
    Eigen::VectorXd ddq_command_;

    bool has_joint_state_;
    bool trajectory_active_;

    ros::Time trajectory_start_time_;
    double trajectory_duration_;
    double control_rate_;

    int ik_max_iters_;
    double ik_eps_;
    double ik_position_eps_;
    double ik_orientation_eps_;
    double ik_damping_;
    double ik_step_size_;
    double ik_max_delta_q_;
    double orientation_weight_;

    double default_kp_;
    double default_kd_;
    double default_tau_ff_;

private:
    void loadRobotModel()
    {
        bool loaded = false;

        if (!urdf_path_.empty())
        {
            ROS_INFO_STREAM("[pinocchio_controller] Loading URDF from file: " << urdf_path_);
            pinocchio::urdf::buildModel(urdf_path_, model_);
            loaded = true;
        }
        else
        {
            std::string robot_description;

            if (!nh_.getParam(robot_description_param_, robot_description))
            {
                ROS_FATAL_STREAM("[pinocchio_controller] Failed to get parameter: "
                                 << robot_description_param_);
                ros::shutdown();
                return;
            }

            ROS_INFO_STREAM("[pinocchio_controller] Loading URDF from parameter: "
                            << robot_description_param_);
            pinocchio::urdf::buildModelFromXML(robot_description, model_);
            loaded = true;
        }

        if (!loaded)
        {
            ROS_FATAL("[pinocchio_controller] Failed to load Pinocchio model.");
            ros::shutdown();
            return;
        }

        data_ = pinocchio::Data(model_);

        if (!model_.existFrame(ee_frame_))
        {
            ROS_FATAL_STREAM("[pinocchio_controller] EE frame not found in Pinocchio model: "
                             << ee_frame_);
            ros::shutdown();
            return;
        }

        ee_frame_id_ = model_.getFrameId(ee_frame_);

        ROS_INFO_STREAM("[pinocchio_controller] Loaded model: " << model_.name);
        ROS_INFO_STREAM("[pinocchio_controller] model.nq = " << model_.nq
                        << ", model.nv = " << model_.nv);
    }

    void loadJointNames()
    {
        if (!pnh_.hasParam("joint_names"))
        {
            ROS_FATAL("[pinocchio_controller] Please provide joint_names parameter.");
            ros::shutdown();
            return;
        }

        XmlRpc::XmlRpcValue joint_list;
        pnh_.getParam("joint_names", joint_list);

        if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("[pinocchio_controller] joint_names must be a list.");
            ros::shutdown();
            return;
        }

        joint_names_.clear();

        for (int i = 0; i < joint_list.size(); ++i)
        {
            std::string name = static_cast<std::string>(joint_list[i]);
            joint_names_.push_back(name);
        }

        if (static_cast<int>(joint_names_.size()) != model_.nq)
        {
            ROS_WARN_STREAM("[pinocchio_controller] joint_names size = "
                            << joint_names_.size()
                            << ", but model.nq = " << model_.nq
                            << ". This is okay only if your robot has special joints. "
                            << "For this arm, they should match.");
        }

        joint_name_to_index_.clear();

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const std::string& joint_name = joint_names_[i];

            if (!model_.existJointName(joint_name))
            {
                ROS_FATAL_STREAM("[pinocchio_controller] Joint not found in Pinocchio model: "
                                 << joint_name);
                ros::shutdown();
                return;
            }

            pinocchio::JointIndex jid = model_.getJointId(joint_name);
            int q_index = model_.idx_qs[jid];

            joint_name_to_index_[joint_name] = q_index;

            ROS_INFO_STREAM("[pinocchio_controller] joint " << joint_name
                            << " -> Pinocchio q index " << q_index);
        }
    }

    void initializeVectors()
    {
        q_current_ = Eigen::VectorXd::Zero(model_.nq);
        dq_current_ = Eigen::VectorXd::Zero(model_.nv);

        q_start_ = Eigen::VectorXd::Zero(model_.nq);
        q_goal_ = Eigen::VectorXd::Zero(model_.nq);

        q_command_ = Eigen::VectorXd::Zero(model_.nq);
        dq_command_ = Eigen::VectorXd::Zero(model_.nv);
        ddq_command_ = Eigen::VectorXd::Zero(model_.nv);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = joint_name_to_index_.find(msg->name[i]);
            if (it == joint_name_to_index_.end())
            {
                continue;
            }

            int idx = it->second;

            if (idx >= 0 && idx < q_current_.size() && i < msg->position.size())
            {
                q_current_[idx] = msg->position[i];
            }

            if (idx >= 0 && idx < dq_current_.size() && i < msg->velocity.size())
            {
                dq_current_[idx] = msg->velocity[i];
            }
        }

        has_joint_state_ = true;
        publishCurrentEePose();
    }

    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!has_joint_state_)
        {
            ROS_WARN("[pinocchio_controller] No joint state received yet. Ignore target pose.");
            return;
        }

        if (msg->header.frame_id != base_frame_)
        {
            ROS_WARN_STREAM("[pinocchio_controller] Target pose frame is "
                            << msg->header.frame_id
                            << ", expected " << base_frame_
                            << ". For now, this node assumes target pose is in base_frame.");
            return;
        }

        Eigen::Vector3d target_position(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        Eigen::Quaterniond target_quat(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );

        if (target_quat.norm() < 1e-9)
        {
            ROS_WARN("[pinocchio_controller] Target orientation quaternion has near-zero norm. Ignore target.");
            return;
        }

        target_quat.normalize();
        Eigen::Matrix3d target_rotation = target_quat.toRotationMatrix();

        Eigen::VectorXd q_solution = q_current_;

        double final_weighted_error = 0.0;
        double final_position_error = 0.0;
        double final_orientation_error = 0.0;

        bool success = solvePoseIK(
            q_current_,
            target_position,
            target_rotation,
            q_solution,
            final_weighted_error,
            final_position_error,
            final_orientation_error
        );

        std_msgs::Bool success_msg;
        success_msg.data = success;
        ik_success_pub_.publish(success_msg);

        std_msgs::Float64 weighted_err_msg;
        weighted_err_msg.data = final_weighted_error;
        ik_error_pub_.publish(weighted_err_msg);

        std_msgs::Float64 pos_err_msg;
        pos_err_msg.data = final_position_error;
        ik_position_error_pub_.publish(pos_err_msg);

        std_msgs::Float64 ori_err_msg;
        ori_err_msg.data = final_orientation_error;
        ik_orientation_error_pub_.publish(ori_err_msg);

        if (!success)
        {
            ROS_WARN_STREAM("[pinocchio_controller] 6D IK failed. "
                            << "weighted_error = " << final_weighted_error
                            << ", position_error = " << final_position_error
                            << ", orientation_error = " << final_orientation_error);
            return;
        }

        ROS_INFO_STREAM("[pinocchio_controller] 6D IK success. "
                        << "weighted_error = " << final_weighted_error
                        << ", position_error = " << final_position_error
                        << ", orientation_error = " << final_orientation_error);

        startJointTrajectory(q_current_, q_solution);
    }

    bool solvePoseIK(
        const Eigen::VectorXd& q_init,
        const Eigen::Vector3d& target_position,
        const Eigen::Matrix3d& target_rotation,
        Eigen::VectorXd& q_solution,
        double& final_weighted_error,
        double& final_position_error,
        double& final_orientation_error)
    {
        Eigen::VectorXd q = q_init;

        for (int iter = 0; iter < ik_max_iters_; ++iter)
        {
            pinocchio::SE3 current_pose = computeEePose(q);

            Eigen::Matrix<double, 6, 1> error_6d;
            computePoseError(
                current_pose,
                target_position,
                target_rotation,
                error_6d,
                final_position_error,
                final_orientation_error
            );

            Eigen::Matrix<double, 6, 1> weighted_error = error_6d;
            weighted_error.tail<3>() *= orientation_weight_;

            final_weighted_error = weighted_error.norm();

            if (final_position_error < ik_position_eps_ &&
                final_orientation_error < ik_orientation_eps_)
            {
                q_solution = q;
                clampToPositionLimits(q_solution);
                return true;
            }

            Eigen::MatrixXd J = computePinocchioPoseJacobian(q);

            Eigen::MatrixXd weighted_J = J;
            weighted_J.bottomRows(3) *= orientation_weight_;

            Eigen::MatrixXd A =
                weighted_J * weighted_J.transpose()
                + ik_damping_ * ik_damping_ * Eigen::MatrixXd::Identity(6, 6);

            Eigen::VectorXd dq =
                weighted_J.transpose() * A.ldlt().solve(weighted_error);

            if (!dq.allFinite())
            {
                ROS_WARN("[pinocchio_controller] IK produced non-finite dq. Abort IK.");
                q_solution = q;
                return false;
            }

            if (dq.norm() > ik_max_delta_q_)
            {
                dq = dq / dq.norm() * ik_max_delta_q_;
            }

            q = q + ik_step_size_ * dq;
            clampToPositionLimits(q);
        }

        pinocchio::SE3 final_pose = computeEePose(q);

        Eigen::Matrix<double, 6, 1> final_error_6d;
        computePoseError(
            final_pose,
            target_position,
            target_rotation,
            final_error_6d,
            final_position_error,
            final_orientation_error
        );

        Eigen::Matrix<double, 6, 1> weighted_final_error = final_error_6d;
        weighted_final_error.tail<3>() *= orientation_weight_;
        final_weighted_error = weighted_final_error.norm();

        q_solution = q;
        clampToPositionLimits(q_solution);

        return final_position_error < ik_position_eps_ &&
               final_orientation_error < ik_orientation_eps_;
    }

    void computePoseError(
        const pinocchio::SE3& current_pose,
        const Eigen::Vector3d& target_position,
        const Eigen::Matrix3d& target_rotation,
        Eigen::Matrix<double, 6, 1>& error_6d,
        double& position_error_norm,
        double& orientation_error_norm) const
    {
        Eigen::Vector3d position_error =
            target_position - current_pose.translation();

        Eigen::Matrix3d rotation_error_matrix =
            target_rotation * current_pose.rotation().transpose();

        Eigen::Vector3d orientation_error =
            rotationMatrixLog(rotation_error_matrix);

        error_6d.head<3>() = position_error;
        error_6d.tail<3>() = orientation_error;

        position_error_norm = position_error.norm();
        orientation_error_norm = orientation_error.norm();
    }

    pinocchio::SE3 computeEePose(const Eigen::VectorXd& q)
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        return data_.oMf[ee_frame_id_];
    }

    Eigen::MatrixXd computePinocchioPoseJacobian(const Eigen::VectorXd& q)
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        Eigen::MatrixXd J6(6, model_.nv);
        J6.setZero();

        pinocchio::getFrameJacobian(
            model_,
            data_,
            ee_frame_id_,
            pinocchio::LOCAL_WORLD_ALIGNED,
            J6
        );

        return J6;
    }

    Eigen::Vector3d rotationMatrixLog(const Eigen::Matrix3d& R) const
    {
        Eigen::AngleAxisd angle_axis(R);

        double angle = angle_axis.angle();

        if (!std::isfinite(angle) || std::abs(angle) < 1e-12)
        {
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d axis = angle_axis.axis();

        if (!axis.allFinite())
        {
            return Eigen::Vector3d::Zero();
        }

        return axis * angle;
    }

    void clampToPositionLimits(Eigen::VectorXd& q)
    {
        for (int i = 0; i < model_.nq; ++i)
        {
            double lower = model_.lowerPositionLimit[i];
            double upper = model_.upperPositionLimit[i];

            if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
            {
                q[i] = std::max(lower, std::min(q[i], upper));
            }
        }
    }

    void startJointTrajectory(
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal)
    {
        q_start_ = q_start;
        q_goal_ = q_goal;

        trajectory_start_time_ = ros::Time::now();
        trajectory_active_ = true;

        ROS_INFO("[pinocchio_controller] Started quintic joint-space trajectory.");
    }

    void controlTimerCallback(const ros::TimerEvent&)
    {
        if (!has_joint_state_)
        {
            return;
        }

        if (!trajectory_active_)
        {
            return;
        }

        double t = (ros::Time::now() - trajectory_start_time_).toSec();
        double tau = t / trajectory_duration_;

        if (tau >= 1.0)
        {
            tau = 1.0;
            trajectory_active_ = false;
        }

        double s = quinticTimeScaling(tau);
        double ds_dt = quinticTimeScalingDot(tau) / trajectory_duration_;
        double dds_dt2 = quinticTimeScalingDDot(tau) /
                         (trajectory_duration_ * trajectory_duration_);

        Eigen::VectorXd delta_q = q_goal_ - q_start_;

        q_command_ = q_start_ + s * delta_q;
        dq_command_ = ds_dt * delta_q;
        ddq_command_ = dds_dt2 * delta_q;

        publishJointReference(q_command_, dq_command_);

        if (!trajectory_active_)
        {
            ROS_INFO("[pinocchio_controller] Joint-space trajectory finished.");
        }
    }

    double quinticTimeScaling(double tau) const
    {
        tau = std::max(0.0, std::min(1.0, tau));

        return 10.0 * std::pow(tau, 3)
             - 15.0 * std::pow(tau, 4)
             + 6.0 * std::pow(tau, 5);
    }

    double quinticTimeScalingDot(double tau) const
    {
        tau = std::max(0.0, std::min(1.0, tau));

        return 30.0 * std::pow(tau, 2)
             - 60.0 * std::pow(tau, 3)
             + 30.0 * std::pow(tau, 4);
    }

    double quinticTimeScalingDDot(double tau) const
    {
        tau = std::max(0.0, std::min(1.0, tau));

        return 60.0 * tau
             - 180.0 * std::pow(tau, 2)
             + 120.0 * std::pow(tau, 3);
    }

    void publishJointReference(
        const Eigen::VectorXd& q_cmd,
        const Eigen::VectorXd& dq_cmd)
    {
        arm_msgs::JointReference msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame_;

        msg.name = joint_names_;

        msg.q_des.resize(joint_names_.size());
        msg.dq_des.resize(joint_names_.size());
        msg.kp.resize(joint_names_.size());
        msg.kd.resize(joint_names_.size());
        msg.tau_ff.resize(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            pinocchio::JointIndex jid = model_.getJointId(joint_names_[i]);
            int q_idx = model_.idx_qs[jid];

            msg.q_des[i] = q_cmd[q_idx];
            msg.dq_des[i] = dq_cmd[q_idx];

            msg.kp[i] = default_kp_;
            msg.kd[i] = default_kd_;
            msg.tau_ff[i] = default_tau_ff_;
        }

        joint_ref_pub_.publish(msg);
    }

    void publishCurrentEePose()
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame_;

        pinocchio::SE3 oMee = computeEePose(q_current_);

        msg.pose.position.x = oMee.translation().x();
        msg.pose.position.y = oMee.translation().y();
        msg.pose.position.z = oMee.translation().z();

        Eigen::Quaterniond quat(oMee.rotation());
        quat.normalize();

        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
        msg.pose.orientation.w = quat.w();

        current_ee_pub_.publish(msg);
    }

    void printSummary()
    {
        ROS_INFO("============================================================");
        ROS_INFO("[pinocchio_controller] Started.");
        ROS_INFO_STREAM("[pinocchio_controller] base_frame: " << base_frame_);
        ROS_INFO_STREAM("[pinocchio_controller] ee_frame: " << ee_frame_);
        ROS_INFO_STREAM("[pinocchio_controller] target_pose_topic: " << target_pose_topic_);
        ROS_INFO_STREAM("[pinocchio_controller] command_topic: " << command_topic_);
        ROS_INFO_STREAM("[pinocchio_controller] trajectory_duration: " << trajectory_duration_);
        ROS_INFO_STREAM("[pinocchio_controller] control_rate: " << control_rate_);
        ROS_INFO_STREAM("[pinocchio_controller] IK max iters: " << ik_max_iters_);
        ROS_INFO_STREAM("[pinocchio_controller] ik_position_eps: " << ik_position_eps_);
        ROS_INFO_STREAM("[pinocchio_controller] ik_orientation_eps: " << ik_orientation_eps_);
        ROS_INFO_STREAM("[pinocchio_controller] orientation_weight: " << orientation_weight_);
        ROS_INFO_STREAM("[pinocchio_controller] ik_damping: " << ik_damping_);
        ROS_INFO_STREAM("[pinocchio_controller] ik_step_size: " << ik_step_size_);
        ROS_INFO_STREAM("[pinocchio_controller] ik_max_delta_q: " << ik_max_delta_q_);
        ROS_INFO_STREAM("[pinocchio_controller] Jacobian: Pinocchio LOCAL_WORLD_ALIGNED frame Jacobian");
        ROS_INFO_STREAM("[pinocchio_controller] default_kp: " << default_kp_);
        ROS_INFO_STREAM("[pinocchio_controller] default_kd: " << default_kd_);
        ROS_INFO_STREAM("[pinocchio_controller] default_tau_ff: " << default_tau_ff_);
        ROS_INFO("============================================================");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pinocchio_controller_node");

    PinocchioControllerNode node;

    ros::spin();

    return 0;
}