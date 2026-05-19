#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>

#include <string>
#include <memory>

class EndEffectorInteractiveMarker
{
public:
    EndEffectorInteractiveMarker()
        : nh_(),
          pnh_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          initialized_(false)
    {
        pnh_.param<std::string>("server_name", server_name_, "ee_interactive_marker");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<std::string>("ee_frame", ee_frame_, "EE_link");
        pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "/ee_target_pose");

        pnh_.param<double>("marker_scale", marker_scale_, 0.08);
        pnh_.param<double>("fallback_x", fallback_x_, 0.3);
        pnh_.param<double>("fallback_y", fallback_y_, 0.0);
        pnh_.param<double>("fallback_z", fallback_z_, 0.3);

        target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            target_pose_topic_,
            10,
            true
        );

        server_.reset(new interactive_markers::InteractiveMarkerServer(server_name_));

        init_timer_ = nh_.createTimer(
            ros::Duration(0.2),
            &EndEffectorInteractiveMarker::initTimerCallback,
            this
        );

        ROS_INFO_STREAM("[interactive_marker_node] Waiting for TF "
                        << base_frame_ << " -> " << ee_frame_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher target_pose_pub_;
    ros::Timer init_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;

    std::string server_name_;
    std::string base_frame_;
    std::string ee_frame_;
    std::string target_pose_topic_;

    double marker_scale_;
    double fallback_x_;
    double fallback_y_;
    double fallback_z_;

    bool initialized_;

private:
    void initTimerCallback(const ros::TimerEvent&)
    {
        if (initialized_)
        {
            return;
        }

        geometry_msgs::Pose initial_pose;

        if (lookupCurrentEePose(initial_pose))
        {
            ROS_INFO_STREAM("[interactive_marker_node] Initialized marker from TF: "
                            << base_frame_ << " -> " << ee_frame_);
        }
        else
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[interactive_marker_node] Cannot lookup TF "
                    << base_frame_ << " -> " << ee_frame_
                    << ". Using fallback pose."
            );

            initial_pose.position.x = fallback_x_;
            initial_pose.position.y = fallback_y_;
            initial_pose.position.z = fallback_z_;
            initial_pose.orientation.x = 0.0;
            initial_pose.orientation.y = 0.0;
            initial_pose.orientation.z = 0.0;
            initial_pose.orientation.w = 1.0;
        }

        normalizePoseQuaternion(initial_pose);

        makeMarker(initial_pose);

        initialized_ = true;
        init_timer_.stop();

        ROS_INFO_STREAM("[interactive_marker_node] Interactive marker server: "
                        << server_name_);
        ROS_INFO_STREAM("[interactive_marker_node] Drag marker to desired pose, "
                        << "then click the box to publish goal.");
        ROS_INFO_STREAM("[interactive_marker_node] Publishing target pose to: "
                        << target_pose_topic_);
    }

    bool lookupCurrentEePose(geometry_msgs::Pose& pose_out)
    {
        try
        {
            geometry_msgs::TransformStamped tf_msg =
                tf_buffer_.lookupTransform(
                    base_frame_,
                    ee_frame_,
                    ros::Time(0),
                    ros::Duration(0.1)
                );

            pose_out.position.x = tf_msg.transform.translation.x;
            pose_out.position.y = tf_msg.transform.translation.y;
            pose_out.position.z = tf_msg.transform.translation.z;
            pose_out.orientation = tf_msg.transform.rotation;

            normalizePoseQuaternion(pose_out);

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            return false;
        }
    }

    void makeMarker(const geometry_msgs::Pose& initial_pose)
    {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = base_frame_;
        int_marker.header.stamp = ros::Time::now();
        int_marker.name = "ee_target_marker";
        int_marker.description = "EE Target: drag, then click box to send goal";
        int_marker.scale = marker_scale_ * 4.0;
        int_marker.pose = initial_pose;

        addGoalBoxControl(int_marker);
        add6DofControls(int_marker);

        server_->insert(
            int_marker,
            boost::bind(&EndEffectorInteractiveMarker::processFeedback, this, _1)
        );

        server_->applyChanges();
    }

    void addGoalBoxControl(visualization_msgs::InteractiveMarker& int_marker)
    {
        visualization_msgs::Marker box;
        box.type = visualization_msgs::Marker::CUBE;

        box.scale.x = marker_scale_ * 1.2;
        box.scale.y = marker_scale_ * 1.2;
        box.scale.z = marker_scale_ * 1.2;

        box.color.r = 0.1;
        box.color.g = 0.8;
        box.color.b = 0.2;
        box.color.a = 0.85;

        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.name = "send_goal_button";
        box_control.description = "Click box to send EE goal";
        box_control.always_visible = true;
        box_control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::BUTTON;
        box_control.markers.push_back(box);

        int_marker.controls.push_back(box_control);
    }

    void add6DofControls(visualization_msgs::InteractiveMarker& int_marker)
    {
        visualization_msgs::InteractiveMarkerControl control;

        // X axis controls
        control.orientation.w = 0.70710678;
        control.orientation.x = 0.70710678;
        control.orientation.y = 0.0;
        control.orientation.z = 0.0;
        control.name = "rotate_x";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_x";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // Y axis controls
        control.orientation.w = 0.70710678;
        control.orientation.x = 0.0;
        control.orientation.y = 0.70710678;
        control.orientation.z = 0.0;
        control.name = "rotate_y";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_y";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // Z axis controls
        control.orientation.w = 0.70710678;
        control.orientation.x = 0.0;
        control.orientation.y = 0.0;
        control.orientation.z = 0.70710678;
        control.name = "rotate_z";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.name = "move_z";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        if (feedback->event_type ==
            visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            geometry_msgs::Pose goal_pose = feedback->pose;
            normalizePoseQuaternion(goal_pose);

            ROS_INFO_STREAM("[interactive_marker_node] Send EE goal by box click. "
                            << "position = ["
                            << goal_pose.position.x << ", "
                            << goal_pose.position.y << ", "
                            << goal_pose.position.z << "]");

            publishTargetPose(goal_pose);
            return;
        }

        // Important:
        // We intentionally do NOT publish goal on POSE_UPDATE or MOUSE_UP.
        // Dragging only changes the marker pose. Clicking the box sends the goal.
    }

    void publishTargetPose(const geometry_msgs::Pose& pose)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame_;
        msg.pose = pose;

        target_pose_pub_.publish(msg);
    }

    void normalizePoseQuaternion(geometry_msgs::Pose& pose)
    {
        double x = pose.orientation.x;
        double y = pose.orientation.y;
        double z = pose.orientation.z;
        double w = pose.orientation.w;

        double norm = std::sqrt(x * x + y * y + z * z + w * w);

        if (norm < 1e-9)
        {
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            return;
        }

        pose.orientation.x /= norm;
        pose.orientation.y /= norm;
        pose.orientation.z /= norm;
        pose.orientation.w /= norm;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interactive_marker_node");

    EndEffectorInteractiveMarker node;

    ros::spin();

    return 0;
}