#include <ros/ros.h>

#include <arm_msgs/JointReference.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/bind/bind.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class PointcloudQpSafetyFilterNode
{
public:
    using CloudMsg = sensor_msgs::PointCloud2;
    using CloudSyncPolicy =
        message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
    using CloudSynchronizer =
        message_filters::Synchronizer<CloudSyncPolicy>;

    PointcloudQpSafetyFilterNode()
        : nh_(),
          pnh_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeCommandPassThrough();
        initializePointCloudFusionBranch();
        printStartupInformation();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ================================================================
    // Command branch:
    //
    // /arm_controller/joint_ref_nominal
    //          ↓
    // current pass-through behavior
    //          ↓
    // /arm_controller/joint_ref
    // ================================================================
    ros::Subscriber nominal_command_sub_;
    ros::Publisher safe_command_pub_;

    std::string nominal_command_topic_;
    std::string safe_command_topic_;

    // ================================================================
    // Point-cloud branch:
    //
    // /link4_sensor1/tof/cloud + /link4_sensor2/tof/cloud
    //          ↓
    // approximate time synchronization
    //          ↓
    // transform into base_link
    //          ↓
    // publish two transformed clouds and fused cloud
    // ================================================================
    message_filters::Subscriber<CloudMsg> cloud_1_sub_;
    message_filters::Subscriber<CloudMsg> cloud_2_sub_;
    std::shared_ptr<CloudSynchronizer> cloud_sync_;

    ros::Publisher cloud_1_base_pub_;
    ros::Publisher cloud_2_base_pub_;
    ros::Publisher fused_cloud_pub_;

    std::string cloud_topic_1_;
    std::string cloud_topic_2_;

    std::string cloud_1_base_topic_;
    std::string cloud_2_base_topic_;
    std::string fused_cloud_topic_;

    std::string target_frame_;

    int cloud_sub_queue_size_;
    int cloud_sync_queue_size_;

    double cloud_sync_slop_;
    double tf_timeout_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void loadParameters()
    {
        // Existing nominal/safe command topics
        pnh_.param<std::string>(
            "nominal_command_topic",
            nominal_command_topic_,
            "/arm_controller/joint_ref_nominal"
        );

        pnh_.param<std::string>(
            "safe_command_topic",
            safe_command_topic_,
            "/arm_controller/joint_ref"
        );

        // Input ToF point clouds
        pnh_.param<std::string>(
            "cloud_topic_1",
            cloud_topic_1_,
            "/link4_sensor1/tof/cloud"
        );

        pnh_.param<std::string>(
            "cloud_topic_2",
            cloud_topic_2_,
            "/link4_sensor2/tof/cloud"
        );

        // Transformed and fused output topics
        pnh_.param<std::string>(
            "cloud_1_base_topic",
            cloud_1_base_topic_,
            "/arm_safety_filter/cloud_1_base"
        );

        pnh_.param<std::string>(
            "cloud_2_base_topic",
            cloud_2_base_topic_,
            "/arm_safety_filter/cloud_2_base"
        );

        pnh_.param<std::string>(
            "fused_cloud_topic",
            fused_cloud_topic_,
            "/arm_safety_filter/fused_cloud"
        );

        // Common frame for subsequent safety computation
        pnh_.param<std::string>(
            "target_frame",
            target_frame_,
            "base_link"
        );

        // Synchronization and TF parameters
        pnh_.param<int>(
            "cloud_sub_queue_size",
            cloud_sub_queue_size_,
            5
        );

        pnh_.param<int>(
            "cloud_sync_queue_size",
            cloud_sync_queue_size_,
            10
        );

        pnh_.param<double>(
            "cloud_sync_slop",
            cloud_sync_slop_,
            0.05
        );

        pnh_.param<double>(
            "tf_timeout",
            tf_timeout_,
            0.05
        );
    }

    void initializeCommandPassThrough()
    {
        nominal_command_sub_ = nh_.subscribe(
            nominal_command_topic_,
            10,
            &PointcloudQpSafetyFilterNode::nominalCommandCallback,
            this
        );

        safe_command_pub_ = nh_.advertise<arm_msgs::JointReference>(
            safe_command_topic_,
            10
        );
    }

    void initializePointCloudFusionBranch()
    {
        cloud_1_base_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            cloud_1_base_topic_,
            1
        );

        cloud_2_base_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            cloud_2_base_topic_,
            1
        );

        fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            fused_cloud_topic_,
            1
        );

        cloud_1_sub_.subscribe(
            nh_,
            cloud_topic_1_,
            cloud_sub_queue_size_
        );

        cloud_2_sub_.subscribe(
            nh_,
            cloud_topic_2_,
            cloud_sub_queue_size_
        );

        /*
        * Important:
        * Pass a temporary ApproximateTime policy object directly into the
        * Synchronizer constructor. In ROS Noetic, passing a named policy
        * variable here can be incorrectly matched to the three-input
        * Synchronizer constructor.
        */
        cloud_sync_ = std::make_shared<CloudSynchronizer>(
            CloudSyncPolicy(static_cast<uint32_t>(cloud_sync_queue_size_)),
            cloud_1_sub_,
            cloud_2_sub_
        );

        cloud_sync_->setMaxIntervalDuration(
            ros::Duration(cloud_sync_slop_)
        );

        cloud_sync_->registerCallback(
            boost::bind(
                &PointcloudQpSafetyFilterNode::synchronizedCloudCallback,
                this,
                boost::placeholders::_1,
                boost::placeholders::_2
            )
        );
    }

    void printStartupInformation() const
    {
        ROS_INFO("============================================================");
        ROS_INFO("[pointcloud_qp_safety_filter] Started.");
        ROS_INFO("[pointcloud_qp_safety_filter] Command mode: PASS-THROUGH.");
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] nominal command input: "
                        << nominal_command_topic_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] safe command output: "
                        << safe_command_topic_);

        ROS_INFO("------------------------------------------------------------");
        ROS_INFO("[pointcloud_qp_safety_filter] Point-cloud TF and fusion enabled.");
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] cloud 1 input: "
                        << cloud_topic_1_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] cloud 2 input: "
                        << cloud_topic_2_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] target frame: "
                        << target_frame_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] cloud 1 base output: "
                        << cloud_1_base_topic_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] cloud 2 base output: "
                        << cloud_2_base_topic_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] fused cloud output: "
                        << fused_cloud_topic_);
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] cloud sync slop: "
                        << cloud_sync_slop_ << " s");
        ROS_INFO_STREAM("[pointcloud_qp_safety_filter] TF timeout: "
                        << tf_timeout_ << " s");

        ROS_WARN("[pointcloud_qp_safety_filter] No QP safety filtering is active yet.");
        ROS_INFO("============================================================");
    }

    void nominalCommandCallback(const arm_msgs::JointReference::ConstPtr& msg)
    {
        /*
         * Current stage:
         * Keep nominal command execution unchanged.
         *
         * The fused point cloud is prepared independently and will be used
         * when the QP safety constraint is added in the next step.
         */
        safe_command_pub_.publish(*msg);
    }

    bool transformCloudToTargetFrame(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
        sensor_msgs::PointCloud2& cloud_out)
    {
        if (cloud_in->header.frame_id.empty())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Received point cloud with empty frame_id."
            );
            return false;
        }

        try
        {
            /*
             * The ToF sensors are mounted on link4, which moves with the arm.
             * Transform at the point-cloud timestamp instead of using the
             * latest transform.
             */
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    cloud_in->header.frame_id,
                    cloud_in->header.stamp,
                    ros::Duration(tf_timeout_)
                );

            tf2::doTransform(*cloud_in, cloud_out, transform);

            cloud_out.header.frame_id = target_frame_;
            cloud_out.header.stamp = cloud_in->header.stamp;

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] TF transform failed from '"
                    << cloud_in->header.frame_id
                    << "' to '"
                    << target_frame_
                    << "': "
                    << ex.what()
            );

            return false;
        }
    }

    void synchronizedCloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_1_msg,
        const sensor_msgs::PointCloud2::ConstPtr& cloud_2_msg)
    {
        sensor_msgs::PointCloud2 cloud_1_base_msg;
        sensor_msgs::PointCloud2 cloud_2_base_msg;

        const bool cloud_1_ok =
            transformCloudToTargetFrame(cloud_1_msg, cloud_1_base_msg);

        const bool cloud_2_ok =
            transformCloudToTargetFrame(cloud_2_msg, cloud_2_base_msg);

        if (!cloud_1_ok || !cloud_2_ok)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Skipping fused cloud because one TF transform failed."
            );
            return;
        }

        /*
         * Continue publishing the two separately transformed clouds.
         * These remain useful for RViz debugging.
         */
        cloud_1_base_pub_.publish(cloud_1_base_msg);
        cloud_2_base_pub_.publish(cloud_2_base_msg);

        /*
         * Convert to XYZ-only point clouds.
         *
         * RGB is not needed for the current safe-executor baseline.
         * Invalid depth pixels are removed here so that the fused cloud
         * can later be used directly for nearest-obstacle computation.
         */
        pcl::PointCloud<pcl::PointXYZ> cloud_1_raw;
        pcl::PointCloud<pcl::PointXYZ> cloud_2_raw;
        pcl::PointCloud<pcl::PointXYZ> cloud_1_valid;
        pcl::PointCloud<pcl::PointXYZ> cloud_2_valid;
        pcl::PointCloud<pcl::PointXYZ> fused_cloud;

        pcl::fromROSMsg(cloud_1_base_msg, cloud_1_raw);
        pcl::fromROSMsg(cloud_2_base_msg, cloud_2_raw);

        std::vector<int> valid_indices_1;
        std::vector<int> valid_indices_2;

        pcl::removeNaNFromPointCloud(
            cloud_1_raw,
            cloud_1_valid,
            valid_indices_1
        );

        pcl::removeNaNFromPointCloud(
            cloud_2_raw,
            cloud_2_valid,
            valid_indices_2
        );

        fused_cloud = cloud_1_valid;
        fused_cloud += cloud_2_valid;

        fused_cloud.header.frame_id = target_frame_;
        fused_cloud.width =
            static_cast<std::uint32_t>(fused_cloud.points.size());
        fused_cloud.height = 1;
        fused_cloud.is_dense = true;

        sensor_msgs::PointCloud2 fused_cloud_msg;
        pcl::toROSMsg(fused_cloud, fused_cloud_msg);

        fused_cloud_msg.header.frame_id = target_frame_;

        /*
         * The two messages were approximately synchronized.
         * Use the newer timestamp for the published fused observation.
         */
        fused_cloud_msg.header.stamp =
            (cloud_1_msg->header.stamp >= cloud_2_msg->header.stamp)
                ? cloud_1_msg->header.stamp
                : cloud_2_msg->header.stamp;

        fused_cloud_pub_.publish(fused_cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_qp_safety_filter_node");

    PointcloudQpSafetyFilterNode node;

    ros::spin();

    return 0;
}