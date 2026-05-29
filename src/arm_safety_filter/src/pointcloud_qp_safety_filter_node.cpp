#include <ros/ros.h>

#include <arm_msgs/JointReference.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <hpp/fcl/data_types.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/octree.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <urdf/model.h>

#include <Eigen/Geometry>

#include <boost/bind/bind.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
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
    using Clock = std::chrono::steady_clock;

    PointcloudQpSafetyFilterNode()
        : nh_(),
          pnh_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          has_latest_distance_result_(false)
    {
        loadParameters();
        loadProtectedPrimitiveGeometryFromUrdf();

        initializeCommandPassThrough();
        initializePointCloudFusionBranch();
        initializeDistanceBranch();

        printStartupInformation();
    }

private:
    // ================================================================
    // Geometry and distance-result data structures
    // ================================================================
    struct PrimitiveCollision
    {
        std::string name;
        std::shared_ptr<hpp::fcl::CollisionGeometry> geometry;
        hpp::fcl::Transform3f link_T_collision;
    };

    struct ObstacleDistanceResult
    {
        bool valid = false;
        bool normal_valid = false;

        ros::Time stamp;

        double distance = std::numeric_limits<double>::infinity();
        std::string primitive_name;

        Eigen::Vector3d robot_point_base = Eigen::Vector3d::Zero();
        Eigen::Vector3d obstacle_point_base = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal_base = Eigen::Vector3d::Zero();

        // Closest point fixed in the protected-link local frame.
        // This is the point that the later Pinocchio Jacobian branch needs.
        Eigen::Vector3d robot_point_link = Eigen::Vector3d::Zero();

        std::size_t obstacle_point_count = 0;

        double tf_ms = 0.0;
        double cloud_ms = 0.0;
        double octree_ms = 0.0;
        double fcl_ms = 0.0;
        double marker_ms = 0.0;
        double total_ms = 0.0;
    };

    // ================================================================
    // ROS node state
    // ================================================================
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ================================================================
    // Command pass-through branch
    // ================================================================
    ros::Subscriber nominal_command_sub_;
    ros::Publisher safe_command_pub_;

    std::string nominal_command_topic_;
    std::string safe_command_topic_;

    // ================================================================
    // Two-ToF fusion branch
    //
    // raw cloud
    //   -> remove NaN
    //   -> per-sensor VoxelGrid downsample in sensor frame
    //   -> transform retained points into base_link
    //   -> fuse and publish for external robot_body_filter
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

    bool enable_voxel_downsample_;
    double voxel_leaf_size_;
    bool publish_individual_base_clouds_;

    // ================================================================
    // Primitive FCL distance branch
    //
    // /arm_safety_filter/fused_cloud_filtered
    //   -> current-frame occupied OcTree
    //   -> primitive link distance query
    //   -> cache latest result for the later QP branch
    // ================================================================
    ros::Subscriber filtered_cloud_sub_;
    ros::Publisher distance_marker_pub_;

    std::string filtered_cloud_topic_;
    std::string robot_description_param_;
    std::string protected_link_;
    std::string distance_marker_topic_;

    double octree_resolution_;
    double distance_tf_timeout_;
    double warning_distance_;
    double marker_point_scale_;
    double line_scale_;
    double distance_log_period_;
    double max_observation_age_;
    bool enable_distance_markers_;

    std::vector<PrimitiveCollision> protected_primitives_;

    // This cache is safe while the node uses ros::spin() (single-threaded).
    // If this node later changes to an AsyncSpinner or a separate QP thread,
    // protect this state with a mutex.
    ObstacleDistanceResult latest_distance_result_;
    bool has_latest_distance_result_;

    // ================================================================
    // General helpers
    // ================================================================
    static double elapsedMs(
        const Clock::time_point& start,
        const Clock::time_point& end)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            end - start
        ).count() / 1000.0;
    }

    void loadParameters()
    {
        // ---------------- Command branch ----------------
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

        // ---------------- Fusion branch ----------------
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

        pnh_.param<std::string>(
            "target_frame",
            target_frame_,
            "base_link"
        );

        pnh_.param<int>(
            "cloud_sub_queue_size",
            cloud_sub_queue_size_,
            1
        );

        pnh_.param<int>(
            "cloud_sync_queue_size",
            cloud_sync_queue_size_,
            2
        );

        pnh_.param<double>(
            "cloud_sync_slop",
            cloud_sync_slop_,
            0.03
        );

        pnh_.param<double>(
            "tf_timeout",
            tf_timeout_,
            0.05
        );

        pnh_.param<bool>(
            "enable_voxel_downsample",
            enable_voxel_downsample_,
            true
        );

        pnh_.param<double>(
            "voxel_leaf_size",
            voxel_leaf_size_,
            0.010
        );

        pnh_.param<bool>(
            "publish_individual_base_clouds",
            publish_individual_base_clouds_,
            false
        );

        // ---------------- Distance branch ----------------
        pnh_.param<std::string>(
            "filtered_cloud_topic",
            filtered_cloud_topic_,
            "/arm_safety_filter/fused_cloud_filtered"
        );

        pnh_.param<std::string>(
            "robot_description_param",
            robot_description_param_,
            "/robot_description_self_filter"
        );

        pnh_.param<std::string>(
            "protected_link",
            protected_link_,
            "link4"
        );

        pnh_.param<std::string>(
            "distance_marker_topic",
            distance_marker_topic_,
            "/arm_safety_filter/link4_primitive_distance_markers"
        );

        pnh_.param<double>(
            "octree_resolution",
            octree_resolution_,
            0.010
        );

        pnh_.param<double>(
            "distance_tf_timeout",
            distance_tf_timeout_,
            0.05
        );

        pnh_.param<double>(
            "warning_distance",
            warning_distance_,
            0.03
        );

        pnh_.param<double>(
            "marker_point_scale",
            marker_point_scale_,
            0.015
        );

        pnh_.param<double>(
            "line_scale",
            line_scale_,
            0.004
        );

        pnh_.param<double>(
            "distance_log_period",
            distance_log_period_,
            1.0
        );

        pnh_.param<double>(
            "max_observation_age",
            max_observation_age_,
            0.10
        );

        pnh_.param<bool>(
            "enable_distance_markers",
            enable_distance_markers_,
            true
        );

        if (cloud_sub_queue_size_ <= 0 ||
            cloud_sync_queue_size_ <= 0 ||
            voxel_leaf_size_ <= 0.0 ||
            octree_resolution_ <= 0.0 ||
            tf_timeout_ < 0.0 ||
            distance_tf_timeout_ < 0.0 ||
            warning_distance_ < 0.0 ||
            marker_point_scale_ <= 0.0 ||
            line_scale_ <= 0.0 ||
            distance_log_period_ <= 0.0 ||
            max_observation_age_ <= 0.0)
        {
            ROS_FATAL("[pointcloud_qp_safety_filter] Invalid numeric parameter.");
            throw std::runtime_error("Invalid safety-filter numeric parameter.");
        }
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
        ROS_INFO("[pointcloud_qp_safety_filter] Fusion branch:");
        ROS_INFO_STREAM("  cloud 1 input: " << cloud_topic_1_);
        ROS_INFO_STREAM("  cloud 2 input: " << cloud_topic_2_);
        ROS_INFO_STREAM("  fused cloud output: " << fused_cloud_topic_);
        ROS_INFO_STREAM("  target frame: " << target_frame_);
        ROS_INFO_STREAM("  voxel downsample: "
                        << (enable_voxel_downsample_ ? "enabled" : "disabled")
                        << ", leaf size: " << voxel_leaf_size_ << " m");
        ROS_INFO_STREAM("  publish individual base clouds: "
                        << (publish_individual_base_clouds_ ? "true" : "false"));

        ROS_INFO("------------------------------------------------------------");
        ROS_INFO("[pointcloud_qp_safety_filter] Distance branch:");
        ROS_INFO_STREAM("  filtered cloud input: " << filtered_cloud_topic_);
        ROS_INFO_STREAM("  robot description: " << robot_description_param_);
        ROS_INFO_STREAM("  protected link: " << protected_link_);
        ROS_INFO_STREAM("  loaded primitives: " << protected_primitives_.size());
        ROS_INFO_STREAM("  octree resolution: " << octree_resolution_ << " m");
        ROS_INFO_STREAM("  distance marker output: " << distance_marker_topic_);
        ROS_INFO_STREAM("  markers enabled: "
                        << (enable_distance_markers_ ? "true" : "false"));
        ROS_INFO_STREAM("  maximum future QP observation age: "
                        << max_observation_age_ << " s");

        ROS_WARN("[pointcloud_qp_safety_filter] Primitive FCL distance is active, but QP command correction is NOT active yet.");
        ROS_INFO("============================================================");
    }

    // ================================================================
    // Command branch
    // ================================================================
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

    void nominalCommandCallback(const arm_msgs::JointReference::ConstPtr& msg)
    {
        // Step B remains pass-through. The cached distance result is not
        // allowed to modify commands until the Jacobian/QP step is added.
        safe_command_pub_.publish(*msg);
    }

    // ================================================================
    // Fusion branch
    // ================================================================
    void initializePointCloudFusionBranch()
    {
        if (publish_individual_base_clouds_)
        {
            cloud_1_base_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                cloud_1_base_topic_,
                1
            );

            cloud_2_base_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                cloud_2_base_topic_,
                1
            );
        }

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

        cloud_sync_ = std::make_shared<CloudSynchronizer>(
            CloudSyncPolicy(static_cast<std::uint32_t>(cloud_sync_queue_size_)),
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

    bool lookupCloudTransformAsEigen(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
        Eigen::Affine3f& transform_eigen)
    {
        if (cloud_msg->header.frame_id.empty())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Received point cloud with empty frame_id."
            );
            return false;
        }

        try
        {
            const geometry_msgs::TransformStamped transform_msg =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    cloud_msg->header.frame_id,
                    cloud_msg->header.stamp,
                    ros::Duration(tf_timeout_)
                );

            const geometry_msgs::Vector3& translation =
                transform_msg.transform.translation;
            const geometry_msgs::Quaternion& rotation =
                transform_msg.transform.rotation;

            Eigen::Quaternionf quaternion(
                static_cast<float>(rotation.w),
                static_cast<float>(rotation.x),
                static_cast<float>(rotation.y),
                static_cast<float>(rotation.z)
            );
            quaternion.normalize();

            transform_eigen =
                Eigen::Translation3f(
                    static_cast<float>(translation.x),
                    static_cast<float>(translation.y),
                    static_cast<float>(translation.z)
                ) * quaternion;

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Fusion TF lookup failed from '"
                    << cloud_msg->header.frame_id
                    << "' to '" << target_frame_
                    << "': " << ex.what()
            );
            return false;
        }
    }

    void downsampleValidCloud(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) const
    {
        output_cloud->clear();

        if (!enable_voxel_downsample_ || input_cloud->empty())
        {
            *output_cloud = *input_cloud;
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);

        const float leaf = static_cast<float>(voxel_leaf_size_);
        voxel_filter.setLeafSize(leaf, leaf, leaf);
        voxel_filter.filter(*output_cloud);

        output_cloud->width =
            static_cast<std::uint32_t>(output_cloud->points.size());
        output_cloud->height = 1;
        output_cloud->is_dense = true;
    }

    bool transformCloudToTargetFrame(
        const sensor_msgs::PointCloud2::ConstPtr& source_msg,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
    {
        target_cloud->clear();

        if (source_cloud->empty())
        {
            target_cloud->header.frame_id = target_frame_;
            target_cloud->width = 0;
            target_cloud->height = 1;
            target_cloud->is_dense = true;
            return true;
        }

        Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();

        if (!lookupCloudTransformAsEigen(source_msg, transform_eigen))
        {
            return false;
        }

        pcl::transformPointCloud(
            *source_cloud,
            *target_cloud,
            transform_eigen
        );

        target_cloud->header.frame_id = target_frame_;
        target_cloud->width =
            static_cast<std::uint32_t>(target_cloud->points.size());
        target_cloud->height = 1;
        target_cloud->is_dense = true;

        return true;
    }

    void publishBaseCloudForDebug(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        const ros::Time& stamp,
        ros::Publisher& publisher) const
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = target_frame_;
        cloud_msg.header.stamp = stamp;
        cloud_msg.is_dense = false;
        publisher.publish(cloud_msg);
    }

    void synchronizedCloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_1_msg,
        const sensor_msgs::PointCloud2::ConstPtr& cloud_2_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_1_raw;
        pcl::PointCloud<pcl::PointXYZ> cloud_2_raw;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_valid_local(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_valid_local(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_downsampled_local(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_downsampled_local(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_base(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_base(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        pcl::fromROSMsg(*cloud_1_msg, cloud_1_raw);
        pcl::fromROSMsg(*cloud_2_msg, cloud_2_raw);

        std::vector<int> valid_indices_1;
        std::vector<int> valid_indices_2;

        pcl::removeNaNFromPointCloud(
            cloud_1_raw,
            *cloud_1_valid_local,
            valid_indices_1
        );

        pcl::removeNaNFromPointCloud(
            cloud_2_raw,
            *cloud_2_valid_local,
            valid_indices_2
        );

        downsampleValidCloud(
            cloud_1_valid_local,
            cloud_1_downsampled_local
        );

        downsampleValidCloud(
            cloud_2_valid_local,
            cloud_2_downsampled_local
        );

        const bool cloud_1_ok = transformCloudToTargetFrame(
            cloud_1_msg,
            cloud_1_downsampled_local,
            cloud_1_base
        );

        const bool cloud_2_ok = transformCloudToTargetFrame(
            cloud_2_msg,
            cloud_2_downsampled_local,
            cloud_2_base
        );

        if (!cloud_1_ok || !cloud_2_ok)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Skipping fused cloud because one TF lookup failed."
            );
            return;
        }

        if (publish_individual_base_clouds_)
        {
            publishBaseCloudForDebug(
                cloud_1_base,
                cloud_1_msg->header.stamp,
                cloud_1_base_pub_
            );

            publishBaseCloudForDebug(
                cloud_2_base,
                cloud_2_msg->header.stamp,
                cloud_2_base_pub_
            );
        }

        pcl::PointCloud<pcl::PointXYZ> fused_cloud;
        fused_cloud = *cloud_1_base;
        fused_cloud += *cloud_2_base;

        fused_cloud.header.frame_id = target_frame_;
        fused_cloud.width =
            static_cast<std::uint32_t>(fused_cloud.points.size());
        fused_cloud.height = 1;
        fused_cloud.is_dense = true;

        sensor_msgs::PointCloud2 fused_cloud_msg;
        pcl::toROSMsg(fused_cloud, fused_cloud_msg);

        fused_cloud_msg.header.frame_id = target_frame_;
        fused_cloud_msg.header.stamp =
            (cloud_1_msg->header.stamp >= cloud_2_msg->header.stamp)
                ? cloud_1_msg->header.stamp
                : cloud_2_msg->header.stamp;

        /*
         * The fused cloud comes from frame-based ToF observations.
         * robot_body_filter is configured with point_by_point=false.
         */
        fused_cloud_msg.is_dense = false;

        fused_cloud_pub_.publish(fused_cloud_msg);
    }

    // ================================================================
    // Primitive FCL distance branch
    // ================================================================
    void initializeDistanceBranch()
    {
        filtered_cloud_sub_ = nh_.subscribe(
            filtered_cloud_topic_,
            1,
            &PointcloudQpSafetyFilterNode::filteredCloudCallback,
            this
        );

        if (enable_distance_markers_)
        {
            distance_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                distance_marker_topic_,
                1
            );
        }
    }

    hpp::fcl::Transform3f urdfPoseToFclTransform(
        const urdf::Pose& pose) const
    {
        hpp::fcl::Quaternion3f quaternion(
            pose.rotation.w,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z
        );
        quaternion.normalize();

        hpp::fcl::Transform3f transform =
            hpp::fcl::Transform3f::Identity();

        transform.setQuatRotation(quaternion);
        transform.setTranslation(
            hpp::fcl::Vec3f(
                pose.position.x,
                pose.position.y,
                pose.position.z
            )
        );

        return transform;
    }

    std::shared_ptr<hpp::fcl::CollisionGeometry> buildFclGeometry(
        const urdf::GeometrySharedPtr& geometry,
        const std::string& collision_name) const
    {
        if (!geometry)
        {
            throw std::runtime_error(
                "Collision '" + collision_name + "' has null geometry."
            );
        }

        if (geometry->type == urdf::Geometry::BOX)
        {
            const urdf::Box* box =
                dynamic_cast<const urdf::Box*>(geometry.get());

            if (box == nullptr)
            {
                throw std::runtime_error(
                    "Failed to parse box collision: " + collision_name
                );
            }

            return std::make_shared<hpp::fcl::Box>(
                box->dim.x,
                box->dim.y,
                box->dim.z
            );
        }

        if (geometry->type == urdf::Geometry::CYLINDER)
        {
            const urdf::Cylinder* cylinder =
                dynamic_cast<const urdf::Cylinder*>(geometry.get());

            if (cylinder == nullptr)
            {
                throw std::runtime_error(
                    "Failed to parse cylinder collision: " + collision_name
                );
            }

            return std::make_shared<hpp::fcl::Cylinder>(
                cylinder->radius,
                cylinder->length
            );
        }

        if (geometry->type == urdf::Geometry::SPHERE)
        {
            const urdf::Sphere* sphere =
                dynamic_cast<const urdf::Sphere*>(geometry.get());

            if (sphere == nullptr)
            {
                throw std::runtime_error(
                    "Failed to parse sphere collision: " + collision_name
                );
            }

            return std::make_shared<hpp::fcl::Sphere>(sphere->radius);
        }

        throw std::runtime_error(
            "Unsupported collision geometry on '" + collision_name +
            "'. Use box, cylinder or sphere in arm_self_filter.urdf."
        );
    }

    void appendCollisionPrimitive(
        const urdf::CollisionSharedPtr& collision,
        const std::string& fallback_name)
    {
        if (!collision)
        {
            return;
        }

        PrimitiveCollision primitive;
        primitive.name =
            collision->name.empty() ? fallback_name : collision->name;
        primitive.geometry =
            buildFclGeometry(collision->geometry, primitive.name);
        primitive.link_T_collision =
            urdfPoseToFclTransform(collision->origin);

        primitive.geometry->computeLocalAABB();
        protected_primitives_.push_back(primitive);

        ROS_INFO_STREAM(
            "[pointcloud_qp_safety_filter] Loaded primitive: "
                << primitive.name
        );
    }

    void loadProtectedPrimitiveGeometryFromUrdf()
    {
        std::string robot_description_xml;

        if (!nh_.getParam(robot_description_param_, robot_description_xml))
        {
            throw std::runtime_error(
                "Could not read robot description parameter: " +
                robot_description_param_
            );
        }

        urdf::Model model;
        if (!model.initString(robot_description_xml))
        {
            throw std::runtime_error(
                "Failed to parse URDF from parameter: " +
                robot_description_param_
            );
        }

        urdf::LinkConstSharedPtr link = model.getLink(protected_link_);
        if (!link)
        {
            throw std::runtime_error(
                "Could not find protected link in self-filter URDF: " +
                protected_link_
            );
        }

        protected_primitives_.clear();

        if (!link->collision_array.empty())
        {
            for (std::size_t i = 0; i < link->collision_array.size(); ++i)
            {
                appendCollisionPrimitive(
                    link->collision_array[i],
                    protected_link_ + "_collision_" + std::to_string(i)
                );
            }
        }
        else
        {
            appendCollisionPrimitive(
                link->collision,
                protected_link_ + "_collision"
            );
        }

        if (protected_primitives_.empty())
        {
            throw std::runtime_error(
                "Protected link has no supported collision primitives: " +
                protected_link_
            );
        }
    }

    bool filteredCloudToTargetFrame(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
        sensor_msgs::PointCloud2& target_msg)
    {
        if (cloud_msg->header.frame_id.empty())
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Filtered cloud has empty frame_id."
            );
            return false;
        }

        if (cloud_msg->header.frame_id == target_frame_)
        {
            target_msg = *cloud_msg;
            return true;
        }

        try
        {
            const geometry_msgs::TransformStamped transform_msg =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    cloud_msg->header.frame_id,
                    cloud_msg->header.stamp,
                    ros::Duration(distance_tf_timeout_)
                );

            tf2::doTransform(
                *cloud_msg,
                target_msg,
                transform_msg
            );

            target_msg.header.frame_id = target_frame_;
            target_msg.header.stamp = cloud_msg->header.stamp;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Filtered-cloud TF failed from '"
                    << cloud_msg->header.frame_id << "' to '"
                    << target_frame_ << "': " << ex.what()
            );
            return false;
        }
    }

    bool getProtectedLinkTransformAtStamp(
        const ros::Time& stamp,
        hpp::fcl::Transform3f& base_T_link_fcl,
        Eigen::Isometry3d& base_T_link_eigen)
    {
        try
        {
            const geometry_msgs::TransformStamped transform_msg =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    protected_link_,
                    stamp,
                    ros::Duration(distance_tf_timeout_)
                );

            const geometry_msgs::Vector3& translation =
                transform_msg.transform.translation;
            const geometry_msgs::Quaternion& rotation =
                transform_msg.transform.rotation;

            hpp::fcl::Quaternion3f fcl_quaternion(
                rotation.w,
                rotation.x,
                rotation.y,
                rotation.z
            );
            fcl_quaternion.normalize();

            base_T_link_fcl = hpp::fcl::Transform3f::Identity();
            base_T_link_fcl.setQuatRotation(fcl_quaternion);
            base_T_link_fcl.setTranslation(
                hpp::fcl::Vec3f(
                    translation.x,
                    translation.y,
                    translation.z
                )
            );

            Eigen::Quaterniond eigen_quaternion(
                rotation.w,
                rotation.x,
                rotation.y,
                rotation.z
            );
            eigen_quaternion.normalize();

            base_T_link_eigen = Eigen::Isometry3d::Identity();
            base_T_link_eigen.linear() =
                eigen_quaternion.toRotationMatrix();
            base_T_link_eigen.translation() =
                Eigen::Vector3d(
                    translation.x,
                    translation.y,
                    translation.z
                );

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "[pointcloud_qp_safety_filter] Distance TF failed from '"
                    << protected_link_ << "' to '" << target_frame_
                    << "': " << ex.what()
            );
            return false;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr prepareFilteredObstacleCloud(
        const sensor_msgs::PointCloud2& cloud_msg) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
            new pcl::PointCloud<pcl::PointXYZ>
        );
        pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        pcl::fromROSMsg(cloud_msg, *raw_cloud);

        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(
            *raw_cloud,
            *valid_cloud,
            valid_indices
        );

        return valid_cloud;
    }

    std::shared_ptr<hpp::fcl::OcTree> buildEnvironmentOcTree(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) const
    {
        std::shared_ptr<octomap::OcTree> octomap_tree(
            new octomap::OcTree(octree_resolution_)
        );

        for (const pcl::PointXYZ& point : cloud->points)
        {
            octomap_tree->updateNode(
                octomap::point3d(point.x, point.y, point.z),
                true
            );
        }

        octomap_tree->updateInnerOccupancy();

        std::shared_ptr<const octomap::OcTree> const_octomap_tree =
            octomap_tree;

        std::shared_ptr<hpp::fcl::OcTree> fcl_octree(
            new hpp::fcl::OcTree(const_octomap_tree)
        );

        fcl_octree->computeLocalAABB();
        return fcl_octree;
    }

    bool computeLinkObstacleDistance(
        const sensor_msgs::PointCloud2::ConstPtr& filtered_cloud_msg,
        ObstacleDistanceResult& result)
    {
        result = ObstacleDistanceResult();
        result.stamp = filtered_cloud_msg->header.stamp;

        const Clock::time_point t_begin = Clock::now();

        sensor_msgs::PointCloud2 target_cloud_msg;
        if (!filteredCloudToTargetFrame(
                filtered_cloud_msg,
                target_cloud_msg))
        {
            return false;
        }

        hpp::fcl::Transform3f base_T_link_fcl;
        Eigen::Isometry3d base_T_link_eigen =
            Eigen::Isometry3d::Identity();

        if (!getProtectedLinkTransformAtStamp(
                target_cloud_msg.header.stamp,
                base_T_link_fcl,
                base_T_link_eigen))
        {
            return false;
        }

        const Clock::time_point t_after_tf = Clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud =
            prepareFilteredObstacleCloud(target_cloud_msg);

        result.obstacle_point_count = obstacle_cloud->size();

        const Clock::time_point t_after_cloud = Clock::now();

        if (obstacle_cloud->empty())
        {
            result.tf_ms = elapsedMs(t_begin, t_after_tf);
            result.cloud_ms = elapsedMs(t_after_tf, t_after_cloud);
            result.total_ms = elapsedMs(t_begin, Clock::now());
            return true;
        }

        std::shared_ptr<hpp::fcl::OcTree> environment_octree =
            buildEnvironmentOcTree(obstacle_cloud);

        const Clock::time_point t_after_octree = Clock::now();

        const hpp::fcl::Transform3f environment_transform =
            hpp::fcl::Transform3f::Identity();

        hpp::fcl::DistanceRequest distance_request(true);

        double best_distance =
            std::numeric_limits<double>::infinity();
        std::string best_primitive_name;
        hpp::fcl::DistanceResult best_result;

        for (const PrimitiveCollision& primitive : protected_primitives_)
        {
            const hpp::fcl::Transform3f base_T_primitive =
                base_T_link_fcl * primitive.link_T_collision;

            hpp::fcl::DistanceResult distance_result;

            hpp::fcl::distance(
                primitive.geometry.get(),
                base_T_primitive,
                environment_octree.get(),
                environment_transform,
                distance_request,
                distance_result
            );

            if (distance_result.min_distance < best_distance)
            {
                best_distance = distance_result.min_distance;
                best_primitive_name = primitive.name;
                best_result = distance_result;
            }
        }

        const Clock::time_point t_after_fcl = Clock::now();

        if (!std::isfinite(best_distance))
        {
            return false;
        }

        result.valid = true;
        result.distance = best_distance;
        result.primitive_name = best_primitive_name;

        result.robot_point_base =
            Eigen::Vector3d(
                best_result.nearest_points[0][0],
                best_result.nearest_points[0][1],
                best_result.nearest_points[0][2]
            );

        result.obstacle_point_base =
            Eigen::Vector3d(
                best_result.nearest_points[1][0],
                best_result.nearest_points[1][1],
                best_result.nearest_points[1][2]
            );

        const Eigen::Vector3d delta =
            result.robot_point_base - result.obstacle_point_base;

        if (delta.norm() > 1e-8)
        {
            result.normal_base = delta.normalized();
            result.normal_valid = true;
        }

        result.robot_point_link =
            base_T_link_eigen.inverse() * result.robot_point_base;

        result.tf_ms = elapsedMs(t_begin, t_after_tf);
        result.cloud_ms = elapsedMs(t_after_tf, t_after_cloud);
        result.octree_ms = elapsedMs(t_after_cloud, t_after_octree);
        result.fcl_ms = elapsedMs(t_after_octree, t_after_fcl);
        result.total_ms = elapsedMs(t_begin, t_after_fcl);

        return true;
    }

    bool getLatestFreshDistanceResult(
        const ros::Time& current_time,
        ObstacleDistanceResult& result) const
    {
        if (!has_latest_distance_result_ ||
            !latest_distance_result_.valid ||
            !latest_distance_result_.normal_valid)
        {
            return false;
        }

        const double age =
            (current_time - latest_distance_result_.stamp).toSec();

        if (age < 0.0 || age > max_observation_age_)
        {
            return false;
        }

        result = latest_distance_result_;
        return true;
    }

    void clearDistanceMarkers(const ros::Time& stamp)
    {
        if (!enable_distance_markers_)
        {
            return;
        }

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = target_frame_;
        clear_marker.header.stamp = stamp;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        distance_marker_pub_.publish(marker_array);
    }

    void publishDistanceMarkers(
        const ObstacleDistanceResult& result)
    {
        if (!enable_distance_markers_ || !result.valid)
        {
            return;
        }

        visualization_msgs::MarkerArray marker_array;

        geometry_msgs::Point robot_point;
        robot_point.x = result.robot_point_base.x();
        robot_point.y = result.robot_point_base.y();
        robot_point.z = result.robot_point_base.z();

        geometry_msgs::Point obstacle_point;
        obstacle_point.x = result.obstacle_point_base.x();
        obstacle_point.y = result.obstacle_point_base.y();
        obstacle_point.z = result.obstacle_point_base.z();

        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = target_frame_;
        robot_marker.header.stamp = result.stamp;
        robot_marker.ns = "link4_primitive_nearest_points";
        robot_marker.id = 0;
        robot_marker.type = visualization_msgs::Marker::SPHERE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.pose.position = robot_point;
        robot_marker.pose.orientation.w = 1.0;
        robot_marker.scale.x = marker_point_scale_;
        robot_marker.scale.y = marker_point_scale_;
        robot_marker.scale.z = marker_point_scale_;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 1.0;
        robot_marker.color.b = 0.0;
        robot_marker.color.a = 1.0;
        marker_array.markers.push_back(robot_marker);

        visualization_msgs::Marker obstacle_marker;
        obstacle_marker.header.frame_id = target_frame_;
        obstacle_marker.header.stamp = result.stamp;
        obstacle_marker.ns = "link4_primitive_nearest_points";
        obstacle_marker.id = 1;
        obstacle_marker.type = visualization_msgs::Marker::SPHERE;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position = obstacle_point;
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.scale.x = marker_point_scale_;
        obstacle_marker.scale.y = marker_point_scale_;
        obstacle_marker.scale.z = marker_point_scale_;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 0.0;
        obstacle_marker.color.b = 0.0;
        obstacle_marker.color.a = 1.0;
        marker_array.markers.push_back(obstacle_marker);

        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = target_frame_;
        line_marker.header.stamp = result.stamp;
        line_marker.ns = "link4_primitive_nearest_pair";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = line_scale_;
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        line_marker.points.push_back(robot_point);
        line_marker.points.push_back(obstacle_point);
        marker_array.markers.push_back(line_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = target_frame_;
        text_marker.header.stamp = result.stamp;
        text_marker.ns = "link4_primitive_distance_text";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position = robot_point;
        text_marker.pose.position.z += 0.03;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.018;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text =
            result.primitive_name + ": d=" +
            std::to_string(result.distance) + " m";
        marker_array.markers.push_back(text_marker);

        distance_marker_pub_.publish(marker_array);
    }

    void logDistanceResult(const ObstacleDistanceResult& result) const
    {
        if (!result.valid)
        {
            ROS_INFO_STREAM_THROTTLE(
                distance_log_period_,
                "[pointcloud_qp_safety_filter] No observed external obstacle points for "
                    << protected_link_ << "."
            );
            return;
        }

        if (result.distance < warning_distance_)
        {
            ROS_WARN_STREAM_THROTTLE(
                0.5,
                "[pointcloud_qp_safety_filter] Near observed obstacle: d="
                    << result.distance << " m"
                    << " primitive=" << result.primitive_name
                    << " points=" << result.obstacle_point_count
                    << " total=" << result.total_ms << " ms"
                    << " [tf=" << result.tf_ms
                    << ", cloud=" << result.cloud_ms
                    << ", octree=" << result.octree_ms
                    << ", fcl=" << result.fcl_ms << "]"
            );
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(
                distance_log_period_,
                "[pointcloud_qp_safety_filter] Distance d="
                    << result.distance << " m"
                    << " primitive=" << result.primitive_name
                    << " points=" << result.obstacle_point_count
                    << " total=" << result.total_ms << " ms"
                    << " [tf=" << result.tf_ms
                    << ", cloud=" << result.cloud_ms
                    << ", octree=" << result.octree_ms
                    << ", fcl=" << result.fcl_ms << "]"
            );
        }
    }

    void filteredCloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        ObstacleDistanceResult result;

        if (!computeLinkObstacleDistance(cloud_msg, result))
        {
            return;
        }

        latest_distance_result_ = result;
        has_latest_distance_result_ = true;

        if (!result.valid)
        {
            clearDistanceMarkers(result.stamp);
        }
        else
        {
            const Clock::time_point marker_begin = Clock::now();
            publishDistanceMarkers(result);
            const Clock::time_point marker_end = Clock::now();

            latest_distance_result_.marker_ms =
                elapsedMs(marker_begin, marker_end);
        }

        logDistanceResult(latest_distance_result_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_qp_safety_filter_node");

    try
    {
        PointcloudQpSafetyFilterNode node;
        ros::spin();
    }
    catch (const std::exception& ex)
    {
        ROS_FATAL_STREAM(
            "[pointcloud_qp_safety_filter] Fatal initialization error: "
                << ex.what()
        );
        return 1;
    }

    return 0;
}
