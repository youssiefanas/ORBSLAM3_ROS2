#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    void GrabImage(const ImageMsg::SharedPtr msg);
    void GrabCompressedImage(const CompressedImageMsg::SharedPtr msg);
    void GrabGroundTruth(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void PublishPoseLoop();
    void PublishMapPointsLoop();
    void PublishPose(Sophus::SE3f Tcw, rclcpp::Time timestamp);

    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetCompressedImage(const CompressedImageMsg::SharedPtr msg);

    std::unique_ptr<ORB_SLAM3::System> m_SLAM;

    // Parameters
    bool bUseCompressed_;
    std::string results_dir_;

    // Subscribers
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRaw_;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgCompressed_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGroundTruth_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Threads
    std::unique_ptr<std::thread> posePublishThread_;
    std::unique_ptr<std::thread> mapPointsThread_;

    bool mbExitPoseThread_;
    bool mbExitMapPointsThread_;

    // Data Buffers & Mutexes
    std::queue<ImageMsg::SharedPtr> imgRawBuf_;
    std::queue<CompressedImageMsg::SharedPtr> imgCompBuf_;
    std::mutex mBufMutex;

    // Pose Publishing
    struct PoseData {
        Sophus::SE3f Tcw;
        rclcpp::Time timestamp;
    };
    std::queue<PoseData> poseQueue_;
    std::mutex poseMutex_;
    std::condition_variable poseCondVar_;

    // Ground Truth
    std::vector<std::pair<double, geometry_msgs::msg::Pose>> gt_trajectory_;
    std::mutex gtMutex_;

    // Path message
    nav_msgs::msg::Path path_msg_;

    // Frame IDs
    std::string world_frame_id_ = "map";
    std::string camera_frame_id_ = "camera_link";
};

#endif
