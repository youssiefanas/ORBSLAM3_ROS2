#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>

#include "System.h"
#include <condition_variable> // Required for thread sleeping
#include <mutex>
#include <queue>
#include <rclcpp/publisher.hpp>
#include <thread>

using ImuMsg = sensor_msgs::msg::Imu;

class MonocularInertialSlamNode : public rclcpp::Node {
public:
  MonocularInertialSlamNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~MonocularInertialSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

  // New struct to pass data between threads
  struct PoseData {
    Sophus::SE3f Tcw;
    rclcpp::Time timestamp;
  };

  void GrabImu(const ImuMsg::SharedPtr msg);
  void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void
  GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void GrabGroundTruth(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  cv::Mat GetImage(const ImageMsg::SharedPtr msg);
  cv::Mat GetCompressedImage(const CompressedImageMsg::SharedPtr msg);

  void SyncWithImu();
  void PublishMapPointsLoop();
  std::unique_ptr<std::thread> mapPointsThread_;
  std::atomic<bool> mbExitMapPointsThread_{false};

  // Modified to push to queue
  void PublishPose(Sophus::SE3f Tcw, rclcpp::Time timestamp);

  // New consumer thread function
  void PublishPoseLoop();

  std::unique_ptr<ORB_SLAM3::System> m_SLAM;

  // Threads
  std::unique_ptr<std::thread> syncThread_;
  std::unique_ptr<std::thread> posePublishThread_; // New thread

  cv_bridge::CvImagePtr m_cvImPtr;

  rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
  rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgCompressed_;
  rclcpp::Subscription<ImageMsg>::SharedPtr subImgRaw_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGroundTruth_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      m_pointcloud_publisher;
  rclcpp::TimerBase::SharedPtr m_pointcloud_timer;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // IMU Buffer
  std::queue<ImuMsg::SharedPtr> imuBuf_;
  std::mutex bufMutex_;

  // Image Buffer
  std::queue<CompressedImageMsg::SharedPtr> imgCompBuf_;
  std::queue<ImageMsg::SharedPtr> imgRawBuf_;
  std::mutex mBufMutex;

  // --- New Variables for Pose Threading ---
  std::queue<PoseData> poseQueue_;
  std::mutex poseMutex_;
  std::condition_variable poseCondVar_;
  bool mbExitPoseThread_ = false;
  // ----------------------------------------

  bool bUseCompressed_;
  bool bClahe_ = false;
  cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
  double m_timeshiftCamImu = 0.0;
  std::string results_dir_;

  // Ground truth storage
  std::vector<std::pair<double, geometry_msgs::msg::Pose>> gt_trajectory_;
  std::mutex gtMutex_;
};

#endif