#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // for cvtColor

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2") {
  m_SLAM = pSLAM;

  // Subscribe to camera images (image_raw topic)
  m_image_subscriber = this->create_subscription<ImageMsg>(
      "image_raw", 10, std::bind(&MonocularSlamNode::GrabImage, this, _1));
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "camera_pose", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  std::cout << "ORB-SLAM3 node initialized" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
  cv::Mat gray_image;

  try {
    // Convert to cv::Mat in BGR8
    cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // Convert to grayscale
    cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::cout << "one frame has been sent" << std::endl;

  // Send grayscale frame to ORB-SLAM3
  Sophus::SE3f Tcw = m_SLAM->TrackMonocular(
      gray_image, Utility::StampToSec(msg->header.stamp));
  int state = m_SLAM->GetTrackingState();

  if (state == ORB_SLAM3::Tracking::OK ||
      state == ORB_SLAM3::Tracking::RECENTLY_LOST) {
    Sophus::SE3f Twc = Tcw.inverse();

    // 4. Extract Translation and Rotation (Eigen types)
    Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf rotation = Twc.unit_quaternion();

    // --- PUBLISH POSE STAMPED ---
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp; // Use original image timestamp
    pose_msg.header.frame_id = world_frame_id_;

    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();

    pose_msg.pose.orientation.x = rotation.x();
    pose_msg.pose.orientation.y = rotation.y();
    pose_msg.pose.orientation.z = rotation.z();
    pose_msg.pose.orientation.w = rotation.w();

    pose_pub_->publish(pose_msg);

    // --- PUBLISH TF ---
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = world_frame_id_;
    tf_msg.child_frame_id = camera_frame_id_;

    tf_msg.transform.translation.x = translation.x();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = translation.z();

    tf_msg.transform.rotation.x = rotation.x();
    tf_msg.transform.rotation.y = rotation.y();
    tf_msg.transform.rotation.z = rotation.z();
    tf_msg.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }
}
