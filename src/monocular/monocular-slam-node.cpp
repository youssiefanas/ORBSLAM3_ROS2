#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utility.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(const rclcpp::NodeOptions &options)
    : Node("monocular_node", options) {

  // Declare parameters
  this->declare_parameter("vocabulary_path", "");
  this->declare_parameter("config_path", "");
  this->declare_parameter("image_topic", "/camera/image_raw");
  this->declare_parameter("use_compressed", false);
  this->declare_parameter("trial_name", "");

  // Get parameters
  std::string vocabulary_path = this->get_parameter("vocabulary_path").as_string();
  std::string config_path = this->get_parameter("config_path").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  std::string trial_name = this->get_parameter("trial_name").as_string();
  bUseCompressed_ = this->get_parameter("use_compressed").as_bool();

  // Create timestamp string
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S");
  std::string timestamp = ss.str();

  // Construct directory name
  results_dir_ = "session_" + timestamp;
  if (!trial_name.empty()) {
      results_dir_ += "_" + trial_name;
  }

  // Create directory
  try {
      if (std::filesystem::create_directory(results_dir_)) {
          RCLCPP_INFO(this->get_logger(), "Created results directory: %s", results_dir_.c_str());
      } else {
           RCLCPP_INFO(this->get_logger(), "Results directory might already exist: %s", results_dir_.c_str());
      }
  } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create results directory: %s", e.what());
  }

  if (vocabulary_path.empty() || config_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Please provide vocabulary_path and config_path parameters.");
    rclcpp::shutdown();
    return;
  }

  // Initialize Publishers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/orb_slam3/map_points", qos);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/orb_slam3/camera_pose", qos);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/orb_slam3/camera_trajectory", qos);
  path_msg_.header.frame_id = "map";

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize SLAM System
  bool visualization = true;
  m_SLAM = std::make_unique<ORB_SLAM3::System>(vocabulary_path, config_path,
                                               ORB_SLAM3::System::MONOCULAR,
                                               visualization);

  // Initialize Subscribers
  if (bUseCompressed_) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to Compressed Image topic: %s", image_topic.c_str());
    subImgCompressed_ = this->create_subscription<CompressedImageMsg>(
        image_topic, qos,
        std::bind(&MonocularSlamNode::GrabCompressedImage, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(this->get_logger(), "Subscribing to Raw Image topic: %s", image_topic.c_str());
    subImgRaw_ = this->create_subscription<ImageMsg>(
        image_topic, 10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
  }

  // Ground Truth Subscriber
  subGroundTruth_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/oceansim/robot/pose", 100,
      std::bind(&MonocularSlamNode::GrabGroundTruth, this, _1));

  RCLCPP_INFO(this->get_logger(), "ORB_SLAM3 Monocular node initialized");

  // Initialize Threads
  mbExitPoseThread_ = false;
  mbExitMapPointsThread_ = false;

  posePublishThread_ = std::make_unique<std::thread>(&MonocularSlamNode::PublishPoseLoop, this);
  mapPointsThread_ = std::make_unique<std::thread>(&MonocularSlamNode::PublishMapPointsLoop, this);
}

MonocularSlamNode::~MonocularSlamNode() {
  RCLCPP_INFO(this->get_logger(), "Destructor called. Saving results...");

  // Stop Pose Publish Thread
  {
    std::lock_guard<std::mutex> lock(poseMutex_);
    mbExitPoseThread_ = true;
  }
  poseCondVar_.notify_all();

  if (posePublishThread_ && posePublishThread_->joinable()) {
    posePublishThread_->join();
  }
  posePublishThread_.reset();

  // Stop Map Points Thread
  mbExitMapPointsThread_ = true;
  if (mapPointsThread_ && mapPointsThread_->joinable())
    mapPointsThread_->join();
  mapPointsThread_.reset();

  // Stop SLAM and Save
  if (m_SLAM) {
    // Save Ground Truth
    std::string gt_path = results_dir_ + "/GroundTruth.txt";
    std::ofstream f(gt_path.c_str());
    f << std::fixed;

    std::lock_guard<std::mutex> lock(gtMutex_);
    for(size_t i=0; i<gt_trajectory_.size(); i++)
    {
        const double &time = gt_trajectory_[i].first;
        const auto &pose = gt_trajectory_[i].second;

        f << std::setprecision(6) << time << " "
          << std::setprecision(9) << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
          << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << std::endl;
    }
    f.close();
    RCLCPP_INFO(this->get_logger(), "Saved Ground Truth to %s", gt_path.c_str());

    std::string point_cloud_path = results_dir_ + "/PointCloud.txt";
    std::string keyframe_path = results_dir_ + "/KeyFrameTrajectory.txt";
    std::string frame_path = results_dir_ + "/FrameTrajectory.txt";
    
    RCLCPP_INFO(this->get_logger(), "Saving results to %s", results_dir_.c_str());
    m_SLAM->SavePointCloudMap(point_cloud_path);
    m_SLAM->SaveKeyFrameTrajectoryTUM(keyframe_path);
    m_SLAM->SaveTrajectoryTUM(frame_path);
    m_SLAM->Shutdown();
  }
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {
    cv::Mat im = GetImage(msg);
    if(im.empty()) return;

    double tImage = Utility::StampToSec(msg->header.stamp);
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(im, tImage);
    PublishPose(Tcw, msg->header.stamp);
}

void MonocularSlamNode::GrabCompressedImage(const CompressedImageMsg::SharedPtr msg) {
    cv::Mat im = GetCompressedImage(msg);
    if(im.empty()) return;

    double tImage = Utility::StampToSec(msg->header.stamp);
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(im, tImage);
    PublishPose(Tcw, msg->header.stamp);
}

void MonocularSlamNode::GrabGroundTruth(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gtMutex_);
    double timestamp = Utility::StampToSec(msg->header.stamp);
    gt_trajectory_.push_back(std::make_pair(timestamp, msg->pose));
}

cv::Mat MonocularSlamNode::GetImage(const ImageMsg::SharedPtr msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
  return cv_ptr->image.clone();
}

cv::Mat MonocularSlamNode::GetCompressedImage(const CompressedImageMsg::SharedPtr msg) {
  try {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
      return cv::Mat();
    }
    return image.clone();
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

void MonocularSlamNode::PublishPose(Sophus::SE3f Tcw, rclcpp::Time timestamp) {
  {
    std::lock_guard<std::mutex> lock(poseMutex_);
    PoseData data;
    data.Tcw = Tcw;
    data.timestamp = timestamp;
    poseQueue_.push(data);
  }
  poseCondVar_.notify_one();
}

void MonocularSlamNode::PublishPoseLoop() {
  while (rclcpp::ok()) {
    PoseData currentPose;
    {
      std::unique_lock<std::mutex> lock(poseMutex_);
      poseCondVar_.wait(lock, [this] { return !poseQueue_.empty() || mbExitPoseThread_; });
      if (mbExitPoseThread_ && poseQueue_.empty()) break;
      currentPose = poseQueue_.front();
      poseQueue_.pop();
    }

    Sophus::SE3f Twc = currentPose.Tcw.inverse();
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::Pose pose;
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = currentPose.timestamp;
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.pose = pose;

    if (pose_pub_) pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = currentPose.timestamp;
    tf_msg.header.frame_id = world_frame_id_;
    tf_msg.child_frame_id = camera_frame_id_;
    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;

    if (tf_broadcaster_) tf_broadcaster_->sendTransform(tf_msg);

    path_msg_.header.stamp = currentPose.timestamp;
    path_msg_.poses.push_back(pose_msg);
    if (path_pub_) path_pub_->publish(path_msg_);
  }
}

void MonocularSlamNode::PublishMapPointsLoop() {
  while (rclcpp::ok() && !mbExitMapPointsThread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (mbExitMapPointsThread_) break;
    if (!m_SLAM) continue;

    ORB_SLAM3::Atlas *pAtlas = m_SLAM->GetAtlas();
    if (!pAtlas) continue;
    ORB_SLAM3::Map *pMap = pAtlas->GetCurrentMap();
    if (!pMap) continue;

    std::vector<ORB_SLAM3::MapPoint *> vpMPs = pMap->GetAllMapPoints();
    if (vpMPs.empty()) continue;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = world_frame_id_;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(vpMPs.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    int valid_points = 0;
    for (size_t i = 0; i < vpMPs.size(); i++) {
      if (vpMPs[i]->isBad()) continue;
      Eigen::Vector3f pos = vpMPs[i]->GetWorldPos();
      *iter_x = pos(2);
      *iter_y = -pos(0);
      *iter_z = -pos(1);
      ++iter_x; ++iter_y; ++iter_z;
      valid_points++;
    }
    modifier.resize(valid_points);
    cloud_msg.width = valid_points;

    if (valid_points > 0 && m_pointcloud_publisher) {
      m_pointcloud_publisher->publish(cloud_msg);
    }
  }
}
