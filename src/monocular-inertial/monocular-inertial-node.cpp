#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utility.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

MonocularInertialSlamNode::MonocularInertialSlamNode(
    const rclcpp::NodeOptions &options)
    : Node("monocular_inertial_node", options) {

  // Declare parameters
  this->declare_parameter("vocabulary_path", "");
  this->declare_parameter("config_path", "");
  this->declare_parameter("imu_topic", "/oceansim/robot/imu");
  this->declare_parameter("image_topic", "/oceansim/robot/uw_img");
  this->declare_parameter("use_compressed", true);
  this->declare_parameter("trial_name", "");
  this->declare_parameter<double>("TimeshiftCamImu", 0.0);
  this->get_parameter("TimeshiftCamImu", m_timeshiftCamImu);

  RCLCPP_INFO(this->get_logger(), "Using TimeshiftCamImu = %.9f sec",
              m_timeshiftCamImu);
  // Get parameters
  std::string vocabulary_path =
      this->get_parameter("vocabulary_path").as_string();
  std::string config_path = this->get_parameter("config_path").as_string();
  std::string imu_topic = this->get_parameter("imu_topic").as_string();
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
      // Fallback or exit? For now just log, but saving will fail later.
  }

  if (vocabulary_path.empty() || config_path.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Please provide vocabulary_path and config_path parameters.");
    rclcpp::shutdown();
    return;
  }

  // 2. Initialize Publishers (ORDER MATTERS: Init before use in
  // threads/callbacks)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  m_pointcloud_publisher =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/orb_slam3/map_points", qos);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/orb_slam3/camera_pose", qos);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/orb_slam3/camera_trajectory", qos);
  path_msg_.header.frame_id = "map";

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 3. Initialize SLAM System
  bool visualization = true;
  m_SLAM = std::make_unique<ORB_SLAM3::System>(vocabulary_path, config_path,
                                               ORB_SLAM3::System::IMU_MONOCULAR,
                                               visualization);

  // 4. Initialize Subscribers
  subImu_ = this->create_subscription<ImuMsg>(
      imu_topic, 1000,
      std::bind(&MonocularInertialSlamNode::GrabImu, this, _1));

  if (bUseCompressed_) {
    std::cout << "Subscribing to Compressed Image topic: " << image_topic
              << std::endl;
    subImgCompressed_ = this->create_subscription<CompressedImageMsg>(
        image_topic, qos,
        std::bind(&MonocularInertialSlamNode::GrabCompressedImage, this,
                  std::placeholders::_1));
  } else {
    std::cout << "Subscribing to Raw Image topic: " << image_topic << std::endl;
    subImgRaw_ = this->create_subscription<ImageMsg>(
        image_topic, 10,
        std::bind(&MonocularInertialSlamNode::GrabImage, this,
                  std::placeholders::_1));
  }

  // Ground Truth Subscriber
  subGroundTruth_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/oceansim/robot/pose", 100,
      std::bind(&MonocularInertialSlamNode::GrabGroundTruth, this, _1));

  std::cout << "slam changed" << std::endl;

  // 5. Initialize Threads
  mbExitPoseThread_ = false;
  mbExitMapPointsThread_ = false;

  posePublishThread_ = std::make_unique<std::thread>(
      &MonocularInertialSlamNode::PublishPoseLoop, this);

  mapPointsThread_ = std::make_unique<std::thread>(
      &MonocularInertialSlamNode::PublishMapPointsLoop, this);

  syncThread_ = std::make_unique<std::thread>(
      &MonocularInertialSlamNode::SyncWithImu, this);
}

MonocularInertialSlamNode::~MonocularInertialSlamNode() {
  RCLCPP_INFO(this->get_logger(), "Destructor called. Saving results...");
  // 1. Stop Sync Thread
  if (syncThread_ && syncThread_->joinable()) {
    syncThread_->join();
  }
  syncThread_.reset();

  // 2. Stop Pose Publish Thread cleanly
  {
    std::lock_guard<std::mutex> lock(poseMutex_);
    mbExitPoseThread_ = true;
  }
  poseCondVar_.notify_all(); // Wake up the thread if it's waiting

  if (posePublishThread_ && posePublishThread_->joinable()) {
    posePublishThread_->join();
  }
  posePublishThread_.reset();

  mbExitMapPointsThread_ = true;
  if (mapPointsThread_ && mapPointsThread_->joinable())
    mapPointsThread_->join();
  mapPointsThread_.reset();

  // 3. Stop SLAM
  if (m_SLAM) {
    m_SLAM->Shutdown();
    
    std::string point_cloud_path = results_dir_ + "/PointCloud.txt";
    std::string keyframe_path = results_dir_ + "/KeyFrameTrajectory.txt";
    std::string frame_path = results_dir_ + "/FrameTrajectory.txt";

    RCLCPP_INFO(this->get_logger(), "Saving results to %s", results_dir_.c_str());
    m_SLAM->SavePointCloudMap(point_cloud_path);
    m_SLAM->SaveKeyFrameTrajectoryTUM(keyframe_path);
    m_SLAM->SaveTrajectoryTUM(frame_path);

    // Save Ground Truth
    std::string gt_path = results_dir_ + "/GroundTruth.txt";
    std::ofstream f;
    f.open(gt_path.c_str());
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
  }
}

void MonocularInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg) {
  if (!std::isnan(msg->linear_acceleration.x) &&
      !std::isnan(msg->linear_acceleration.y) &&
      !std::isnan(msg->linear_acceleration.z) &&
      !std::isnan(msg->angular_velocity.x) &&
      !std::isnan(msg->angular_velocity.y) &&
      !std::isnan(msg->angular_velocity.z)) {
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
  }
}

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutex);
  if (imgRawBuf_.size() > 0) {
    imgRawBuf_.pop();
  //   RCLCPP_WARN(this->get_logger(),
  //               "Raw Image buffer overflow, dropping oldest frame");
  }
  imgRawBuf_.push(msg);
}

void MonocularInertialSlamNode::GrabCompressedImage(
    const CompressedImageMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutex);
  if (imgCompBuf_.size() > 0) {
    imgCompBuf_.pop();
  //   RCLCPP_WARN(this->get_logger(),
  //               "Compressed Image buffer overflow, dropping oldest frame");
  }
  imgCompBuf_.push(msg);
}

void MonocularInertialSlamNode::GrabGroundTruth(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Got Ground Truth: %f", Utility::StampToSec(msg->header.stamp));
    std::lock_guard<std::mutex> lock(gtMutex_);
    double timestamp = Utility::StampToSec(msg->header.stamp);
    gt_trajectory_.push_back(std::make_pair(timestamp, msg->pose));
}

cv::Mat MonocularInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // Force conversion to MONO8 for ORB_SLAM
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }

  return cv_ptr->image.clone();
}

cv::Mat MonocularInertialSlamNode::GetCompressedImage(
    const CompressedImageMsg::SharedPtr msg) {
  try {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
      return cv::Mat();
    }

    if (image.type() == 0)
      return image.clone();

    RCLCPP_WARN(this->get_logger(), "Image type is %d, expected grayscale",
                image.type());
    return image.clone();
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

void MonocularInertialSlamNode::SyncWithImu() {
  while (rclcpp::ok()) {
    // Lock both mutexes simultaneously to prevent race conditions during the
    // whole cycle
    std::unique_lock<std::mutex> img_lock(mBufMutex, std::defer_lock);
    std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);

    std::lock(img_lock, imu_lock);

    // Peek at the front based on the selected topic type
    bool hasImg = false;
    double tImage = 0.0;

    if (bUseCompressed_) {
      if (!imgCompBuf_.empty()) {
        hasImg = true;
        tImage = Utility::StampToSec(imgCompBuf_.front()->header.stamp);
      }
    } else {
      if (!imgRawBuf_.empty()) {
        hasImg = true;
        tImage = Utility::StampToSec(imgRawBuf_.front()->header.stamp);
      }
    }
    tImage += m_timeshiftCamImu;

    // Process only if we have both Image and IMU data
    if (hasImg && !imuBuf_.empty()) {
      // Decode the image immediately
      cv::Mat im;
      if (bUseCompressed_) {
        im = GetCompressedImage(imgCompBuf_.front());
      } else {
        im = GetImage(imgRawBuf_.front());
      }

      // Collect ALL available IMU measurements up to the image timestamp
      std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
      while (!imuBuf_.empty() &&
             Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage) {
        auto imuMsg = imuBuf_.front();
        double t = Utility::StampToSec(imuMsg->header.stamp);

        imuBuf_.pop();
        cv::Point3f acc(imuMsg->linear_acceleration.x,
                        imuMsg->linear_acceleration.y,
                        imuMsg->linear_acceleration.z);
        cv::Point3f gyr(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y,
                        imuMsg->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
      }

      // Pop the processed image from the correct buffer
      if (bUseCompressed_) {
        imgCompBuf_.pop();
      } else {
        imgRawBuf_.pop();
      }

      // Track (only if image valid and we have enough IMU data)
      if (!im.empty() && !vImuMeas.empty()) {
        if (bClahe_)
          clahe_->apply(im, im);

        Sophus::SE3f Tcw = m_SLAM->TrackMonocular(im, tImage, vImuMeas);
        PublishPose(Tcw, this->now());
      } else if (vImuMeas.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Not enough IMU measurements (%zu) for image at %.6f",
                    vImuMeas.size(), tImage);
      } else if (im.empty()) {
        RCLCPP_WARN(this->get_logger(), "Not enough image measurements at %.6f",
                    tImage);
      }
    }

    img_lock.unlock();
    imu_lock.unlock();

    // Sleep to prevent CPU spinning
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MonocularInertialSlamNode::PublishMapPointsLoop() {
  // Run as long as the node is OK and we haven't been asked to exit
  while (rclcpp::ok() && !mbExitMapPointsThread_) {

    // 1. Sleep for a fixed duration (e.g., 500ms = 2Hz update rate)
    // This is better than a timer because it's in a dedicated thread
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (mbExitMapPointsThread_)
      break;

    // 2. Safety Checks
    if (!m_SLAM)
      continue;
    ORB_SLAM3::Atlas *pAtlas = m_SLAM->GetAtlas();
    if (!pAtlas)
      continue;
    ORB_SLAM3::Map *pMap = pAtlas->GetCurrentMap();
    if (!pMap)
      continue;

    // 3. Get Map Points (Thread safe access from ORB-SLAM3)
    std::vector<ORB_SLAM3::MapPoint *> vpMPs = pMap->GetAllMapPoints();
    if (vpMPs.empty())
      continue;

    // 4. Create PointCloud2 Message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    // Resize buffer to max possible size (all points) to avoid re-allocating
    modifier.resize(vpMPs.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    int valid_points = 0;

    // 5. Fill Data
    for (size_t i = 0; i < vpMPs.size(); i++) {
      if (vpMPs[i]->isBad())
        continue;

      // Access position safely
      Eigen::Vector3f pos = vpMPs[i]->GetWorldPos();

      // Transform: ORB-SLAM3 (Optical Z-fwd) -> ROS (Map X-fwd)
      *iter_x = pos(2);
      *iter_y = -pos(0);
      *iter_z = -pos(1);

      ++iter_x;
      ++iter_y;
      ++iter_z;
      valid_points++;
    }

    // Resize to actual valid count
    modifier.resize(valid_points);
    cloud_msg.width = valid_points;

    // 6. Publish
    if (valid_points > 0 && m_pointcloud_publisher) {
      m_pointcloud_publisher->publish(cloud_msg);
    }
  }
}

void MonocularInertialSlamNode::PublishPose(Sophus::SE3f Tcw,
                                            rclcpp::Time timestamp) {
  {
    std::lock_guard<std::mutex> lock(poseMutex_);
    PoseData data;
    data.Tcw = Tcw;
    data.timestamp = timestamp;
    poseQueue_.push(data);
  }
  poseCondVar_.notify_one(); // Signal the consumer thread
}

void MonocularInertialSlamNode::PublishPoseLoop() {
  while (rclcpp::ok()) {
    PoseData currentPose;

    {
      std::unique_lock<std::mutex> lock(poseMutex_);
      // Wait here until data is added to the queue OR we need to exit
      poseCondVar_.wait(
          lock, [this] { return !poseQueue_.empty() || mbExitPoseThread_; });

      if (mbExitPoseThread_ && poseQueue_.empty()) {
        break;
      }

      currentPose = poseQueue_.front();
      poseQueue_.pop();
    }

    // --- Processing Logic (No Lock Held Here) ---

    // ORB-SLAM3 returns World-to-Camera (Tcw), so we invert it to get
    // Camera-in-World
    Sophus::SE3f Twc = currentPose.Tcw.inverse();

    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // ROS Pose message
    geometry_msgs::msg::Pose pose;
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = currentPose.timestamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pose;

    if (pose_pub_) {
      pose_pub_->publish(pose_msg);
    }

    // Broadcast TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = currentPose.timestamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "camera_link";

    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;

    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(tf_msg);
    }

    path_msg_.header.stamp = currentPose.timestamp;
    path_msg_.poses.push_back(pose_msg);

    // Publish the full path
    if (path_pub_) {
      path_pub_->publish(path_msg_);
    }
}
}
