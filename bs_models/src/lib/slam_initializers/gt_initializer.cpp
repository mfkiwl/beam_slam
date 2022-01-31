#include <bs_models/slam_initializers/gt_initializer.h>

#include <pluginlib/class_list_macros.h>

#include <beam_utils/math.h>
#include <bs_common/bs_msgs.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GTInitializer, fuse_core::SensorModel)

namespace bs_models {

void GTInitializer::onInit() {
  // load parameters from ros
  gt_initializer_params_.loadFromROS(private_node_handle_);

  // init frame initializer
  if (gt_initializer_params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            gt_initializer_params_.frame_initializer_info, 100, 30,
            gt_initializer_params_.frame_initializer_sensor_frame_id);
  } else if (gt_initializer_params_.frame_initializer_type == "POSEFILE") {
    frame_initializer_ =
        std::make_unique<frame_initializers::PoseFileFrameInitializer>(
            gt_initializer_params_.frame_initializer_info);
  } else if (gt_initializer_params_.frame_initializer_type == "TRANSFORM") {
    frame_initializer_ =
        std::make_unique<frame_initializers::TransformFrameInitializer>(
            gt_initializer_params_.frame_initializer_info, 100, 30,
            gt_initializer_params_.frame_initializer_sensor_frame_id);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY, POSEFILE, TRANSFORM";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  // compute the max # of poses to keep given they are added at ~10hz
  max_poses_ = gt_initializer_params_.trajectory_time_window.toSec() / 0.1;
}

void GTInitializer::onStart() {
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      gt_initializer_params_.imu_topic, 100, &GTInitializer::processIMU, this);
}

void GTInitializer::onStop() {
  // if a reset request is called then we set initialization to be incomplete
  // and we wipe memory
  initialization_complete_ = false;
  trajectory_.clear();
  times_.clear();
  current_pose_time_ = ros::Time(0);
  imu_subscriber_.shutdown();
}

void GTInitializer::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  if (initialization_complete_) {
    imu_subscriber_.shutdown();
    return;
  }
  Eigen::Matrix4d T_WORLD_SENSOR;
  bool success = frame_initializer_->GetEstimatedPose(
      T_WORLD_SENSOR, msg->header.stamp, extrinsics_.GetBaselinkFrameId());
  // push poses to trajectory at ~10hz
  if (success && (msg->header.stamp - current_pose_time_).toSec() >= 0.1) {
    current_pose_time_ = msg->header.stamp;
    trajectory_.push_back(T_WORLD_SENSOR);
    times_.push_back(msg->header.stamp);
    if (beam::PassedMotionThreshold(
            trajectory_[0], T_WORLD_SENSOR, 0.0,
            gt_initializer_params_.min_trajectory_length, true, true, false)) {
      ROS_INFO("GT Initializer trajectory long enough. Complete.");
      PublishResults();
      initialization_complete_ = true;
      stop();
    } else if (trajectory_.size() > max_poses_) {
      trajectory_.erase(trajectory_.begin());
      times_.erase(times_.begin());
    }
  }
}

} // namespace bs_models
