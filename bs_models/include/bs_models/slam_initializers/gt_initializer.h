#pragma once

#include <bs_models/slam_initializers/slam_initializer_base.h>

#include <sensor_msgs/Imu.h>

#include <bs_models/frame_initializers/frame_initializers.h>

#include <bs_parameters/models/gt_initializer_params.h>

namespace bs_models {

class GTInitializer : public SLAMInitializerBase {
public:
  SMART_PTR_DEFINITIONS(GTInitializer);

  GTInitializer() : SLAMInitializerBase() {}

  ~GTInitializer() override = default;

  /**
   * @brief Callback for imu processing, this callback has most of the
   * intializer implementation
   * @param[in] msg - The lidar message to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

protected:
  /**
   * @brief Initialize variables that should remain constant in the case of a
   * reset request (publishers, parameters, extrinsics etc)
   */
  void onInit() override;

  /**
   * @brief Subscribe to topics and initialize values that should be fresh after
   * a reset
   */
  void onStart() override;

  /**
   * @brief Shutdown subscribers and reset variables that require resetting
   */
  void onStop() override;

  // parameters
  bs_parameters::models::GTInitializerParams gt_initializer_params_;

  // subscribers
  ros::Subscriber imu_subscriber_;

  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  ros::Time current_pose_time_ = ros::Time(0);
  uint32_t max_poses_;
};
} // namespace bs_models
