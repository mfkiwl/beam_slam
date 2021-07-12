#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_utils/utils.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by searching through the current fuse graph,
 * or preintegrating to the time point
 */
class InternalFrameInitializer : public FrameInitializerBase {
 public:
  /**
   * @brief Constructor
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   * @param sensor_frame_id frame ID attached to the sensor. See
   * FrameInitializerBase for description
   */
  InternalFrameInitializer(int64_t poses_buffer_time,
                           const std::string& sensor_frame_id = "");

  /**
   * @brief Adds a pose to the tf buffercore
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the pose being added
   * @param sensor_frame sensor frame id. If empty, the sensor frame will be the
   * sensor frame used to instantiate this class. This option is useful when
   * multiple sensors are being used
   * @return true if pose was added successfully
   */
  bool AddPose(const Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& stamp,
               std::string sensor_frame_id = "");
};

}  // namespace frame_initializers
}  // namespace beam_models