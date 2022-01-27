#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/Image.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/vo_initializer_params.h>

namespace bs_models {

class VOInitializer : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VOInitializer);

  VOInitializer();

  ~VOInitializer() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

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

  /**
   * @brief publish results of initialization to a InitializedPathMsg
   */
  void PublishResults();

  // parameters
  bs_parameters::models::VOInitializerParams vo_initializer_params_;
  bs_parameters::models::CalibrationParams calibration_params_;

  // subscribers
  ros::Subscriber image_subscriber_;
  ros::Publisher results_publisher_;

  // get access to extrinsics singleton
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // bool for tracking if initialization has completed
  bool initialization_complete_{false};
  std::vector<Eigen::Matrix4d> trajectory_;
  std::deque<ros::Time> times_;
  std::deque<ros::Time> output_times_;

  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;

  // optimization stuff
  std::shared_ptr<vision::VisualMap> visual_map_;
  fuse_core::Graph::SharedPtr local_graph_;
};
} // namespace bs_models
