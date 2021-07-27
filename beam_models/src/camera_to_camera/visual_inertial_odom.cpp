#include <beam_models/camera_to_camera/visual_inertial_odom.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>

#include <beam_common/utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&VisualInertialOdom::processImage,
                                          this, std::placeholders::_1)),
      throttled_imu_callback_(std::bind(&VisualInertialOdom::processIMU, this,
                                        std::placeholders::_1)) {}

void VisualInertialOdom::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  camera_params_.loadFromROS(private_node_handle_);
  global_params_.loadFromROS(private_node_handle_);
  /***********************************************************
   *       Initialize pose refiner object with params        *
   ***********************************************************/
  ceres::Solver::Options pose_refinement_options;
  pose_refinement_options.minimizer_progress_to_stdout = false;
  pose_refinement_options.logging_type = ceres::SILENT;
  pose_refinement_options.max_solver_time_in_seconds = 1e-3;
  pose_refinement_options.function_tolerance = 1e-4;
  pose_refinement_options.gradient_tolerance = 1e-6;
  pose_refinement_options.parameter_tolerance = 1e-4;
  pose_refinement_options.linear_solver_type = ceres::SPARSE_SCHUR;
  pose_refinement_options.preconditioner_type = ceres::SCHUR_JACOBI;
  pose_refiner_ =
      std::make_shared<beam_cv::PoseRefinement>(pose_refinement_options);
  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  cam_model_ =
      beam_calibration::CameraModel::Create(global_params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(cam_model_, camera_params_.source);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  beam_cv::DescriptorType descriptor_type =
      beam_cv::DescriptorTypeStringMap[camera_params_.descriptor];
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      beam_cv::Descriptor::Create(descriptor_type);
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::GFTTDetector>(
          camera_params_.num_features_to_track);
  tracker_ = std::make_shared<beam_cv::KLTracker>(detector, descriptor,
                                                  camera_params_.window_size);
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = node_handle_.subscribe(camera_params_.image_topic, 1000,
                                             &ThrottledImageCallback::callback,
                                             &throttled_image_callback_);
  imu_subscriber_ = node_handle_.subscribe(camera_params_.imu_topic, 10000,
                                           &ThrottledIMUCallback::callback,
                                           &throttled_imu_callback_);
  path_subscriber_ = private_node_handle_.subscribe(
      camera_params_.init_path_topic, 1, &VisualInertialOdom::processInitPath,
      this);
  init_odom_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>(
          camera_params_.frame_odometry_output_topic, 100);
  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  nlohmann::json J;
  std::ifstream file(global_params_.imu_intrinsics_path);
  file >> J;
  initializer_ =
      std::make_shared<beam_models::camera_to_camera::VIOInitializer>(
          cam_model_, tracker_, J["cov_gyro_noise"], J["cov_accel_noise"],
          J["cov_gyro_bias"], J["cov_accel_bias"], false,
          camera_params_.init_max_optimization_time_in_seconds,
          camera_params_.init_map_output_directory);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // push image onto buffer
  image_buffer_.push(*msg);
  // get current imu and image timestamps
  ros::Time imu_time = imu_buffer_.front().header.stamp;
  ros::Time img_time = image_buffer_.front().header.stamp;
  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (imu_time > img_time && !imu_buffer_.empty()) {
    tracker_->AddImage(ExtractImage(image_buffer_.front()), img_time);
    if (!initializer_->Initialized()) {
      if ((img_time - cur_kf_time_).toSec() >= 1.0) {
        keyframes_.push_back(img_time);
        cur_kf_time_ = img_time;
        if (initializer_->AddImage(img_time)) {
          ROS_INFO("Initialization Success: %f", cur_kf_time_.toSec());
          // get the preintegration object
          imu_preint_ = initializer_->GetPreintegrator();
          // copy init graph and send to fuse optimizer
          SendInitializationGraph(initializer_->GetGraph());
        } else {
          ROS_INFO("Initialization Failure: %f", cur_kf_time_.toSec());
        }
      }
    } else {
      beam::HighResolutionTimer timer;
      // dont process frame unless the initial graph has been optimized
      if (!init_graph_optimized_) return;
      // get most recent extrinsics, if failure then dont process frame
      if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
        ROS_ERROR("Unable to get camera to baselink transform.");
        return;
      }
      beam::HighResolutionTimer frame_timer;
      // localize frame
      std::vector<uint64_t> triangulated_ids;
      std::vector<uint64_t> untriangulated_ids;
      Eigen::Matrix4d T_WORLD_CAMERA;
      bool visual_localization_passed = LocalizeFrame(
          img_time, triangulated_ids, untriangulated_ids, T_WORLD_CAMERA);
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
      // publish pose to odom topic
      geometry_msgs::PoseStamped pose;
      beam_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, img_time,
                                                 pose);
      init_odom_publisher_.publish(pose);
      // process if keyframe
      if (IsKeyframe(img_time, triangulated_ids, untriangulated_ids,
                     T_WORLD_CAMERA)) {
        ROS_INFO("Estimated Keyframe Pose:");
        std::cout << T_WORLD_BASELINK << std::endl;
        // extend map
        ExtendMap(img_time, T_WORLD_CAMERA, triangulated_ids,
                  untriangulated_ids);
        // add imu constraint
        // SendInertialConstraint(img_time);
        cur_kf_time_ = img_time;
        keyframes_.push_back(img_time);
        added_since_kf_ = 0;
      } else {
        added_since_kf_++;
      }
      ROS_DEBUG("Total time to process frame: %.5f", frame_timer.elapsed());
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // push imu message onto buffer
  imu_buffer_.push(*msg);
  // get current image timestamp
  ros::Time img_time = image_buffer_.front().header.stamp;
  /**************************************************************************
   *          Add IMU messages to preintegrator or initializer              *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp <= img_time && !imu_buffer_.empty()) {
    if (!initializer_->Initialized()) {
      initializer_->AddIMU(imu_buffer_.front());
    } else {
      imu_preint_->AddToBuffer(imu_buffer_.front());
    }
    imu_buffer_.pop();
  }
}

void VisualInertialOdom::processInitPath(
    const InitializedPathMsg::ConstPtr& msg) {
  initializer_->SetPath(*msg);
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  visual_map_->UpdateGraph(graph);
  init_graph_optimized_ = true;
}

void VisualInertialOdom::onStop() {}

cv::Mat VisualInertialOdom::ExtractImage(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}

void VisualInertialOdom::SendInitializationGraph(
    const fuse_graphs::HashGraph& init_graph) {
  auto transaction = fuse_core::Transaction::make_shared();
  for (auto& var : init_graph.getVariables()) {
    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared();
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared();

    if (var.type() == landmark->type()) {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
      visual_map_->AddLandmark(landmark, transaction);
    } else if (var.type() == orientation->type()) {
      *orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(var);
      visual_map_->AddOrientation(orientation, transaction);
    } else if (var.type() == position->type()) {
      *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
      visual_map_->AddPosition(position, transaction);
    } else {
      transaction->addVariable(std::move(var.clone()));
    }
  }
  for (auto& constraint : init_graph.getConstraints()) {
    transaction->addConstraint(std::move(constraint.clone()));
  }
  sendTransaction(transaction);
}

bool VisualInertialOdom::LocalizeFrame(
    const ros::Time& img_time, std::vector<uint64_t>& triangulated_ids,
    std::vector<uint64_t>& untriangulated_ids,
    Eigen::Matrix4d& T_WORLD_CAMERA) {
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);
  // get 2d-3d correspondences
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm = visual_map_->GetLandmark(id);
    if (lm) {
      triangulated_ids.push_back(id);
      Eigen::Vector2i pixeli = tracker_->Get(img_time, id).cast<int>();
      pixels.push_back(pixeli);
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      points.push_back(point);
    } else {
      untriangulated_ids.push_back(id);
    }
  }
  // perform ransac pnp for initial estimate
  if (points.size() >= 15) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model_, pixels,
                                                        points);
    // refine pose using motion only BA
    Eigen::Matrix4d T_CAMERA_WORLD_ref = pose_refiner_->RefinePose(
        T_CAMERA_WORLD_est, cam_model_, pixels, points);
    T_WORLD_CAMERA = T_CAMERA_WORLD_ref.inverse();
    return true;
  } else {
    return false;
  }
}

bool VisualInertialOdom::IsKeyframe(
    const ros::Time& img_time, const std::vector<uint64_t>& triangulated_ids,
    const std::vector<uint64_t>& untriangulated_ids,
    const Eigen::Matrix4d& T_WORLD_CAMERA) {
  Eigen::Matrix4d T_WORLD_prevkf = visual_map_->GetPose(cur_kf_time_).value();
  bool is_keyframe = false;
  if ((img_time - cur_kf_time_).toSec() >=
          camera_params_.keyframe_min_time_in_seconds &&
      beam::PassedMotionThreshold(T_WORLD_prevkf, T_WORLD_CAMERA, 0.0, 0.1,
                                  true, true, false)) {
    ROS_INFO("New keyframe chosen at: %f", img_time.toSec());
    is_keyframe = true;
    // // combine all the id's seen in the image
    // std::vector<uint64_t> all_ids;
    // all_ids.insert(all_ids.end(), triangulated_ids.begin(),
    //                triangulated_ids.end());
    // all_ids.insert(all_ids.end(), untriangulated_ids.begin(),
    //                untriangulated_ids.end());
    // // compute the parallax between this frame and the last keyframe
    // double avg_parallax = ComputeAvgParallax(cur_kf_time_, img_time,
    // all_ids);
    // // test against parameters to see if this frame is a keyframe
    // if (beam::PassedMotionThreshold(T_WORLD_prevkf, T_WORLD_CAMERA, 0.0,
    // 0.05,
    //                                 true, true, false)) {
    //   if (triangulated_ids.size() < camera_params_.keyframe_tracks_drop ||
    //       avg_parallax > camera_params_.keyframe_parallax) {
    //     is_keyframe = true;
    //     ROS_INFO("New keyframe chosen at: %f", img_time.toSec());
    //     ROS_INFO("Avg parallax: %f", avg_parallax);
    //     ROS_INFO("Visible landmarks: %zu", triangulated_ids.size());
    //   } else if (added_since_kf_ == (camera_params_.window_size - 1)) {
    //     ROS_INFO("New keyframe chosen at: %f", img_time.toSec());
    //     ROS_INFO("Max # of frames reached.");
    //     is_keyframe = true;
    //   }
    // } else {
    //   is_keyframe = false;
    // }
  } else {
    is_keyframe = false;
  }
  return is_keyframe;
}

void VisualInertialOdom::ExtendMap(
    const ros::Time& img_time, const Eigen::Matrix4d& T_WORLD_CAMERA,
    const std::vector<uint64_t>& triangulated_ids,
    const std::vector<uint64_t>& untriangulated_ids) {
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(img_time);
  // add camera pose
  visual_map_->AddPose(T_WORLD_CAMERA, img_time, transaction);
  // add constraints to triangulated ids
  for (auto& id : triangulated_ids) {
    visual_map_->AddConstraint(img_time, id, tracker_->Get(img_time, id),
                               transaction);
  }
  size_t added_lms = 0;
  // triangulate untriangulated ids and add constraints
  for (auto& id : untriangulated_ids) {
    Eigen::Vector2d pixel_prv_kf, pixel_cur_kf;
    Eigen::Matrix4d T_cam_world_prv_kf, T_cam_world_cur_kf;
    try {
      // get measurements
      pixel_prv_kf = tracker_->Get(cur_kf_time_, id);
      pixel_cur_kf = tracker_->Get(img_time, id);
      T_cam_world_prv_kf = visual_map_->GetPose(cur_kf_time_).value().inverse();
      T_cam_world_cur_kf = visual_map_->GetPose(img_time).value().inverse();
      // triangulate point
      Eigen::Vector2i pixel_prv_kf_i = pixel_prv_kf.cast<int>();
      Eigen::Vector2i pixel_cur_kf_i = pixel_cur_kf.cast<int>();
      beam::opt<Eigen::Vector3d> point =
          beam_cv::Triangulation::TriangulatePoint(
              cam_model_, cam_model_, T_cam_world_prv_kf, T_cam_world_cur_kf,
              pixel_prv_kf_i, pixel_cur_kf_i);
      // add landmark and constraints to map
      if (point.has_value()) {
        added_lms++;
        visual_map_->AddLandmark(point.value(), id, transaction);
        visual_map_->AddConstraint(cur_kf_time_, id, pixel_prv_kf, transaction);
        visual_map_->AddConstraint(img_time, id, pixel_cur_kf, transaction);
      }
    } catch (const std::out_of_range& oor) {}
  }
  ROS_INFO("Added %zu new landmarks.", added_lms);
  sendTransaction(transaction);
}

void VisualInertialOdom::SendInertialConstraint(const ros::Time& img_time) {
  // get robot pose variables at timestamp
  fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
      visual_map_->GetOrientation(img_time);
  fuse_variables::Position3DStamped::SharedPtr img_position =
      visual_map_->GetPosition(img_time);
  // get inertial constraint transaction
  fuse_core::Transaction::SharedPtr transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(img_time, img_orientation,
                                                     img_position);
  // send transaction to opimizer
  sendTransaction(transaction);
}

double VisualInertialOdom::ComputeAvgParallax(
    const ros::Time& t1, const ros::Time& t2,
    const std::vector<uint64_t>& t2_landmarks) {
  std::vector<double> parallaxes;
  for (auto& id : t2_landmarks) {
    try {
      Eigen::Vector2d p1 = tracker_->Get(t1, id);
      Eigen::Vector2d p2 = tracker_->Get(t2, id);
      double dist = beam::distance(p1, p2);
      parallaxes.push_back(dist);
    } catch (const std::out_of_range& oor) {}
  }
  std::sort(parallaxes.begin(), parallaxes.end());
  return parallaxes[parallaxes.size() / 2];
}

}} // namespace beam_models::camera_to_camera
