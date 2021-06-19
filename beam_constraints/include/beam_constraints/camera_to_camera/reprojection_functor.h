#ifndef FUSE_MODELS_VISUAL_COST_FUNCTOR_H
#define FUSE_MODELS_VISUAL_COST_FUNCTOR_H

#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/util.h>

#include <beam_calibration/CameraModel.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>

template <class T>
using opt = beam::optional<T>;

namespace fuse_constraints {

struct CameraProjectionFunctor {
  CameraProjectionFunctor(
      const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
      Eigen::Vector2d pixel_detected)
      : camera_model_(camera_model), pixel_detected_(pixel_detected) {}

  bool operator()(const double* P, double* pixel) const {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    Eigen::Vector2d pixel_projected;
    bool in_image = false;
    bool in_domain = camera_model_->ProjectPoint(P_CAMERA_eig, pixel_projected, in_image);

    // get image dims in case projection fails
    uint16_t height =
        camera_model_->GetHeight() != 0 ? camera_model_->GetHeight() : 5000;
    uint16_t width =
        camera_model_->GetWidth() != 0 ? camera_model_->GetWidth() : 5000;

    if (in_image) {
      pixel[0] = pixel_projected[0];
      pixel[1] = pixel_projected[1];
    } else {
      // if the projection failed, set the projected point to
      // be the nearest edge point to the detected point
      int near_u =
          (width - pixel_detected_[0]) < pixel_detected_[0] ? width : 0;
      int dist_u = (width - pixel_detected_[0]) < pixel_detected_[0]
                       ? (width - pixel_detected_[0])
                       : pixel_detected_[0];
      int near_v =
          (height - pixel_detected_[1]) < pixel_detected_[1] ? height : 0;
      int dist_v = (height - pixel_detected_[1]) < pixel_detected_[1]
                       ? (height - pixel_detected_[1])
                       : pixel_detected_[1];
      if (dist_u <= dist_v) {
        pixel[0] = near_u;
        pixel[1] = pixel_detected_[1];
      } else {
        pixel[0] = pixel_detected_[0];
        pixel[1] = near_v;
      }
    }

    return true;
  }

  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  Eigen::Vector2d pixel_detected_;
};

class ReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, Recommended: 2x2 identity
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  ReprojectionFunctor(
      const fuse_core::Matrix2d& A, const Eigen::Vector2d& pixel_measurement,
      const std::shared_ptr<beam_calibration::CameraModel> cam_model)
      : A_(A), pixel_measurement_(pixel_measurement), cam_model_(cam_model) {
    compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
        new ceres::NumericDiffCostFunction<CameraProjectionFunctor,
                                           ceres::CENTRAL, 2, 3>(
            new CameraProjectionFunctor(cam_model_, pixel_measurement_))));
  }

  template <typename T>
  bool operator()(const T* const cam_orientation, const T* const cam_position,
                  const T* const landmark_position, T* residual) const {
    // rotate and translate point
    T P_CAMERA[3];
    ceres::QuaternionRotatePoint(cam_orientation, landmark_position, P_CAMERA);
    P_CAMERA[0] -= cam_position[0];
    P_CAMERA[1] -= cam_position[1];
    P_CAMERA[2] -= cam_position[2];

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    residual[0] = pixel_measurement_.cast<T>()[0] - pixel_projected[0];
    residual[1] = pixel_measurement_.cast<T>()[1] - pixel_projected[1];
    return true;
  }

private:
  fuse_core::Matrix2d A_;
  Eigen::Vector2d pixel_measurement_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;
};

} // namespace fuse_constraints

#endif // FUSE_MODELS_VISUAL_COST_FUNCTOR_H