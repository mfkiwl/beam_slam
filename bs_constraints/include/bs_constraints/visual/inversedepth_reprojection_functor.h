#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <bs_constraints/helpers.h>

#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace bs_constraints {

class InverseDepthReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance for an inverse depth reprojection
   *
   * @param[in] information_matrix Residual weighting matrix
   * @param[in] pixel_measurement Pixel measurement
   * @param[in] intrinsic_matrix Camera intrinsic matrix (K):
   * [fx, 0, cx]
   * [0, fy, cy]
   * [0,  0,  1]
   * @param[in] T_cam_baselink Camera extrinsic
   * @param[in] bearing Bearing vector of the inverse depth landmark [mx, my, 1]
   */
  InverseDepthReprojectionFunctor(const Eigen::Matrix2d& information_matrix,
                                  const Eigen::Vector2d& pixel_measurement,
                                  const Eigen::Matrix3d& intrinsic_matrix,
                                  const Eigen::Matrix4d& T_cam_baselink,
                                  const Eigen::Vector3d& bearing)
      : information_matrix_(information_matrix),
        pixel_measurement_(pixel_measurement),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink),
        bearing_(bearing) {}

  /**
   * @brief Construct a cost function instance for an inverse depth reprojection
   * An inverse depth reprojection cost function is defined by two poses: the
   * anchor pose of the inverse depth landmark, the measurement pose and the
   * inverse depth landmark itself. The anchor pose is used to determine the
   * euclidean location of the landmark in the world frame, which then gets
   * projected into the measurement frame and the normal reprojection is
   * computed.
   *
   * This transformation can be done by finding the relative transform between
   * anchor and measurement frame and constructing a projection matrix:
   * P = [K]*[R|t], where [R|t] is the 3x4 matrix representing the
   * transformation from the anchor frame to the measurement frame.
   */
  template <typename T>
  bool operator()(const T* const o_WORLD_BASELINKa,
                  const T* const p_WORLD_BASELINKa,
                  const T* const o_WORLD_BASELINKm,
                  const T* const p_WORLD_BASELINKm,
                  const T* const inverse_depth, T* residual) const {
    // get extrinsic
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    // get anchor pose as 4x4 matrix
    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINKa =
        bs_constraints::OrientationAndPositionToTransformationMatrix(
            o_WORLD_BASELINKa, p_WORLD_BASELINKa);

    // get measurement pose as 4x4 matrix
    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINKm =
        bs_constraints::OrientationAndPositionToTransformationMatrix(
            o_WORLD_BASELINKm, p_WORLD_BASELINKm);

    // get relative pose between anchor and measurement
    Eigen::Matrix<T, 4, 4> T_CAMERAm_CAMERAa =
        T_CAM_BASELINK * bs_constraints::InvertTransform(T_WORLD_BASELINKm) *
        T_WORLD_BASELINKa * bs_constraints::InvertTransform(T_CAM_BASELINK);

    // create projection matrix
    const Eigen::Matrix<T, 3, 4> projection_matrix =
        intrinsic_matrix_.cast<T>() * T_CAMERAm_CAMERAa.block(0, 0, 3, 4);

    // compute the inverse depth and bearing vector (mx, my, 1, 1/Z)
    Eigen::Matrix<T, 4, 1> bearing_and_inversedepth;
    bearing_and_inversedepth << bearing_.cast<T>(), inverse_depth;

    // project into measurement image
    Eigen::Matrix<T, 2, 1> reproj =
        (projection_matrix * bearing_and_inversedepth).hnormalized();

    Eigen::Matrix<T, 2, 1> E =
        information_matrix_.cast<T>() * (pixel_measurement_.cast<T>() - reproj);
    residual[0] = E[0];
    residual[1] = E[1];

    return true;
  }

private:
  Eigen::Matrix2d information_matrix_; //!< The residual weighting matrix
  Eigen::Vector2d pixel_measurement_;  //!< The measured pixel value
  Eigen::Vector3d bearing_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace bs_constraints
