#ifndef HANDEYE_PROJECT_POINT_TO_IMAGE_H
#define HANDEYE_PROJECT_POINT_TO_IMAGE_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <ceres/rotation.h>
#include<theia/theia.h>

using namespace theia;
// Projects a homogeneous 3D point to an image by assuming a camera model
// defined by the Camera class. This function is templated so that it can be
// used by Ceres for bundle adjument in addition to the standard reprojection
// with doubles.
//
// Returns the depth of the 3D point subject to the projective scale (i.e. the
// depth of the point assuming the image plane is at a depth of 1). The depth is
// useful, for instance, to determine if the point reprojects behind the image.
//
// NOTE: The unit test for this method is included in
// theia/sfm/camera/camera_test.cc
template <typename T>
T HandEyeProjectPointToImage(const T* extrinsic_parameters,
                             const T* intrinsic_parameters,
                             const T* point,
                             T* pixel)
{
    typedef Eigen::Matrix<T, 3, 1> Matrix3T;
    typedef Eigen::Map<const Matrix3T> ConstMap3T;


    // Remove the translation.
    Eigen::Matrix<T, 3, 1> adjusted_point =
        ConstMap3T(point) -
        point[3] * ConstMap3T(extrinsic_parameters + Camera::POSITION);

    // Rotate the point.
    T rotated_point[3];
    T rotm_para[3];
    rotm_para[0] = - *(extrinsic_parameters + Camera::ORIENTATION);
    rotm_para[1] = - *(extrinsic_parameters + Camera::ORIENTATION+1);
    rotm_para[2] = - *(extrinsic_parameters + Camera::ORIENTATION+2);

    ceres::AngleAxisRotatePoint(/*extrinsic_parameters + Camera::ORIENTATION*/rotm_para,
            adjusted_point.data(),
            rotated_point);

    // Get normalized pixel projection at image plane depth = 1.
    const T& depth = adjusted_point(2);
    const T normalized_pixel[2] = { adjusted_point(0) / depth,
                                    adjusted_point(1) / depth
                                  };

    // Apply radial distortion.
    T distorted_pixel[2];
    RadialDistortPoint(normalized_pixel[0],
                       normalized_pixel[1],
                       intrinsic_parameters[Camera::RADIAL_DISTORTION_1],
                       intrinsic_parameters[Camera::RADIAL_DISTORTION_2],
                       distorted_pixel,
                       distorted_pixel + 1);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& focal_length = intrinsic_parameters[Camera::FOCAL_LENGTH];
    const T& skew = intrinsic_parameters[Camera::SKEW];
    const T& aspect_ratio = intrinsic_parameters[Camera::ASPECT_RATIO];
    const T& principal_point_x = intrinsic_parameters[Camera::PRINCIPAL_POINT_X];
    const T& principal_point_y = intrinsic_parameters[Camera::PRINCIPAL_POINT_Y];

    pixel[0] = focal_length * distorted_pixel[0] + skew * distorted_pixel[1] +
               principal_point_x;
    pixel[1] = focal_length * aspect_ratio * distorted_pixel[1] +
               principal_point_y;

    return depth / point[3];
}

#endif // HANDEYE_PROJECT_POINT_TO_IMAGE_H
