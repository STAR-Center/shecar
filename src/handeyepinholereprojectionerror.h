#ifndef HANDEYEPINHOLEREPROJECTIONERROR_H
#define HANDEYEPINHOLEREPROJECTIONERROR_H
#include<theia/theia.h>
#include "type.h"
#include "handeyecalibration_utils.h"
#include "handeye_project_point_to_image.h"
#include "handeyetransformation.h"

using namespace theia;


struct HandEyePinholeReprojectionError
{
public:
    explicit HandEyePinholeReprojectionError(const Feature& feature,const Pose handpose)
        : feature_(feature),handpose_(handpose) {}

    template<typename T> bool operator()(const T* handeyetrans,
                                         const T* camera_intrinsics,
                                         const T* point_parameters,
                                         T* reprojection_error) const
    {
        // Do not evaluate invalid camera configurations.
        if (camera_intrinsics[Camera::FOCAL_LENGTH] < T(0.0) ||
                camera_intrinsics[Camera::ASPECT_RATIO] < T(0.0))
        {
            return false;
        }

        T cameraposeparameter[6];

        // it is remarkable that theia and visualSFM use the transpose of rotation, i.e.
        // a point whose coordinate is xw in world frame and xc in camera frame,
        // we have xc = R(xw-t), this is the format of the right side of the follow equation.
        // the left side, we still exploit the normal format.
        // Rx[Rh'(x-th)]+tx = R(x-t)
        // that is R=Rx*Rh', t = th-Rh*Rx'*tx

        // transform type of handpose_ to T.
        Eigen::Matrix<T, 4, 4> handpose_t;
        for(int i=0; i<16; i++)
            handpose_t.data()[i] = (T) handpose_.data()[i];
        // transform hand eye rotation part to rotation matrix format.
        Eigen::Matrix<T, 3, 3> handeyerotation;
        ceres::AngleAxisToRotationMatrix(
            handeyetrans+HandEyeTransformation::ROTATION,
            ceres::ColumnMajorAdapter3x3(handeyerotation.data()));
        //
        Eigen::Matrix<T,3,1> handeyetranslation = Eigen::Map<const Eigen::Matrix<T,3,1>>(handeyetrans+HandEyeTransformation::TRANSLATION);
        // solve camera orientation R=Rx*Rh'
        Eigen::Matrix<T, 3, 3> cameraorientation = handeyerotation*handpose_t.topLeftCorner(3,3).transpose();
        // solve camera position t = th-Rh*Rx'*tx = th-R'*tx
        Eigen::Matrix<T,3,1> cameraposition = handpose_t.topRightCorner(3,1) - cameraorientation.transpose()*handeyetranslation;
        // map camera orientation to camera pose parameter
        ceres::RotationMatrixToAngleAxis(
            (const T*)cameraorientation.data(),
            (T*)cameraposeparameter+Camera::ORIENTATION);
        // map camera position to camera pose parameter
        Eigen::Map<Eigen::Matrix<T,3,1>>(cameraposeparameter + Camera::POSITION) = cameraposition;

        T reprojection[2];
        ProjectPointToImage(cameraposeparameter,
                            camera_intrinsics,
                            point_parameters,
                            reprojection);
        reprojection_error[0] = reprojection[0] - T(feature_.x());
        reprojection_error[1] = reprojection[1] - T(feature_.y());
        return true;
    }

private:
    const Feature feature_;
    const Pose handpose_;
};

#endif // HANDEYEPINHOLEREPROJECTIONERROR_H
