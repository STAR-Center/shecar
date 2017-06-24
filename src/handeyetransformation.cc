
#include "handeyetransformation.h"

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ceres/rotation.h>


const double* HandEyeTransformation::HandEyeParameter() const
{
    return hand_eye_parameter_;
}
double* HandEyeTransformation::Mutable_HandEyeParameter()
{
    return hand_eye_parameter_;
}

void HandEyeTransformation::SetHandEyeTranslatation(const Eigen::Vector3d& translation)
{
    Eigen::Map<Eigen::Vector3d>(Mutable_HandEyeParameter() + TRANSLATION) = translation;
}

Eigen::Vector3d HandEyeTransformation::GetHandEyeTranslation() const
{
    return Eigen::Map<const Eigen::Vector3d>(HandEyeParameter() + TRANSLATION);
}

void HandEyeTransformation::SetHandEyeRotationFromRotationMatrix(const Eigen::Matrix3d& rotation)
{
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(rotation.data()),
        hand_eye_parameter_ );
}

void HandEyeTransformation::SetHandEyeRotationFromAngleAxis(const Eigen::Vector3d& angle_axis)
{
    Eigen::Map<Eigen::Vector3d>(Mutable_HandEyeParameter()+ROTATION) = angle_axis;
}

Eigen::Matrix3d HandEyeTransformation::GetHandEyeRotationAsRotationMatrix() const
{
    Eigen::Matrix3d rotation;
    ceres::AngleAxisToRotationMatrix(
        HandEyeParameter(),
        ceres::ColumnMajorAdapter3x3(rotation.data()));
    return rotation;
}
Eigen::Vector3d HandEyeTransformation::GetHandEyeRotationAsAngleAxis() const
{
    return Eigen::Map<const Eigen::Vector3d>(hand_eye_parameter_);
}

