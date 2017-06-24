
#ifndef HANDEYETRANSFORMATION_H_
#define HANDEYETRANSFORMATION_H_

#include <theia/theia.h>
#include "type.h"

class HandEyeTransformation
{
public:

    const double* HandEyeParameter() const;
    double* Mutable_HandEyeParameter();

    void SetHandEyeTranslatation(const Eigen::Vector3d& translation);
    Eigen::Vector3d GetHandEyeTranslation() const;

    void SetHandEyeRotationFromRotationMatrix(const Eigen::Matrix3d& rotation);
    void SetHandEyeRotationFromAngleAxis(const Eigen::Vector3d& angle_axis);
    Eigen::Matrix3d GetHandEyeRotationAsRotationMatrix() const;
    Eigen::Vector3d GetHandEyeRotationAsAngleAxis() const;

    enum ParametersIndex
    {
        ROTATION = 0,
        TRANSLATION = 3
    };

private:

    double hand_eye_parameter_[6];

};

#endif  //HANDEYETRANSFORMATION_H_
