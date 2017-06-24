#ifndef HANDEYEREPROJECTIONERROR_H
#define HANDEYEREPROJECTIONERROR_H
#include <theia/theia.h>
#include "handeyepinholereprojectionerror.h"
#include "type.h"

using namespace theia;
struct HandEyeReprojectionError
{
public:
    explicit HandEyeReprojectionError(const Feature& feature, const Pose handpose) :
        feature_(feature),handpose_(handpose) {}

    static ceres::CostFunction* Create(const Feature& feature, const Pose handpose)
    {
        static const int kPointSize = 4;
        return new ceres::AutoDiffCostFunction<HandEyePinholeReprojectionError,
               2,
               Camera::kExtrinsicsSize,
               Camera::kIntrinsicsSize,
               kPointSize>(
                   new HandEyePinholeReprojectionError (feature,handpose));
    }

private:
    const Feature feature_;
    const Pose handpose_;
};

#endif // HANDEYEREPROJECTIONERROR_H
