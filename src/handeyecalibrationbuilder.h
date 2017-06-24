#ifndef HANDEYECALIBRATIONBUILDER_H
#define HANDEYECALIBRATIONBUILDER_H

#include<theia/theia.h>
#include"handeyetransformation.h"
#include"type.h"

using namespace theia;

class HandEyeCalibrationBuilder:public ReconstructionBuilder
{
public:
    explicit HandEyeCalibrationBuilder(const ReconstructionBuilderOptions& options);
    bool BuildHandEyeCalibration(HandEyeTransformation* handeyetrans);
    std::unique_ptr<Reconstruction> GetReconstruction()
    {
        return std::move(reconstruction_);
    }
    void SetHandPoses(Poses* poses);

private:
    // the pose of hand
    Poses hand_poses_;
};

#endif // HANDEYECALIBRATIONBUILDER_H
