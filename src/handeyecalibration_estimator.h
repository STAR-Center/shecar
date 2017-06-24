#ifndef HANDEYECALIBRATION_ESTIMATOR_H_
#define HANDEYECALIBRATION_ESTIMATOR_H_

#include<theia/theia.h>
#include "handeyetransformation.h"

using namespace theia;

class HandEyeCalibrationEstimator:public GlobalReconstructionEstimator
{
public:
    HandEyeCalibrationEstimator(
        const ReconstructionEstimatorOptions& options);

    ReconstructionEstimatorSummary Estimate(ViewGraph* view_graph,
                                            Reconstruction* reconstruction,Poses* handposes,HandEyeTransformation* handeyetrans);
    void EstimateStructure(Poses* handposes,HandEyeTransformation* handeyetrans);
    bool HandEyeBundleAdjustment(Poses* handposes,HandEyeTransformation* handeyetrans);
    bool FilterTooFewerInlierViewPair();
    DISALLOW_COPY_AND_ASSIGN(HandEyeCalibrationEstimator);
};

#endif  // HANDEYECALIBRATION_ESTIMATOR_H_
