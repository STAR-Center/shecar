#include "handeyecalibrationbuilder.h"
#include "handeyecalibration_estimator.h"

HandEyeCalibrationBuilder::HandEyeCalibrationBuilder(const ReconstructionBuilderOptions &options)
    :ReconstructionBuilder(options)
{

}

void HandEyeCalibrationBuilder::SetHandPoses(Poses* poses)
{
    CHECK(reconstruction_->NumViews()==poses->size())
            <<"numer of views "<<reconstruction_->NumViews()<<" is unequal to that of hand poses";

    // the sort of hand poses is different to that of views
    // so we need to resort hand poses according to name and ID of views
    hand_poses_.resize(poses->size());
    for(auto view_id: reconstruction_->ViewIds())
    {
        std::string name = reconstruction_->View(view_id)->Name();
        hand_poses_[view_id] = poses->at(atoi(name.c_str()));
    }
}

bool HandEyeCalibrationBuilder::BuildHandEyeCalibration(HandEyeTransformation* handeyetrans)
{
    CHECK_GE(view_graph_->NumViews(), 2) << "At least 2 images must be provided "
                                         "in order to create a "
                                         "reconstruction.";

    // Build tracks if they were not explicitly specified.
    if (reconstruction_->NumTracks() == 0)
    {
        track_builder_->BuildTracks(reconstruction_.get());
    }

    // Remove uncalibrated views from the reconstruction and view graph.
    if (options_.only_calibrated_views)
    {
        LOG(INFO) << "Removing uncalibrated views.";
        RemoveUncalibratedViews();
    }


    LOG(INFO) << "Attempting to reconstruct " << reconstruction_->NumViews()
              << " images from " << view_graph_->NumEdges()
              << " two view matches.";

    std::unique_ptr<HandEyeCalibrationEstimator> handeyecalibrationestimator=
        std::unique_ptr<HandEyeCalibrationEstimator>(
            new HandEyeCalibrationEstimator(options_.reconstruction_estimator_options));

    const auto& summary = handeyecalibrationestimator->Estimate(
                              view_graph_.get(), reconstruction_.get(),&hand_poses_,handeyetrans);

    //  if (!summary.success) {
    //    return false;
    //  }

    LOG(INFO)
            << "\nReconstruction estimation statistics: "
            << "\n\tNum estimated views = " << summary.estimated_views.size()
            << "\n\tNum input views = " << reconstruction_->NumViews()
            << "\n\tNum estimated tracks = " << summary.estimated_tracks.size()
            << "\n\tNum input tracks = " << reconstruction_->NumTracks()
            << "\n\tPose estimation time = " << summary.pose_estimation_time
            << "\n\tTriangulation time = " << summary.triangulation_time
            << "\n\tBundle Adjustment time = " << summary.bundle_adjustment_time
            << "\n\tTotal time = " << summary.total_time
            << "\n\n" << summary.message;

    return true;
}
