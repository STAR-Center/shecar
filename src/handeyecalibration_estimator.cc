
#include "handeyecalibration_estimator.h"
#include "handeyetrackestimator.h"
#include "hand_eye_bundle_adjustment.h"
#include "handeyecalibration_utils.h"
#include "axxb/axxbestimator.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <sstream>  // NOLINT
#include <fstream>

namespace
{

// All times are given in seconds.
struct GlobalReconstructionEstimatorTimings
{
    double initial_view_graph_filtering_time = 0.0;
    double camera_intrinsics_calibration_time = 0.0;
    double rotation_estimation_time = 0.0;
    double rotation_filtering_time = 0.0;
    double relative_translation_optimization_time = 0.0;
    double relative_translation_filtering_time = 0.0;
    double position_estimation_time = 0.0;
};

FilterViewPairsFromRelativeTranslationOptions
SetRelativeTranslationFilteringOptions(
    const ReconstructionEstimatorOptions& options)
{
    FilterViewPairsFromRelativeTranslationOptions fvpfrt_options;
    fvpfrt_options.num_threads = options.num_threads;
    fvpfrt_options.num_iterations = options.translation_filtering_num_iterations;
    fvpfrt_options.translation_projection_tolerance =
        options.translation_filtering_projection_tolerance;
    return fvpfrt_options;
}

void SetUnderconstrainedAsUnestimated(Reconstruction* reconstruction)
{
    int num_underconstrained_views = -1;
    int num_underconstrained_tracks = -1;
    while (num_underconstrained_views != 0 && num_underconstrained_tracks != 0)
    {
        num_underconstrained_views =
            SetUnderconstrainedViewsToUnestimated(reconstruction);
        num_underconstrained_tracks =
            SetUnderconstrainedTracksToUnestimated(reconstruction);
    }
}

}

HandEyeCalibrationEstimator::HandEyeCalibrationEstimator(
    const ReconstructionEstimatorOptions& options):GlobalReconstructionEstimator(options)
{

}

// The pipeline for estimating camera poses and structure is as follows:
//   1) Filter potentially bad pairwise geometries by enforcing a loop
//      constaint on rotations that form a triplet.
//   2) Initialize focal lengths.
//   3) Estimate the global rotation for each camera.
//   4) Remove any pairwise geometries where the relative rotation is not
//      consistent with the global rotation.
//   5) Optimize the relative translation given the known rotations.
//   6) Filter potentially bad relative translations.
//   7) Estimate positions.
//   8) Estimate structure.
//   9) Bundle adjustment.
//   10) Retriangulate, and bundle adjust.
//
// After each filtering step we remove any views which are no longer connected
// to the largest connected component in the view graph.
ReconstructionEstimatorSummary HandEyeCalibrationEstimator::Estimate(
    ViewGraph* view_graph, Reconstruction* reconstruction,Poses* handposes,HandEyeTransformation* handeyetrans)
{
    CHECK_NOTNULL(reconstruction);
    reconstruction_ = reconstruction;
    view_graph_ = view_graph;
    orientations_.clear();
    positions_.clear();

    ReconstructionEstimatorSummary summary;
    GlobalReconstructionEstimatorTimings global_estimator_timings;
    Timer total_timer;
    Timer timer;

    // Step 1. Filter the initial view graph and remove any bad two view
    // geometries.
    LOG(INFO) << "Filtering the intial view graph.";
    timer.Reset();
    if (!FilterTooFewerInlierViewPair())
    {
        LOG(INFO) << "Insufficient view pairs to perform estimation.";
        return summary;
    }
    global_estimator_timings.initial_view_graph_filtering_time =
        timer.ElapsedTimeInSeconds();

    // Step 2. Calibrate any uncalibrated cameras.
    LOG(INFO) << "Calibrating any uncalibrated cameras.";
    timer.Reset();
    CalibrateCameras();
    summary.camera_intrinsics_calibration_time = timer.ElapsedTimeInSeconds();


    //step 3. solve AX=XB
    auto edges = view_graph->GetAllEdges();
    Poses handmotions, cameramotions;

    Eigen::Matrix3d temp;
    for(auto edge: edges)
    {
        handmotions.emplace_back( handposes->at(edge.first.first).inverse()*handposes->at(edge.first.second));

        //ceres::AngleAxisToRotationMatrix(edge.second.rotation_2.data(),ceres::ColumnMajorAdapter3x3(temp.data()));
        Eigen::AngleAxisd angleaxis(edge.second.rotation_2.norm(),edge.second.rotation_2/edge.second.rotation_2.norm());
        // it is remarkable that theia and visualSFM use the transpose of rotation, i.e.
        // a point whose coordinate is xw in world frame and xc in camera frame,
        // we have xc = R(xw-t), hence we need transpose rotation part
        cameramotions.emplace_back( Rt2hom(angleaxis.toRotationMatrix().transpose(),edge.second.position_2) );
    }

    std::vector<MotionPair> motionpairs;
    for(int i=0; i<handmotions.size(); i++)
        motionpairs.emplace_back(cameramotions[i],handmotions[i]);

    AXXBEstimator axxb_estimator;
    RansacParameters params;
    params.error_thresh = 0.01;
    params.failure_probability = 0.001;
    params.max_iterations = 500;
    //  params.min_inlier_ratio = 0.6;
    params.min_iterations = 50;
    params.use_mle = true;
    Ransac<AXXBEstimator> ransac_estimator(params,axxb_estimator);
    // Initialize must always be called!
    ransac_estimator.Initialize();
    RansacSummary ransacsummary;
    Pose x;
    ransac_estimator.Estimate(motionpairs, &x, &ransacsummary);

    handeyetrans->SetHandEyeRotationFromRotationMatrix(x.topLeftCorner(3,3));
    handeyetrans->SetHandEyeTranslatation(x.topRightCorner(3,1));

    // Set the poses in the reconstruction object.
    SetCameraPosesFromHandPoses(*handposes,handeyetrans, reconstruction_);

    // Always triangulate once, then retriangulate and remove outliers depending
    // on the reconstruciton estimator options.
    double old_handeyetrans[6];
    std::memcpy(old_handeyetrans,handeyetrans->HandEyeParameter(),6*sizeof(double));

    for (int i = 0; i < options_.num_retriangulation_iterations + 1; i++)
    {
        // Step 4. Triangulate features.
        LOG(INFO) << "Triangulating all features.";
        timer.Reset();
        EstimateStructure(handposes,handeyetrans);
        summary.triangulation_time += timer.ElapsedTimeInSeconds();

        SetUnderconstrainedAsUnestimated(reconstruction_);

        // Step 5. Bundle Adjustment.
        LOG(INFO) << "Performing bundle adjustment.";
        timer.Reset();

        if (!HandEyeBundleAdjustment(handposes, handeyetrans))
        {
            summary.success = false;
            LOG(WARNING) << "Bundle adjustment failed!";
            return summary;
        }
        summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();

        // Set the poses in the reconstruction object.
        SetCameraPosesFromHandPoses(*handposes,handeyetrans, reconstruction_);

        int num_points_removed = RemoveOutlierFeatures(
                                     options_.max_reprojection_error_in_pixels,
                                     options_.min_triangulation_angle_degrees,
                                     reconstruction_);
        LOG(INFO) << num_points_removed << " outlier points were removed.";

        // if handeyetrans changes less than threshold, break the iteration.
        if( (Eigen::Map<Eigen::Matrix<double,3,1>>(old_handeyetrans+HandEyeTransformation::TRANSLATION)
                -Eigen::Map<Eigen::Matrix<double,3,1>>(handeyetrans->Mutable_HandEyeParameter()+HandEyeTransformation::TRANSLATION)).norm()<1e-6)
            break;
        else
            std::memcpy(old_handeyetrans,handeyetrans->HandEyeParameter(),6*sizeof(double));
    }

    // Set the output parameters.
    GetEstimatedViewsFromReconstruction(*reconstruction_,
                                        &summary.estimated_views);
    GetEstimatedTracksFromReconstruction(*reconstruction_,
                                         &summary.estimated_tracks);
    summary.success = true;
    summary.total_time = total_timer.ElapsedTimeInSeconds();

    // Output some timing statistics.
    std::ostringstream string_stream;
    string_stream
            << "Global Reconstruction Estimator timings:"
            << "\n\tInitial view graph filtering time = "
            << global_estimator_timings.initial_view_graph_filtering_time
            << "\n\tCamera intrinsic calibration time = "
            << summary.camera_intrinsics_calibration_time
            << "\n\tRotation estimation time = "
            << global_estimator_timings.rotation_estimation_time
            << "\n\tRotation filtering time = "
            << global_estimator_timings.rotation_filtering_time
            << "\n\tRelative translation optimization time = "
            << global_estimator_timings.relative_translation_optimization_time
            << "\n\tRelative translation filtering time = "
            << global_estimator_timings.relative_translation_filtering_time
            << "\n\tPosition estimation time = "
            << global_estimator_timings.position_estimation_time;
    summary.message = string_stream.str();

    return summary;
}

void HandEyeCalibrationEstimator::EstimateStructure(
    Poses* handposes,HandEyeTransformation* handeyetrans)
{
    // Estimate all tracks.
    TrackEstimator::Options triangulation_options;
    triangulation_options.max_acceptable_reprojection_error_pixels =
        options_.triangulation_max_reprojection_error_in_pixels;
    triangulation_options.min_triangulation_angle_degrees =
        options_.min_triangulation_angle_degrees;
    triangulation_options.bundle_adjustment = options_.bundle_adjust_tracks;
    triangulation_options.ba_options = SetBundleAdjustmentOptions(options_, 0);
    triangulation_options.ba_options.num_threads = 1;
    triangulation_options.ba_options.verbose = false;
    triangulation_options.num_threads = options_.num_threads;
    HandEyeTrackEstimator track_estimator(triangulation_options, reconstruction_,handposes,handeyetrans);
    const TrackEstimator::Summary summary = track_estimator.HandEyeEstimateAllTracks();
}

bool HandEyeCalibrationEstimator::HandEyeBundleAdjustment(Poses* handposes,HandEyeTransformation* handeyetrans)
{
    // Bundle adjustment.
    int size = positions_.size();
    bundle_adjustment_options_ =
        SetBundleAdjustmentOptions(options_, positions_.size());
    const auto& bundle_adjustment_summary =
        BundleAdjusthandEye(bundle_adjustment_options_, reconstruction_,handposes,handeyetrans);
    return bundle_adjustment_summary.success;
}

bool HandEyeCalibrationEstimator::FilterTooFewerInlierViewPair()
{
    // Remove any view pairs that do not have a sufficient number of inliers.
    std::unordered_set<ViewIdPair> view_pairs_to_remove;
    const auto& view_pairs = view_graph_->GetAllEdges();
    for (const auto& view_pair : view_pairs)
    {
        if (view_pair.second.num_verified_matches <
                options_.min_num_two_view_inliers)
        {
            view_pairs_to_remove.insert(view_pair.first);
        }
    }
    for (const ViewIdPair view_id_pair : view_pairs_to_remove)
    {
        view_graph_->RemoveEdge(view_id_pair.first, view_id_pair.second);
    }

    //  // Only reconstruct the largest connected component.
    //  RemoveDisconnectedViewPairs(view_graph_);
    return view_graph_->NumEdges() >= 1;
}
