
#ifndef HAND_EYE_BUNDLE_ADJUSTMENT_H_
#define HAND_EYE_BUNDLE_ADJUSTMENT_H_

#include <ceres/ceres.h>
#include <unordered_set>

#include <theia/theia.h>
#include "type.h"
#include "handeyetransformation.h"
using namespace theia;

// Bundle adjust all views and tracks in the reconstruction.
BundleAdjustmentSummary BundleAdjusthandEye(
    const BundleAdjustmentOptions& options, Reconstruction* reconstruction,Poses* handposes,HandEyeTransformation* handeyetrans);

// Bundle adjust the specified views and all tracks observed by those views.
BundleAdjustmentSummary BundleAdjustPartialHandEye(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& views_to_optimize,
    const std::unordered_set<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction,
    Poses* handposes,HandEyeTransformation* handeyetrans);

BundleAdjustmentSummary BundleAdjustView(
    const BundleAdjustmentOptions& options,
    const ViewId view_id,
    Reconstruction* reconstruction,
    Poses *handposes, HandEyeTransformation *handeyetrans);

BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options,
    const TrackId my_track_id,
    Reconstruction* reconstruction,
    Poses *handposes, HandEyeTransformation *handeyetrans);
#endif  // HAND_EYE_BUNDLE_ADJUSTMENT_H_
