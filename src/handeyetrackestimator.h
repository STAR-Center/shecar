#ifndef HANDEYETRACKESTIMATOR_H
#define HANDEYETRACKESTIMATOR_H

#include <theia/theia.h>
#include "handeyetransformation.h"
#include "type.h"
using namespace theia;

class HandEyeTrackEstimator: public TrackEstimator
{
public:
    HandEyeTrackEstimator(const Options& options, Reconstruction* reconstruction,
                          Poses *handpose,HandEyeTransformation *handeyetrans)
        : TrackEstimator(options,reconstruction),
          handeyetrans_(handeyetrans),handpose_(handpose) {}
    // Attempts to estimate all unestimated tracks.
    TrackEstimator::Summary HandEyeEstimateAllTracks();
    TrackEstimator::Summary HandEyeEstimateTracks(
        const std::unordered_set<TrackId>& track_ids);
    void HandEyeEstimateTrackSet(const int start, const int end);
    bool HandEyeEstimateTrack(const TrackId track_id);
private:
    HandEyeTransformation *handeyetrans_;
    Poses* handpose_;
};

#endif // HANDEYETRACKESTIMATOR_H
