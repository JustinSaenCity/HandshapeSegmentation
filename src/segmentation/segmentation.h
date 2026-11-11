#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <nlohmann/json.hpp>

#include "lib/types.h"

#include "lib/math/math.h"
#include "lib/anim/anim.h"

using json = nlohmann::json;

struct PreMatchInfo {
    bool do_pre = false;
    std::vector<JointLocationTrack> hand_location_tracks;
    json bone_mapping;
};

namespace segmentation
{
    class Segmentation
    {
    public:
        static std::vector<SegmentData> get_segments(
            const std::vector<JointTrack>& tracks, const double fps, const std::string& config_file,
            bool save_to_csv = false, bool take_middle_50_percent = true, PreMatchInfo pre_match_info = PreMatchInfo());

        static std::vector<std::tuple<int, int>> get_segments_as_tuples(
            const std::vector<JointTrack>& tracks, const double fps, const std::string& config_file,
            bool save_to_csv = false);
    };
}
