#pragma once

#include <Eigen/Geometry>

#include "lib/types.h"
#include "lib/config/matching_config.h"

namespace handskel
{
    class Handskel
    {
    public:
        static HandSkeleton get_average_handskeleton_from_segment(
            const std::vector<JointLocationTrack>& tracks, const Segment& segment,
            const json& bone_mapping, bool take_middle_50_percent = true);

        static std::vector<Match> match_handskeletons(
            const std::vector<HandSkeleton>& handskeletons, const HandSkeleton& input_handskeleton,
            const MatchingMethod method);

        static HandSkeletonInfo get_handskeletoninfo_from_file(
            const std::string& handshape_file, const std::vector<std::string>& bones, const json& bone_mapping,
            const std::string& string_addition = "", const Segment& segment = Segment{0, -1}, bool take_middle_50_percent = false);

        static void create_distance_matrix_from_handshapefiles(
            const std::vector<std::string>& handshape_files, const std::vector<std::string>& input_files,
            const std::vector<std::string>& bones, const json& bone_mapping, const MatchingMethod method,
            const std::string& output_file);
    };
}