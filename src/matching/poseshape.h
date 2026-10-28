#pragma once

#include <vector>
#include <string>
#include <Eigen/Geometry>

#include "types.h"

#include "segmentation.h"
#include "math/math.h"
#include "anim/anim.h"

namespace poseshape
{
    class Poseshape
    {
    public:
        static std::vector<std::string> get_all_handshape_files(std::string handshape_dir);
        static std::vector<PoseInfo> get_poses_from_files(const std::vector<std::string>& handshape_files, const std::vector<std::string>& bones);
        static Pose get_average_pose_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment, bool take_middle_50_percent = true);
        static double pose_distance(const Pose& shape1, const Pose& shape2);
        static std::vector<Match> match_poses(const std::vector<Pose>& poses, const Pose& input_shape);
        static std::vector<MatchInfo> get_matches(const SegmentData& segment_data, const std::string& segment_config_file, const std::string& matching_config_file);
    };
}