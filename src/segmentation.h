#pragma once

#include <string>
#include <vector>
#include <tuple>

#include "math/math.h"
#include "anim/anim.h"

struct SegmentData {
    Segment segment;
    Pose average_pose;
};

namespace segmentation
{
    class Segmentation
    {
    public:
        static void get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps);
        static std::vector<SegmentData> get_segments(const std::vector<JointTrack>& tracks, const double fps, const std::string& config_file, bool save_to_csv = false);
        static std::vector<std::tuple<int, int>> get_segments_as_tuples(const std::vector<JointTrack>& tracks, const double fps, const std::string& config_file, bool save_to_csv = false);
    };
}