#pragma once

#include <string>
#include <vector>
#include <tuple>
#include "math/math.h"

namespace handseg
{
    class Handseg
    {
    public:
        static void get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps);
        static std::vector<Segment> get_segments(char** argv, bool save_to_csv = false, bool plot = false);
        static std::vector<std::tuple<int, int>> get_segments_as_tuples(char** argv);
    };
}