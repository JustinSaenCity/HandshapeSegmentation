#pragma once

#include <vector>
#include <string>
#include <Eigen/Geometry>

#include "math/math.h"
#include "anim/anim.h"

struct Match { int handshape_id; double distance; };

struct Pose {
    std::vector<std::string> joint_names;
    std::vector<Eigen::Quaterniond> joints;
};

namespace poseshape
{
    class Poseshape
    {
    public:
        static std::vector<std::string> get_all_handshape_files();
        static Pose get_average_pose_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment, bool take_middle_50_percent = true);
        static double pose_distance(const Pose& shape1, const Pose& shape2);
        static std::vector<Match> match_poses(const std::vector<Pose>& handshapes, const Pose& input_shape);
    };
}