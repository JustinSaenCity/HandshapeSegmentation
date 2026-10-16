#pragma once

#include <vector>
#include <string>
#include <Eigen/Geometry>

#include "math/math.h"
#include "anim/anim.h"

struct Match { int handshape_id; double distance; };

struct Hand {
    std::vector<std::string> joint_names;
    std::vector<Eigen::Quaterniond> joints;
};

namespace handshape
{
    class Handshape
    {
    public:
        static std::vector<std::string> get_all_handshape_files();
        static Hand get_average_handshape_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment);
        static double pose_distance(const Hand& shape1, const Hand& shape2);
        static std::vector<Match> match_handshapes(const std::vector<Hand>& handshapes, const Hand& input_shape);
    };
}