#pragma once

#include <Eigen/Geometry>

#include "types.h"
#include "config/matching_config.h"

struct HandSkeleton {  // Based on papers representation
    Eigen::Vector3d thumb_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d thumb_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d thumb_middle = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d thumb_distal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d thumb_tip = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    Eigen::Vector3d index_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d index_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d index_middle = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d index_distal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d index_tip = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    Eigen::Vector3d middle_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d middle_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d middle_middle = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d middle_distal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d middle_tip = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    Eigen::Vector3d ring_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d ring_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d ring_middle = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d ring_distal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d ring_tip = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    Eigen::Vector3d pinky_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d pinky_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d pinky_middle = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d pinky_distal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d pinky_tip = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

namespace handshape
{
    class Handshape
    {
    public:
        static std::vector<double> compute_handshape_representation(const HandSkeleton& skeleton);
        static std::vector<double> compute_handshape_representation_from_pose(const Pose& pose);

        static void create_distance_matrix_from_handshapefiles(const std::vector<std::string>& handshape_files, const std::vector<std::string>& input_files, const std::vector<std::string>& bones, const json& bone_mapping, const MatchingMethod method, const std::string& output_file);
    };
}