#pragma once

#include <Eigen/Geometry>

#include "types.h"
#include "config/matching_config.h"

namespace handskel
{
    class Handskel
    {
    public:
        static void create_distance_matrix_from_handshapefiles(const std::vector<std::string>& handshape_files, const std::vector<std::string>& input_files, const std::vector<std::string>& bones, const json& bone_mapping, const MatchingMethod method, const std::string& output_file);
    };
}