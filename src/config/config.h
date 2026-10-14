#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

struct Config {
    std::vector<std::string> bone_filter;
    int min_segment_length;
    SegmentBy segment_by;
    double smooth_segment_values_sigma;
    double acceleration_threshold_factor;

    static Config load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        json j;
        file >> j;

        Config config;
        config.bone_filter = j.at("bone_filter").get<std::vector<std::string>>();
        config.min_segment_length = j.at("min_segment_length").get<int>();
        config.segment_by = static_cast<SegmentBy>(j.at("segment_by").get<int>());
        config.smooth_segment_values_sigma = j.at("smooth_segment_values_sigma").get<double>();
        config.acceleration_threshold_factor = j.at("acceleration_threshold_factor").get<double>();

        return config;
    }
};