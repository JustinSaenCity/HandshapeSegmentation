#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

struct Config {
    std::vector<std::string> bones;
    int min_segment_length;
    double min_acceleration_threshold;
    double smoothing_sigma;
    double threshold_factor;

    static Config load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        json j;
        file >> j;

        Config config;
        config.bones = j.at("bones").get<std::vector<std::string>>();
        config.min_segment_length = j.at("min_segment_length").get<int>();
        config.min_acceleration_threshold = j.at("min_acceleration_threshold").get<double>();
        config.smoothing_sigma = j.at("smoothing_sigma").get<double>();
        config.threshold_factor = j.at("threshold_factor").get<double>();

        return config;
    }
};