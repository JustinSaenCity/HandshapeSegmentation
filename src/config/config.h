#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

struct Config {
    std::vector<std::string> segmentation_bones;
    int segmentation_min_segment_length;
    double segmentation_min_acceleration_threshold;
    double segmentation_smoothing_sigma;
    double segmentation_threshold_factor;

    std::vector<std::string> matching_pinky_bones;
    std::vector<std::string> matching_ring_bones;
    std::vector<std::string> matching_middle_bones;
    std::vector<std::string> matching_index_bones;
    std::vector<std::string> matching_thumb_bones;

    static Config load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        json j;
        file >> j;

        Config config;
        config.segmentation_bones = j.at("segmentation_bones").get<std::vector<std::string>>();
        config.segmentation_min_segment_length = j.at("segmentation_min_segment_length").get<int>();
        config.segmentation_min_acceleration_threshold = j.at("segmentation_min_acceleration_threshold").get<double>();
        config.segmentation_smoothing_sigma = j.at("segmentation_smoothing_sigma").get<double>();
        config.segmentation_threshold_factor = j.at("segmentation_threshold_factor").get<double>();

        config.matching_pinky_bones = j.at("matching_pinky_bones").get<std::vector<std::string>>();
        config.matching_ring_bones = j.at("matching_ring_bones").get<std::vector<std::string>>();
        config.matching_middle_bones = j.at("matching_middle_bones").get<std::vector<std::string>>();
        config.matching_index_bones = j.at("matching_index_bones").get<std::vector<std::string>>();
        config.matching_thumb_bones = j.at("matching_thumb_bones").get<std::vector<std::string>>();

        return config;
    }
};