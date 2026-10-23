#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

struct BoneCombination {
    bool IsFinger;
    std::vector<std::string> Bones;
};

struct Config {
    std::vector<std::string> segmentation_bones;
    int segmentation_min_segment_length;
    double segmentation_min_acceleration_threshold;
    double segmentation_smoothing_sigma;
    double segmentation_threshold_factor;

    std::vector<BoneCombination> matching_bone_combinations;

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

        for (const auto& item : j.at("matching_bone_combinations")) {
            BoneCombination combination;
            combination.IsFinger = item.at("IsFinger").get<bool>();
            combination.Bones = item.at("Bones").get<std::vector<std::string>>();
            config.matching_bone_combinations.push_back(combination);
        }

        return config;
    }
};