#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

struct MatchingConfig {
    std::string handshape_dir;

    static MatchingConfig load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        json j;
        file >> j;

        MatchingConfig config;
        config.handshape_dir = j.at("handshape_dir").get<std::string>();

        return config;
    }
};