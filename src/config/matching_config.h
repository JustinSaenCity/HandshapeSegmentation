#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "math/math.h"

using json = nlohmann::json;

enum class MatchingMethod {
    Euclidean,
    Cosine,
    Manhattan
};

NLOHMANN_JSON_SERIALIZE_ENUM(MatchingMethod, {
    {MatchingMethod::Euclidean, "Euclidean"},
    {MatchingMethod::Cosine,    "Cosine"},
    {MatchingMethod::Manhattan, "Manhattan"},
})

struct MatchingConfig {
    std::string handshape_dir;
    MatchingMethod matching_method;

    static MatchingConfig load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        json j;
        file >> j;

        MatchingConfig config;
        config.handshape_dir = j.at("handshape_dir").get<std::string>();
        config.matching_method = j.at("matching_method").get<MatchingMethod>();

        return config;
    }
};