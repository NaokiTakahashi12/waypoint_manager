
#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

namespace waypoint_server {
    struct Waypoint {
        using Vector = Eigen::Matrix<float, 6, 1>;
        using Quaternion = Eigen::Quaternionf;
        using Indexes = std::vector<std::string>;
        using Properties = std::unordered_map<std::string, std::string>;

        bool enable_position = true;
        bool enable_orientation = true;

        Vector goal;
        Quaternion quaternion;

        Indexes connections;
        Properties properties;
    };
}

