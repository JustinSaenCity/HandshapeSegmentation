#pragma once

#include <vector>
#include <string>
#include <Eigen/Geometry>

struct Segment { 
    int start, end;
};

struct Match { int handshape_id; double distance; };

struct MatchInfo {
    std::string handshape_name;
    Match match;
};

struct Pose {
    std::vector<std::string> joint_names;
    std::vector<Eigen::Quaterniond> joints;
};

struct PoseInfo {
    std::string name;
    Pose handshape;
};

struct SegmentData {
    Segment segment;
    Pose average_pose;
};

struct JointTrack {
    std::string name;
    std::vector<Eigen::Quaterniond> q;
};