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

struct HandSkeleton {  // Based on papers representation
    Eigen::Vector3d thumb_metacarpal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d thumb_proximal = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
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

struct HandSkeletonInfo {
    std::string name;
    HandSkeleton handskeleton;
};

struct SegmentData {
    Segment segment;
    HandSkeleton hand_skeleton;
};

struct JointTrack {
    std::string name;
    std::vector<Eigen::Quaterniond> q;
};

struct JointLocationTrack {
    std::string name;
    std::vector<Eigen::Vector3d> locations;
};

struct HandFrame { Eigen::Vector3d origin; Eigen::Matrix3d R; bool valid; };
