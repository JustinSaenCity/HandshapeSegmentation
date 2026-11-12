#include <vector>
#include <Eigen/Geometry>

#include "handshape.h"

using namespace handshape;

Eigen::Vector3d get_palm_center(const std::vector<Eigen::Vector3d>& metacarpal_joint_positions) {
    if (metacarpal_joint_positions.empty()) {
        return Eigen::Vector3d(0, 0, 0);
    }

    Eigen::Vector3d sum(0, 0, 0);
    for (const Eigen::Vector3d& pos : metacarpal_joint_positions) {
        sum = sum + pos;
    }

    return sum / static_cast<double>(metacarpal_joint_positions.size());
}

Eigen::Vector3d get_palm_normal(const Eigen::Vector3d& ring_metacarpal_start, const Eigen::Vector3d& ring_metacarpal_end, const Eigen::Vector3d& index_metacarpal_start, const Eigen::Vector3d& index_metacarpal_end) {
    const Eigen::Vector3d ring_vector = ring_metacarpal_end - ring_metacarpal_start;
    const Eigen::Vector3d index_vector = index_metacarpal_end - index_metacarpal_start;

    return ring_vector.cross(index_vector);
}

Eigen::Vector3d safe_normalise(const Eigen::Vector3d& vec) {
    double norm = vec.norm();
    return (norm > 1e-8) ? (vec / norm) : Eigen::Vector3d(0, 0, 0);
}

Eigen::Vector3d get_finger_representation(const Eigen::Vector3d& finger_tip, const Eigen::Vector3d& finger_direction, const Eigen::Vector3d& palm_center, const Eigen::Vector3d& palm_normal) {
    const Eigen::Vector3d u = safe_normalise(palm_normal);
    const Eigen::Vector3d n = safe_normalise(finger_direction);
    const Eigen::Vector3d d = safe_normalise(finger_tip - palm_center);

    const Eigen::Vector3d v = safe_normalise((d / d.norm()).cross(u));
    const Eigen::Vector3d w = u.cross(v);

    const double alpha = std::acos(std::clamp(v.dot(n), -1.0, 1.0));
    const double phi = std::acos(std::clamp(u.dot(d / d.norm()), -1.0, 1.0));
    const double theta = std::atan2(w.dot(n), u.dot(n));  // NOTE: ATAN2
    // const double theta = std::atan(w.dot(n) / u.dot(n));

    return Eigen::Vector3d(alpha, phi, theta);
}

std::vector<double> Handshape::compute_handshape_representation(const HandSkeleton& skeleton) {
    std::vector<Eigen::Vector3d> metacarpal_joint_positions = {
        skeleton.thumb_metacarpal,
        skeleton.thumb_proximal,
        skeleton.index_metacarpal,
        skeleton.index_proximal,
        skeleton.middle_metacarpal,
        skeleton.middle_proximal,
        skeleton.ring_metacarpal,
        skeleton.ring_proximal,
        skeleton.pinky_metacarpal,
        skeleton.pinky_proximal
    };

    Eigen::Vector3d palm_center = get_palm_center(metacarpal_joint_positions);

    Eigen::Vector3d palm_normal = get_palm_normal(skeleton.ring_metacarpal, skeleton.ring_proximal, skeleton.index_metacarpal, skeleton.index_proximal);

    std::vector<double> handshape_representation;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> fingers = {
        {skeleton.thumb_tip, skeleton.thumb_tip - skeleton.thumb_distal},
        {skeleton.index_tip, skeleton.index_tip - skeleton.index_distal},
        {skeleton.middle_tip, skeleton.middle_tip - skeleton.middle_distal},
        {skeleton.ring_tip, skeleton.ring_tip - skeleton.ring_distal},
        {skeleton.pinky_tip, skeleton.pinky_tip - skeleton.pinky_distal}
    };

    for (const std::pair<Eigen::Vector3d, Eigen::Vector3d>& finger : fingers) {
        Eigen::Vector3d finger_representation = get_finger_representation(finger.first, finger.second, palm_center, palm_normal);
        handshape_representation.push_back(finger_representation.x());
        handshape_representation.push_back(finger_representation.y());
        handshape_representation.push_back(finger_representation.z());
    }

    return handshape_representation;
}
