#include <vector>
#include <Eigen/Geometry>

#include "types.h"

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

Eigen::Vector3d get_finger_representation(const Eigen::Vector3d& finger_tip, const Eigen::Vector3d& finger_direction, const Eigen::Vector3d& palm_center, const Eigen::Vector3d& palm_normal) {
    const Eigen::Vector3d u = palm_normal;
    const Eigen::Vector3d n = finger_direction;
    const Eigen::Vector3d d = finger_tip - palm_center;

    const Eigen::Vector3d v = (d / d.norm()).cross(u);
    const Eigen::Vector3d w = u.cross(v);

    const double alpha = std::acos(v.dot(n));
    const double phi = std::acos(u.dot(d / d.norm()));
    const double theta = std::atan2(w.dot(n), u.dot(n));

    return Eigen::Vector3d(alpha, phi, theta);
}

HandSkeleton pose_to_hand_skeleton(const Pose& pose) {
    // NOTE: ASSUMPTION IS THAT THE POSE ORDER IS TAKEN AS THUMB, INDEX, MIDDLE, RING, PINKY AND DOES NOT CONTAIN TIPS
    HandSkeleton skeleton;

    // since it only contains relative rotations, set default positions for the metacarpal joints
    skeleton.thumb_metacarpal = Eigen::Vector3d(0.0, 0.0, 0.0);
    skeleton.index_metacarpal = Eigen::Vector3d(0.0, 1.0, 0.0);
    skeleton.middle_metacarpal = Eigen::Vector3d(0.0, 2.0, 0.0);
    skeleton.ring_metacarpal = Eigen::Vector3d(0.0, 3.0, 0.0);
    skeleton.pinky_metacarpal = Eigen::Vector3d(0.0, 4.0, 0.0);

    // take arbitrary lengths
    double finger_length = 1.0;

    // set the positions of the other joints based on the rotations in the pose
    skeleton.thumb_proximal = skeleton.thumb_metacarpal + pose.joints[0].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.thumb_distal = skeleton.thumb_proximal + pose.joints[1].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.thumb_tip = skeleton.thumb_distal + pose.joints[2].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);

    skeleton.index_proximal = skeleton.index_metacarpal + pose.joints[3].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.index_middle = skeleton.index_proximal + pose.joints[4].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.index_distal = skeleton.index_middle + pose.joints[5].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.index_tip = skeleton.index_distal + pose.joints[6].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);

    skeleton.middle_proximal = skeleton.middle_metacarpal + pose.joints[7].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.middle_middle = skeleton.middle_proximal + pose.joints[8].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.middle_distal = skeleton.middle_middle + pose.joints[9].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.middle_tip = skeleton.middle_distal + pose.joints[10].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);

    skeleton.ring_proximal = skeleton.ring_metacarpal + pose.joints[11].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.ring_middle = skeleton.ring_proximal + pose.joints[12].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.ring_distal = skeleton.ring_middle + pose.joints[13].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.ring_tip = skeleton.ring_distal + pose.joints[14].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);

    skeleton.pinky_proximal = skeleton.pinky_metacarpal + pose.joints[15].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.pinky_middle = skeleton.pinky_proximal + pose.joints[16].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.pinky_distal = skeleton.pinky_middle + pose.joints[17].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);
    skeleton.pinky_tip = skeleton.pinky_distal + pose.joints[18].toRotationMatrix() * Eigen::Vector3d(finger_length, 0.0, 0.0);

    return skeleton;
}

std::vector<double> Handshape::compute_handshape_representation(const HandSkeleton& skeleton) {
    std::vector<Eigen::Vector3d> metacarpal_joint_positions = {
        skeleton.thumb_metacarpal,
        skeleton.thumb_distal,
        skeleton.index_metacarpal,
        skeleton.index_distal,
        skeleton.middle_metacarpal,
        skeleton.middle_distal,
        skeleton.ring_metacarpal,
        skeleton.ring_distal,
        skeleton.pinky_metacarpal,
        skeleton.pinky_distal
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

std::vector<double> Handshape::compute_handshape_representation_from_pose(const Pose& pose) {
    HandSkeleton skeleton = pose_to_hand_skeleton(pose);
    return Handshape::compute_handshape_representation(skeleton);
}