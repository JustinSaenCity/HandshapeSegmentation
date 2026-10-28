#include <filesystem>
#include <vector>

#include "config/segmentation_config.h"
#include "config/matching_config.h"

#include "poseshape.h"

using namespace math;
using namespace segmentation;

using namespace poseshape;

std::vector<std::string> Poseshape::get_all_handshape_files(std::string handshape_dir) {
    std::vector<std::string> handshape_files;

    // Go through all folders, in the folder get the fbx file located in unreal/Vicon/
    for (const auto& entry : std::filesystem::directory_iterator(handshape_dir)) {
        if (entry.is_regular_file() &&
            (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            handshape_files.push_back(entry.path().string());
        }
    }
    return handshape_files;
}

std::vector<PoseInfo> Poseshape::get_poses_from_files(const std::vector<std::string>& handshape_files, const std::vector<std::string>& bones) {
    std::vector<PoseInfo> poses;
    for (const auto& handshape_file : handshape_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Segmentation::get_quaternion_tracks_from_fbx(handshape_file, bones, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Process the tracks
        for (JointTrack& quaternion_track : handshape_tracks) { 
            Math::align_quaternion_signs(quaternion_track.q);
            Math::smooth_quaternions(quaternion_track.q, 2.0);
        }

        PoseInfo average_handshape;
        average_handshape.name = handshape_file;
        average_handshape.handshape = Poseshape::get_average_pose_from_segment(handshape_tracks, {0, static_cast<int>(handshape_tracks[0].q.size()) - 1}, false);
        poses.push_back(average_handshape);
    }
    return poses;
}

std::vector<std::string> get_all_handshape_files_old() {
    std::vector<std::string> handshape_files;
    std::string handshape_dir = "fbx/handshapes";

    for (const auto& entry : std::filesystem::directory_iterator(handshape_dir)) {
        // Not case sensitive check for .fbx extension
        if (entry.is_regular_file() &&
           (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            handshape_files.push_back(entry.path().string());
        }
    }
    return handshape_files;
}

Pose Poseshape::get_average_pose_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment, bool take_middle_50_percent) {
    Pose average_pose;
    average_pose.joint_names.reserve(handshape_tracks.size());
    average_pose.joints.reserve(handshape_tracks.size());

    // Since the hand may still be moving slightly after the acceleration maximum, take the middle 50% frames (or all)
    int segment_length = segment.end - segment.start + 1;
    int start_frame = take_middle_50_percent ? segment.start + segment_length / 4 : segment.start;
    int end_frame = take_middle_50_percent ? segment.end - segment_length / 4 : segment.end;

    for (size_t i = 0; i < handshape_tracks.size(); ++i) {  // Go through each JointTrack (finger bone)
        Eigen::Quaterniond avg_quat(0, 0, 0, 0);
        for (int t = start_frame; t <= end_frame; ++t) {  // Go through each frame in the middle 50% of the segment
            avg_quat.coeffs() += handshape_tracks[i].q[t].coeffs();
        }
        avg_quat.coeffs() /= static_cast<double>(end_frame - start_frame + 1);
        avg_quat.normalize();
        average_pose.joints.push_back(avg_quat);
        average_pose.joint_names.push_back(handshape_tracks[i].name);
    }

    return average_pose;
}

std::vector<double> quaternions_to_vector(const std::vector<Eigen::Quaterniond>& quaternions) {
    std::vector<double> vec;
    vec.reserve(quaternions.size() * 4);
    for (const auto& q : quaternions) {
        vec.push_back(q.w());
        vec.push_back(q.x());
        vec.push_back(q.y());
        vec.push_back(q.z());
    }
    return vec;
}

std::vector<double> quaternions_to_euler_vector(const std::vector<Eigen::Quaterniond>& quaternions) {
    std::vector<double> vec;
    vec.reserve(quaternions.size() * 3);

    for (const auto& quat : quaternions) {
        // Get Euler angles from quaternion
        Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll (X), Pitch (Y), Yaw (Z)
        vec.push_back(euler_angles.x());
        vec.push_back(euler_angles.y());
        vec.push_back(euler_angles.z());
    }

    return vec;
}

std::vector<double> quaternions_to_relevant_euler_vector(const std::vector<Eigen::Quaterniond>& quaternions) {
    std::vector<double> vec;
    vec.reserve(quaternions.size() * 3);

    for (const auto& quat : quaternions) {
        // Get Euler angles from quaternion
        Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll (X), Pitch (Y), Yaw (Z)
        vec.push_back(euler_angles.x());
    }

    return vec;
}

double get_cosine_similarity(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    if (vec1.size() != vec2.size()) {
        throw std::invalid_argument("Vectors must be of the same size for cosine similarity.");
    }

    double dot_product = 0.0;
    double norm_vec1 = 0.0;
    double norm_vec2 = 0.0;

    for (size_t i = 0; i < vec1.size(); ++i) {
        dot_product += vec1[i] * vec2[i];
        norm_vec1 += vec1[i] * vec1[i];
        norm_vec2 += vec2[i] * vec2[i];
    }

    return dot_product / (std::sqrt(norm_vec1) * std::sqrt(norm_vec2));
}

double get_euclidean_distance(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    if (vec1.size() != vec2.size()) {
        throw std::invalid_argument("Vectors must be of the same size for Euclidean distance.");
    }

    double sum_squared_diff = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        double diff = vec1[i] - vec2[i];
        sum_squared_diff += diff * diff;
    }

    return std::sqrt(sum_squared_diff);
}

double get_manhattan_distance(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    if (vec1.size() != vec2.size()) {
        throw std::invalid_argument("Vectors must be of the same size for Manhattan distance.");
    }

    double sum_abs_diff = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        sum_abs_diff += std::abs(vec1[i] - vec2[i]);
    }

    return sum_abs_diff;
}

double Poseshape::pose_distance(const Pose& shape1, const Pose& shape2) {
    if (shape1.joints.size() != shape2.joints.size()) {
        printf("Error: Poses must have the same number of quaternions. Shape1 has %zu, Shape2 has %zu\n", shape1.joints.size(), shape2.joints.size());
        throw std::invalid_argument("Poses must have the same number of quaternions.");
    }

    // double angular_distance = 0.0;
    // for (size_t i = 0; i < shape1.joints.size(); ++i)
    //     angular_distance += Math::radial_angle_between_quats(shape1.joints[i], shape2.joints[i]);

    // angular_distance /= static_cast<double>(shape1.joints.size());

    // Cosine distance between shapes
    // std::vector<double> shape1_vec = quaternions_to_vector(shape1.joints);
    // std::vector<double> shape2_vec = quaternions_to_vector(shape2.joints);
    std::vector<double> shape1_vec = quaternions_to_relevant_euler_vector(shape1.joints);
    std::vector<double> shape2_vec = quaternions_to_relevant_euler_vector(shape2.joints);

    // Compute cosine similarity
    double cosine_similarity = get_cosine_similarity(shape1_vec, shape2_vec);
    double cosine_distance = 1.0 - cosine_similarity;  // Convert similarity to distance

    double euclid = get_euclidean_distance(shape1_vec, shape2_vec);

    double manhattan = get_manhattan_distance(shape1_vec, shape2_vec);

    double distance = euclid;

    return distance;
}

std::vector<Match> Poseshape::match_poses(const std::vector<Pose>& poses, const Pose& input_pose) {
    std::vector<Match> matches;
    for (size_t i = 0; i < poses.size(); ++i) {
        double distance = Poseshape::pose_distance(poses[i], input_pose);
        matches.push_back({static_cast<int>(i), distance});
    }

    // Sort matches by distance
    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
        return a.distance < b.distance;
    });

    return matches;
}

void get_matching_poses(const Pose& input_pose, const std::string& handshape_dir, const std::vector<std::string>& bones, std::vector<Match>& output_matches, std::vector<std::string>& output_handshape_names) {
    // Get all handshape files
    std::vector<std::string> handshape_files = Poseshape::get_all_handshape_files(handshape_dir);

    // Get poses from files
    std::vector<PoseInfo> poses_info = Poseshape::get_poses_from_files(handshape_files, bones);

    // Extract poses from PoseInfo
    std::vector<Pose> poses;
    std::vector<std::string> handshape_names;
    for (const auto& pose_info : poses_info) {
        poses.push_back(pose_info.handshape);
        output_handshape_names.push_back(pose_info.name);
    }

    // Match poses
    output_matches = Poseshape::match_poses(poses, input_pose);
}

std::vector<MatchInfo> Poseshape::get_matches(const SegmentData& segment_data, const std::string& segment_config_file, const std::string& matching_config_file) {
    // Load configs
    Config seg_config = Config::load_from_file(segment_config_file);
    MatchingConfig match_config = MatchingConfig::load_from_file(matching_config_file);

    Pose input_pose = segment_data.average_pose;
    
    std::vector<Match> matches;
    std::vector<std::string> handshape_names;
    get_matching_poses(input_pose, match_config.handshape_dir, seg_config.bones, matches, handshape_names);
    std::vector<MatchInfo> match_info;
    for (const auto& match : matches) {
        match_info.push_back({handshape_names[match.handshape_id], match});
    }

    return match_info;
}