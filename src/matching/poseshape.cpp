#include <filesystem>
#include <variant>

#include "config/config.h"

#include "poseshape.h"

using namespace math;

using namespace poseshape;

std::vector<std::string> Poseshape::get_all_handshape_files() {
    std::vector<std::string> handshape_files;
    std::string handshape_dir = "../HandformSession/HandformsTrimmed/";

    // Go through all folders, in the folder get the fbx file located in unreal/Vicon/
    for (const auto& entry : std::filesystem::directory_iterator(handshape_dir)) {
        if (entry.is_regular_file() &&
            (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            handshape_files.push_back(entry.path().string());
        }
    }
    return handshape_files;
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

Pose Poseshape::get_average_pose_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment) {
    Pose average_pose;
    average_pose.joint_names.reserve(handshape_tracks.size());
    average_pose.joints.reserve(handshape_tracks.size());

    // Since the hand may still be moving slightly after the acceleration maximum, take the middle 50% frames
    int segment_length = segment.end - segment.start + 1;
    int start_frame = segment.start + segment_length / 4;
    int end_frame = segment.end - segment_length / 4;

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

struct FingerRepresentation {
    double curl;
    double spread;
};

struct SingleRepresentation {
    std::string representation_name;
    std::variant<FingerRepresentation, Eigen::Quaterniond> representation;
};

struct PoseRepresentation {
    std::vector<SingleRepresentation> finger_representations;
};

// TODO: Write function that converts config lists of combinatorial joints to its representation.
PoseRepresentation convert_pose_to_representation(const std::vector<BoneCombination>& bone_combinations, const Pose& pose) {
    PoseRepresentation representation;
    for (const auto& combination : bone_combinations) {
        if (combination.IsFinger) {
            // Calculate curl and spread for the finger
            FingerRepresentation finger_rep;

            double total_curl = 0.0;
            double total_spread = 0.0;
            for (const auto& bone_name : combination.Bones) {
                auto it = std::find(pose.joint_names.begin(), pose.joint_names.end(), bone_name);
                if (it != pose.joint_names.end()) {
                    size_t index = std::distance(pose.joint_names.begin(), it);
                    Eigen::Vector3d euler_angles = pose.joints[index].toRotationMatrix().eulerAngles(0, 1, 2);
                    total_curl += euler_angles[2];  // Z-axis rotation for curl
                    total_spread += euler_angles[1];  // Y-axis rotation for spread
                }
            }

            finger_rep.curl = total_curl / static_cast<double>(combination.Bones.size());
            finger_rep.spread = total_spread / static_cast<double>(combination.Bones.size());

            representation.finger_representations.push_back({combination.Bones[0], finger_rep});
        } else {
            // Handle non-finger bones
            Eigen::Quaterniond avg_quat(0, 0, 0, 0);
            for (const auto& bone_name : combination.Bones) {
                auto it = std::find(pose.joint_names.begin(), pose.joint_names.end(), bone_name);
                if (it != pose.joint_names.end()) {
                    size_t index = std::distance(pose.joint_names.begin(), it);
                    avg_quat.coeffs() += pose.joints[index].coeffs();
                }
            }
            avg_quat.coeffs() /= static_cast<double>(combination.Bones.size());
            avg_quat.normalize();
            representation.finger_representations.push_back({combination.Bones[0], avg_quat});
        }
    }
    return representation;
}

double Poseshape::pose_distance(const Pose& shape1, const Pose& shape2) {
    if (shape1.joints.size() != shape2.joints.size()) {
        printf("Error: Poses must have the same number of quaternions. Shape1 has %zu, Shape2 has %zu\n", shape1.joints.size(), shape2.joints.size());
        throw std::invalid_argument("Poses must have the same number of quaternions.");
    }

    Config config = Config::load_from_file("config.json");

    PoseRepresentation rep1 = convert_pose_to_representation(config.matching_bone_combinations, shape1);
    PoseRepresentation rep2 = convert_pose_to_representation(config.matching_bone_combinations, shape2);

    // ------------------------------------------------------------------------------
    double angular_distance = 0.0;
    for (size_t i = 0; i < shape1.joints.size(); ++i)
        angular_distance += Math::radial_angle_between_quats(shape1.joints[i], shape2.joints[i]);

    angular_distance /= static_cast<double>(shape1.joints.size());
    // ------------------------------------------------------------------------------

    double distance = 0.0;

    for (size_t i = 0; i < rep1.finger_representations.size(); ++i) {
        const auto& rep1_variant = rep1.finger_representations[i].representation;
        const auto& rep2_variant = rep2.finger_representations[i].representation;

        if (std::holds_alternative<FingerRepresentation>(rep1_variant) && std::holds_alternative<FingerRepresentation>(rep2_variant)) {
            const FingerRepresentation& finger1 = std::get<FingerRepresentation>(rep1_variant);
            const FingerRepresentation& finger2 = std::get<FingerRepresentation>(rep2_variant);

            double curl_diff = finger1.curl - finger2.curl;
            double spread_diff = finger1.spread - finger2.spread;
            spread_diff *= 0.3;

            // Use Euclidean distance for the finger representation
            double cur_distance = std::sqrt(curl_diff * curl_diff + spread_diff * spread_diff);
            // Accumulate the distance
            distance += cur_distance;
        } else if (std::holds_alternative<Eigen::Quaterniond>(rep1_variant) && std::holds_alternative<Eigen::Quaterniond>(rep2_variant)) {
            const Eigen::Quaterniond& quat1 = std::get<Eigen::Quaterniond>(rep1_variant);
            const Eigen::Quaterniond& quat2 = std::get<Eigen::Quaterniond>(rep2_variant);

            // Use angular distance for the quaternion representation
            double cur_distance = Math::radial_angle_between_quats(quat1, quat2);
            // Accumulate the distance
            distance += cur_distance;
        } else {
            printf("Error: Mismatched representation types for %s\n", rep1.finger_representations[i].representation_name.c_str());
            throw std::invalid_argument("Mismatched representation types.");
        }
    }

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