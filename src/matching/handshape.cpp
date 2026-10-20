#include <filesystem>

#include "handshape.h"

using namespace math;

using namespace handshape;

std::vector<std::string> Handshape::get_all_handshape_files() {
    std::vector<std::string> handshape_files;
    std::string handshape_dir = "../HandformSession/2026-10-19/";

    // Go through all folders, in the folder get the fbx file located in unreal/Vicon/
    for (const auto& entry : std::filesystem::directory_iterator(handshape_dir)) {
        if (entry.is_directory()) {
            std::string unreal_vicon_path = entry.path().string() + "/unreal/Vicon/";
            for (const auto& sub_entry : std::filesystem::directory_iterator(unreal_vicon_path)) {
                if (sub_entry.is_regular_file() &&
                    (sub_entry.path().extension().string() == ".fbx" || sub_entry.path().extension().string() == ".FBX")) {
                    handshape_files.push_back(sub_entry.path().string());
                }
            }
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

Hand Handshape::get_average_handshape_from_segment(const std::vector<JointTrack>& handshape_tracks, const Segment& segment) {
    Hand average_handshape;
    average_handshape.joint_names.reserve(handshape_tracks.size());
    average_handshape.joints.reserve(handshape_tracks.size());

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
        average_handshape.joints.push_back(avg_quat);
        average_handshape.joint_names.push_back(handshape_tracks[i].name);
    }

    return average_handshape;
}

double Handshape::pose_distance(const Hand& shape1, const Hand& shape2) {
    if (shape1.joints.size() != shape2.joints.size()) {
        printf("Error: Handshapes must have the same number of quaternions. Shape1 has %zu, Shape2 has %zu\n", shape1.joints.size(), shape2.joints.size());
        throw std::invalid_argument("Handshapes must have the same number of quaternions.");
    }

    double angular_distance = 0.0;
    for (size_t i = 0; i < shape1.joints.size(); ++i)
        angular_distance += Math::radial_angle_between_quats(shape1.joints[i], shape2.joints[i]);

    angular_distance /= static_cast<double>(shape1.joints.size());

    return angular_distance;
}

std::vector<Match> Handshape::match_handshapes(const std::vector<Hand>& handshapes, const Hand& input_shape) {
    std::vector<Match> matches;
    for (size_t i = 0; i < handshapes.size(); ++i) {
        double distance = Handshape::pose_distance(handshapes[i], input_shape);
        matches.push_back({static_cast<int>(i), distance});
    }

    // Sort matches by distance
    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
        return a.distance < b.distance;
    });

    return matches;
}