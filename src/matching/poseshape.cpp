#include <filesystem>

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

double Poseshape::pose_distance(const Pose& shape1, const Pose& shape2) {
    if (shape1.joints.size() != shape2.joints.size()) {
        printf("Error: Poses must have the same number of quaternions. Shape1 has %zu, Shape2 has %zu\n", shape1.joints.size(), shape2.joints.size());
        throw std::invalid_argument("Poses must have the same number of quaternions.");
    }

    double angular_distance = 0.0;
    for (size_t i = 0; i < shape1.joints.size(); ++i)
        angular_distance += Math::radial_angle_between_quats(shape1.joints[i], shape2.joints[i]);

    angular_distance /= static_cast<double>(shape1.joints.size());

    return angular_distance;
}

std::vector<Match> Poseshape::match_poses(const std::vector<Pose>& poshes, const Pose& input_pose) {
    std::vector<Match> matches;
    for (size_t i = 0; i < poshes.size(); ++i) {
        double distance = Poseshape::pose_distance(poshes[i], input_pose);
        matches.push_back({static_cast<int>(i), distance});
    }

    // Sort matches by distance
    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
        return a.distance < b.distance;
    });

    return matches;
}