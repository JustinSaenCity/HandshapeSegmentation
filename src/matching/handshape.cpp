#include <vector>
#include <Eigen/Geometry>

#include "types.h"

#include "handshape.h"

using namespace handshape;

// --------------------------------
// Representation
// --------------------------------

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

// --------------------------------
// TODO: PLACE IN ANOTHER FILE
// --------------------------------

#include "config/matching_config.h"
#include "anim/anim.h"
#include "math/math.h"
#include "segmentation.h"

using namespace anim;
using namespace math;
using namespace segmentation;

#include <Eigen/Dense>

struct HandFrame { Eigen::Vector3d origin; Eigen::Matrix3d R; bool valid; };

HandFrame make_hand_frame(const Eigen::Vector3d& middle_metacarpal,
                          const Eigen::Vector3d& middle_proximal,
                          const Eigen::Vector3d& index_metacarpal,
                          const Eigen::Vector3d& pinky_metacarpal) 
{
    HandFrame f; 
    f.valid = false;

    Eigen::Vector3d z = middle_proximal - middle_metacarpal;
    if (z.norm() < 1e-8) return f;
    z.normalize();

    Eigen::Vector3d y = index_metacarpal - pinky_metacarpal;
    y -= y.dot(z) * z;
    if (y.norm() < 1e-8) return f;
    y.normalize();

    Eigen::Vector3d x = y.cross(z).normalized();

    f.R.col(0) = x; f.R.col(1) = y; f.R.col(2) = z;
    f.origin = middle_metacarpal;
    f.valid = true;

    return f;
}

HandSkeleton get_average_handskeleton_from_segment(
        const std::vector<JointLocationTrack>& tracks, const Segment& segment,
        const json& bone_mapping, bool take_middle_50_percent) 
{
    int segment_length = segment.end - segment.start + 1;
    int start_frame = take_middle_50_percent ? segment.start + segment_length / 4 : segment.start;
    int end_frame = take_middle_50_percent ? segment.end - segment_length / 4 : segment.end;

    struct Entry { const JointLocationTrack* track = nullptr; Eigen::Vector3d HandSkeleton::* member; const char* finger; const char* joint; };
    std::vector<Entry> entries = {
        {nullptr, &HandSkeleton::thumb_metacarpal, "thumb", "metacarpal"},
        {nullptr, &HandSkeleton::thumb_proximal,   "thumb", "proximal"},
        {nullptr, &HandSkeleton::thumb_distal,     "thumb", "distal"},
        {nullptr, &HandSkeleton::thumb_tip,        "thumb", "tip"},
        {nullptr, &HandSkeleton::index_metacarpal, "index", "metacarpal"},
        {nullptr, &HandSkeleton::index_proximal,   "index", "proximal"},
        {nullptr, &HandSkeleton::index_middle,     "index", "middle"},
        {nullptr, &HandSkeleton::index_distal,     "index", "distal"},
        {nullptr, &HandSkeleton::index_tip,        "index", "tip"},
        {nullptr, &HandSkeleton::middle_metacarpal,"middle", "metacarpal"},
        {nullptr, &HandSkeleton::middle_proximal,  "middle", "proximal"},
        {nullptr, &HandSkeleton::middle_middle,    "middle", "middle"},
        {nullptr, &HandSkeleton::middle_distal,    "middle", "distal"},
        {nullptr, &HandSkeleton::middle_tip,       "middle", "tip"},
        {nullptr, &HandSkeleton::ring_metacarpal,  "ring", "metacarpal"},
        {nullptr, &HandSkeleton::ring_proximal,    "ring", "proximal"},
        {nullptr, &HandSkeleton::ring_middle,      "ring", "middle"},
        {nullptr, &HandSkeleton::ring_distal,      "ring", "distal"},
        {nullptr, &HandSkeleton::ring_tip,         "ring", "tip"},
        {nullptr, &HandSkeleton::pinky_metacarpal, "pinky", "metacarpal"},
        {nullptr, &HandSkeleton::pinky_proximal,   "pinky", "proximal"},
        {nullptr, &HandSkeleton::pinky_middle,     "pinky", "middle"},
        {nullptr, &HandSkeleton::pinky_distal,     "pinky", "distal"},
        {nullptr, &HandSkeleton::pinky_tip,        "pinky", "tip"},
    };

    // Fill in the tracks for each entry
    for (auto& e : entries)
        for (const auto& t : tracks)
            if (t.name == bone_mapping.at(e.finger).at(e.joint).get<std::string>()) { e.track = &t; break; }

    // Helper function to find the track based on the bone type
    auto track_of = [&](Eigen::Vector3d HandSkeleton::* m) -> const JointLocationTrack* {
        for (auto& e : entries) if (e.member == m) return e.track;
        return nullptr;
    };
    auto* t_mid_metacarpal = track_of(&HandSkeleton::middle_metacarpal);
    auto* t_mid_proximal = track_of(&HandSkeleton::middle_proximal);
    auto* t_idx_metacarpal = track_of(&HandSkeleton::index_metacarpal);
    auto* t_pnk_metacarpal = track_of(&HandSkeleton::pinky_metacarpal);

    // For each frame in the segment, make a handframe and add the transformed joint positions to the sum
    HandSkeleton avg;
    std::vector<Eigen::Vector3d> sum(entries.size(), Eigen::Vector3d::Zero());
    int used = 0;
    for (int t = start_frame; t <= end_frame; ++t) {  // Each frame
        HandFrame f = make_hand_frame(t_mid_metacarpal->locations[t], t_mid_proximal->locations[t],
                                      t_idx_metacarpal->locations[t], t_pnk_metacarpal->locations[t]);
        if (!f.valid) continue;

        for (size_t i = 0; i < entries.size(); ++i)  // Each joint
            if (entries[i].track)
                sum[i] += f.R.transpose() * (entries[i].track->locations[t] - f.origin);
        ++used;
    }
    if (used == 0) return avg;

    for (size_t i = 0; i < entries.size(); ++i)
        if (entries[i].track) avg.*(entries[i].member) = sum[i] / used;

    return avg;
}

std::vector<HandSkeleton> get_handskeletons_from_files(const std::vector<std::string>& handshape_files, const std::vector<std::string>& bones, const json& bone_mapping) {
    std::vector<HandSkeleton> handshapes;
    for (const auto& handshape_file : handshape_files) {
        double handshape_fps;
        std::vector<JointLocationTrack> handshape_location_tracks;
        Anim::get_location_tracks_from_fbx(handshape_file, bones, handshape_location_tracks, handshape_fps);
        if (handshape_location_tracks.empty()) continue;

        // Process the tracks
        HandSkeleton average_handshape = get_average_handskeleton_from_segment(handshape_location_tracks, Segment{0, static_cast<int>(handshape_location_tracks[0].locations.size() - 1)}, bone_mapping, true);
        handshapes.push_back(average_handshape);
    }
    return handshapes;
}

#include <iostream>

void print_handskeleton(const HandSkeleton& skeleton) {
    std::cout << "Thumb: " << skeleton.thumb_metacarpal.transpose() << ", " << skeleton.thumb_proximal.transpose() << ", " << skeleton.thumb_distal.transpose() << ", " << skeleton.thumb_tip.transpose() << std::endl;
    std::cout << "Index: " << skeleton.index_metacarpal.transpose() << ", " << skeleton.index_proximal.transpose() << ", " << skeleton.index_middle.transpose() << ", " << skeleton.index_distal.transpose() << ", " << skeleton.index_tip.transpose() << std::endl;
    std::cout << "Middle: " << skeleton.middle_metacarpal.transpose() << ", " << skeleton.middle_proximal.transpose() << ", " << skeleton.middle_middle.transpose() << ", " << skeleton.middle_distal.transpose() << ", " << skeleton.middle_tip.transpose() << std::endl;
    std::cout << "Ring: " << skeleton.ring_metacarpal.transpose() << ", " << skeleton.ring_proximal.transpose() << ", " << skeleton.ring_middle.transpose() << ", " << skeleton.ring_distal.transpose() << ", " << skeleton.ring_tip.transpose() << std::endl;
    std::cout << "Pinky: " << skeleton.pinky_metacarpal.transpose() << ", " << skeleton.pinky_proximal.transpose() << ", " << skeleton.pinky_middle.transpose() << ", " << skeleton.pinky_distal.transpose() << ", " << skeleton.pinky_tip.transpose() << std::endl;
}

// --------------------------------

double handskeleton_distance(const HandSkeleton& shape1, const HandSkeleton& shape2, const MatchingMethod method) {
    std::vector<double> shape1_vec = Handshape::compute_handshape_representation(shape1);
    std::vector<double> shape2_vec = Handshape::compute_handshape_representation(shape2);

    switch (method) {
        case MatchingMethod::Euclidean:
            return Math::get_euclidean_distance(shape1_vec, shape2_vec);
        case MatchingMethod::Cosine:
            return 1.0 - Math::get_cosine_similarity(shape1_vec, shape2_vec);  // Convert similarity to distance
        case MatchingMethod::Manhattan:
            return Math::get_manhattan_distance(shape1_vec, shape2_vec);
        default:
            throw std::invalid_argument("Unknown matching method.");
    }
}

std::vector<Match> match_Handskeletons(const std::vector<HandSkeleton>& handskeletons, const HandSkeleton& input_handskeleton, const MatchingMethod method) {
    std::vector<Match> matches;
    for (size_t i = 0; i < handskeletons.size(); ++i) {
        double distance = handskeleton_distance(handskeletons[i], input_handskeleton, method);
        matches.push_back({static_cast<int>(i), distance});
    }

    // Sort matches by distance
    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
        return a.distance < b.distance;
    });

    return matches;
}

struct HandSkeletonInfo {
    std::string name;
    HandSkeleton handskeleton;
};

void create_distance_matrix_from_handskeletons(const std::vector<HandSkeletonInfo>& handshapes, const MatchingMethod method, std::string output_file)
{
    int N = handshapes.size();
    std::vector<std::vector<double>> distance_matrix(N, std::vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double distance = handskeleton_distance(handshapes[i].handskeleton, handshapes[j].handskeleton, method);
            distance_matrix[i][j] = distance;
            distance_matrix[j][i] = distance; // Symmetric
        }
    }

    std::ofstream file(output_file);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open output file: " + output_file);
    }

    // Write header
    file << "Name";
    for (const auto& handshape_info : handshapes) {
        file << "," << handshape_info.name;
    }

    file << "\n";

    // Write distance matrix
    for (int i = 0; i < N; ++i) {
        file << handshapes[i].name;  // Write the name of the handshape
        for (int j = 0; j < N; ++j) {
            file << "," << distance_matrix[i][j];
        }
        file << "\n";
    }

    file.close();
}

HandSkeletonInfo get_handskeletoninfo_from_file(const std::string& handshape_file, const std::vector<std::string>& bones, const json& bone_mapping, const std::string& string_addition = "", const Segment& segment = Segment{0, -1}, bool take_middle_50_percent = false) {

    double handshape_fps;
    std::vector<JointLocationTrack> handshape_location_tracks;
    Anim::get_location_tracks_from_fbx(handshape_file, bones, handshape_location_tracks, handshape_fps);
    if (handshape_location_tracks.empty()) return {"", HandSkeleton()};

    // Handle the case where the segment end is -1, which means we want to use the entire track
    Segment effective_segment = segment;
    if (segment.end == -1) effective_segment.end = static_cast<int>(handshape_location_tracks[0].locations.size() - 1);

    // Process the tracks
    HandSkeleton average_handshape = get_average_handskeleton_from_segment(handshape_location_tracks, effective_segment, bone_mapping, take_middle_50_percent);
    std::string name = handshape_file.substr(handshape_file.find_last_of("/\\") + 1); // Extract filename from path
    if (!string_addition.empty()) name += "_" + string_addition;

    return {name, average_handshape};
}

void Handshape::create_distance_matrix_from_handshapefiles(const std::vector<std::string>& handshape_files, const std::vector<std::string>& input_files, const std::vector<std::string>& bones, const json& bone_mapping, const MatchingMethod method, const std::string& output_file) {
    std::vector<HandSkeletonInfo> handskeletons_info;

    for (const auto& handshape_file : handshape_files) {
        HandSkeletonInfo handskeleton_info = get_handskeletoninfo_from_file(handshape_file, bones, bone_mapping);
        if (!handskeleton_info.name.empty())
            handskeletons_info.push_back(handskeleton_info);
    }

    for (const auto& input_file : input_files) {
        double fps;
        std::vector<JointTrack> tracks;
        Anim::get_quaternion_tracks_from_fbx(input_file, {}, tracks, fps);
        if (tracks.empty()) continue;

        std::vector<SegmentData> segments = Segmentation::get_segments(tracks, fps, "./segmentation_config.json", true);

        for (int i = 0; i < segments.size(); ++i) {
            HandSkeletonInfo handskeleton_info = get_handskeletoninfo_from_file(input_file, bones, bone_mapping, std::to_string(i), segments[i].segment, true);
            if (!handskeleton_info.name.empty())
                handskeletons_info.push_back(handskeleton_info);
        }
    }

    create_distance_matrix_from_handskeletons(handskeletons_info, method, output_file);
}
