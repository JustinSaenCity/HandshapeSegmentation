#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <string>
#include <vector>

#include "anim/anim.h"
#include "config/config.h"

#include "handseg.h"

using namespace math;
using namespace anim;
using namespace handseg;

void Handseg::get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps)
{
    // Load the FBX file
    ufbx_error error;
    ufbx_scene* scene;
    if (Anim::load_fbx_scene(fbx_file.c_str(), &scene))
        return;

    // Find the finger bones
    std::vector<ufbx_node*> nodes;
    std::vector<JointTrack> tracks;
    int num_fingers = Anim::get_bones_from_scene(scene, bone_filter, &nodes, &tracks);
    if (num_fingers == 0) {
        printf("No finger bones found in %s, please adjust the filter.\n", fbx_file.c_str());
        ufbx_free_scene(scene);
        return;
    }

    // Sample the joint tracks for each finger bone
    const ufbx_anim* anim = scene->anim;
    Anim::fill_joint_tracks(anim, nodes, tracks);

    out_tracks = tracks;
    out_fps = scene->settings.frames_per_second;

    ufbx_free_scene(scene);
}

std::tuple<std::vector<Segment>, std::vector<Segment>> Handseg::get_segments(const std::string& fbx_file, bool save_to_csv)
{
    // Load config data from config.json
    Config config = Config::load_from_file("config.json");

    // Load the quaternion tracks from the FBX file
    double fps;
    std::vector<JointTrack> tracks;
    Handseg::get_quaternion_tracks_from_fbx(fbx_file, config.bone_filter, tracks, fps);
    if (tracks.empty()) {
        std::puts("No tracks found, please adjust the filter.");
        return {};
    }

    // Filter tracks by hand
    std::vector<JointTrack> right_hand_quaternion_tracks;
    std::vector<JointTrack> left_hand_quaternion_tracks;
    for (const JointTrack& tr : tracks)
        if (tr.name.find(config.right_hand_filter) != std::string::npos)
            right_hand_quaternion_tracks.push_back(tr);
        else if (tr.name.find(config.left_hand_filter) != std::string::npos)
            left_hand_quaternion_tracks.push_back(tr);

    // Process the quaternion tracks
    for (JointTrack& quaternion_track : right_hand_quaternion_tracks) { 
        Math::align_quaternion_signs(quaternion_track.q);
        Math::smooth_quaternions(quaternion_track.q, 2.0);
    }
    for (JointTrack& quaternion_track : left_hand_quaternion_tracks) { 
        Math::align_quaternion_signs(quaternion_track.q);
        Math::smooth_quaternions(quaternion_track.q, 2.0);
    }

    // Aggregate the quaternion data into a single vector of values for segmentation
    const double dt = 1.0 / fps;
    std::vector<std::vector<Eigen::Quaterniond>> right_hand_quaternion_tracks_only;
    std::vector<std::vector<Eigen::Quaterniond>> left_hand_quaternion_tracks_only;
    for (const JointTrack& tr : right_hand_quaternion_tracks) right_hand_quaternion_tracks_only.push_back(tr.q);
    for (const JointTrack& tr : left_hand_quaternion_tracks) left_hand_quaternion_tracks_only.push_back(tr.q);
    std::vector<double> aggregate_right_data = Math::aggregate_quaternion_data(right_hand_quaternion_tracks_only, dt, config.segment_by);
    std::vector<double> aggregate_left_data = Math::aggregate_quaternion_data(left_hand_quaternion_tracks_only, dt, config.segment_by);
    std::vector<double> smoothed_right_data = aggregate_right_data;
    std::vector<double> smoothed_left_data = aggregate_left_data;
    Math::smooth_values(smoothed_right_data, config.smooth_segment_values_sigma);
    Math::smooth_values(smoothed_left_data, config.smooth_segment_values_sigma);

    // Segment the handshapes based on speed thresholds
    std::vector<Segment> right_segs = Math::segment_handshapes_by_acceleration(smoothed_right_data, config.acceleration_threshold_factor, config.min_segment_length, config.min_acceleration_threshold);
    std::vector<Segment> left_segs = Math::segment_handshapes_by_acceleration(smoothed_left_data, config.acceleration_threshold_factor, config.min_segment_length, config.min_acceleration_threshold);

    if (save_to_csv) {
        std::filesystem::create_directory("output");
        std::ofstream csv_file("output/handshape_segments.csv");
        csv_file << "hand,start,end\n";
        for (const auto& s : left_segs) csv_file << "left," << s.start << "," << s.end << "\n";
        for (const auto& s : right_segs) csv_file << "right," << s.start << "," << s.end << "\n";
        csv_file.close();

        std::ofstream csv_file2("output/handshape_acceleration.csv");
        csv_file2 << "hand,hand_index,time,acceleration\n";
        for (size_t i = 0; i < smoothed_right_data.size(); ++i) csv_file2 << "right," << i << "," << i * dt << "," << smoothed_right_data[i] << "\n";
        for (size_t i = 0; i < smoothed_left_data.size(); ++i) csv_file2 << "left," << i << "," << i * dt << "," << smoothed_left_data[i] << "\n";
        csv_file2.close();
    }

    return {left_segs, right_segs};
}

std::tuple<std::vector<std::tuple<int, int>>, std::vector<std::tuple<int, int>>> Handseg::get_segments_as_tuples(const std::string& fbx_file)
{
    std::tuple<std::vector<Segment>, std::vector<Segment>> segments = Handseg::get_segments(fbx_file);
    std::vector<std::tuple<int, int>> left_segs_as_tuples;
    std::vector<std::tuple<int, int>> right_segs_as_tuples;

    for (const auto& s : std::get<0>(segments)) left_segs_as_tuples.push_back(std::make_tuple(s.start, s.end));
    for (const auto& s : std::get<1>(segments)) right_segs_as_tuples.push_back(std::make_tuple(s.start, s.end));
    
    return {left_segs_as_tuples, right_segs_as_tuples};
}