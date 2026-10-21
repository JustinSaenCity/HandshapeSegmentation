#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>

#include "config/config.h"

#include "segmentation.h"

using namespace math;
using namespace anim;
using namespace segmentation;

void Segmentation::get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps)
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
    const double fps = scene->settings.frames_per_second;
    Anim::fill_joint_tracks(anim, nodes, fps, tracks);

    out_tracks = tracks;
    out_fps = scene->settings.frames_per_second;

    ufbx_free_scene(scene);
}

std::vector<Segment> Segmentation::get_segments(const std::vector<JointTrack>& tracks, const double fps, 
                                           const std::string& config_file, bool save_to_csv)
{
    // Load config data from config.json
    Config config = Config::load_from_file(config_file);

    // Filter tracks by only taking bones that are listed in the config file
    std::vector<JointTrack> filtered_tracks;
    for (const JointTrack& tr : tracks) {
        for (const std::string& bone : config.segmentation_bones) {
            if (tr.name.find(bone) != std::string::npos) {
                filtered_tracks.push_back(tr);
                break;
            }
        }
    }

    // Preprocess the tracks
    for (JointTrack& track : filtered_tracks) { 
        Math::align_quaternion_signs(track.q);
        Math::smooth_quaternions(track.q, 2.0);
    }

    // Aggregate data into a single vector of values for segmentation
    const double dt = 1.0 / fps;
    std::vector<std::vector<Eigen::Quaterniond>> quaternion_tracks_only;
    for (const JointTrack& tr : filtered_tracks) quaternion_tracks_only.push_back(tr.q);
    std::vector<double> aggregate_data = Math::aggregate_quaternion_data(quaternion_tracks_only, dt);
    std::vector<double> smoothed_data = aggregate_data;
    Math::smooth_values(smoothed_data, config.segmentation_smoothing_sigma);

    // Segment the data based on acceleration
    std::vector<Segment> segments = Math::segment_handshapes_by_acceleration(smoothed_data, config.segmentation_threshold_factor,
                                    config.segmentation_min_segment_length, config.segmentation_min_acceleration_threshold);

    if (save_to_csv) {
        std::filesystem::create_directory("output");
        std::ofstream csv_file("output/handshape_segments.csv");
        csv_file << "start,end\n";
        for (const auto& s : segments) csv_file << s.start << "," << s.end << "\n";
        csv_file.close();

        std::ofstream csv_file2("output/handshape_acceleration.csv");
        csv_file2 << "frame,time,acceleration\n";
        for (size_t i = 0; i < smoothed_data.size(); ++i) csv_file2 << i << "," << i * dt << "," << smoothed_data[i] << "\n";
        csv_file2.close();
    }
    
    return segments;
}

std::vector<std::tuple<int,int>> Segmentation::get_segments_as_tuples(const std::vector<JointTrack>& tracks, const double fps, 
                                                                 const std::string& config_file, bool save_to_csv)
{
    std::vector<Segment> segments = Segmentation::get_segments(tracks, fps, config_file);
    std::vector<std::tuple<int,int>> segments_as_tuples;

    for (const auto& s : segments) segments_as_tuples.push_back(std::make_tuple(s.start, s.end));
    
    return segments_as_tuples;
}
