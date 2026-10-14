#include <ufbx.h>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>

#include <cstdio>
#include <string>
#include <vector>

#include "anim/anim.h"
#include "config/config.h"

#include "handseg.h"

using namespace math;
using namespace anim;
using namespace handseg;

void plot_acceleration(const std::vector<double>& smoothed_data, const double& dt, const std::vector<struct Segment>& segs)
{
    std::vector<double> x(smoothed_data.size());
    for (size_t i = 0; i < smoothed_data.size(); ++i) x[i] = i * dt;

    // matplotlibcpp::figure();
    // matplotlibcpp::plot(x, speed);

    // // Show handmade lines for self input segments, each segment its own colour
    // auto plotsegs = {
    //     Segment{0, 83},
    //     Segment{86, 89},
    //     Segment{92, 95},
    //     Segment{100, 132},
    //     Segment{136, 138},
    //     Segment{142, 147},
    //     Segment{152, 155},
    //     Segment{163, 169},
    //     Segment{175, 177},
    //     Segment{180, 182},
    //     Segment{191, 216}
    // };

    // for (const auto& s : plotsegs) {
    //     std::vector<double> seg_x, seg_y;
    //     for (int i = s.start; i <= s.end; ++i) {
    //         seg_x.push_back(i * dt);
    //         seg_y.push_back(speed[i]);
    //     }
    //     matplotlibcpp::plot(seg_x, seg_y);
    // }

    matplotlibcpp::figure();
    matplotlibcpp::plot(x, smoothed_data);

    for (const auto& s : segs) {
        std::vector<double> seg_x, seg_y;
        for (int i = s.start; i <= s.end; ++i) {
            seg_x.push_back(i * dt);
            seg_y.push_back(smoothed_data[i]);
        }
        matplotlibcpp::plot(seg_x, seg_y);
    }

    std::filesystem::create_directory("output");
    matplotlibcpp::save("output/acceleration_plot.png");
}

std::vector<Segment> Handseg::get_segments(char** argv, bool save_to_csv, bool plot)
{
    // Load config data from config.json
    Config config = Config::load_from_file("config.json");

    // Load the FBX file
    ufbx_error error;
    ufbx_scene* scene;
    if (Anim::load_fbx_scene(argv[1], &scene))
        return {};

    // Find the finger bones
    std::vector<ufbx_node*> nodes;
    std::vector<JointTrack> tracks;
    int num_fingers = Anim::get_bones_from_scene(scene, config.bone_filter, &nodes, &tracks);
    if (num_fingers == 0) {
        std::puts("No finger bones found, please adjust the filter.");
        ufbx_free_scene(scene);
        return {};
    }

    // Sample the joint tracks for each finger bone
    const ufbx_anim* anim = scene->anim;
    Anim::fill_joint_tracks(anim, nodes, tracks);

    // Filter right hand tracks and apply preprocessing
    std::vector<std::vector<Eigen::Quaterniond>> right_hand_quaternion_tracks;
    for (const auto& tr : tracks)
        if (tr.name.find("_r") != std::string::npos)
            right_hand_quaternion_tracks.push_back(tr.q);

    for (auto& quaternion_track : right_hand_quaternion_tracks) { 
        Math::align_quaternion_signs(quaternion_track);
        Math::smooth_quaternions(quaternion_track, 2.0);
    }

    // Aggregate the quaternion data into a single vector of values for segmentation
    const double fps = anim->time_end > 0 ? scene->settings.frames_per_second : 24.0;
    const double dt = 1.0 / fps;
    std::vector<double> aggregate_data = Math::aggregate_quaternion_data(right_hand_quaternion_tracks, dt, config.segment_by);
    std::vector<double> smoothed_data = aggregate_data;
    Math::smooth_values(smoothed_data, config.smooth_segment_values_sigma);

    // Segment the handshapes based on speed thresholds
    std::vector<Segment> segs = Math::segment_handshapes_by_acceleration(smoothed_data, config.acceleration_threshold_factor, config.min_segment_length);


    if (save_to_csv) {
        std::filesystem::create_directory("output");
        std::ofstream csv_file("output/handshape_segments.csv");
        csv_file << "start,end\n";
        for (const auto& s : segs) csv_file << s.start << "," << s.end << "\n";
        csv_file.close();
    }

    if (plot)
        plot_acceleration(smoothed_data, dt, segs);

    ufbx_free_scene(scene);

    return segs;
}

std::vector<std::tuple<int, int>> Handseg::get_segments_as_tuples(char** argv)
{
    std::vector<Segment> segments = get_segments(argv);
    std::vector<std::tuple<int, int>> segment_tuples;
    for (const auto& s : segments) {
        segment_tuples.emplace_back(s.start, s.end);
    }
    return segment_tuples;
}