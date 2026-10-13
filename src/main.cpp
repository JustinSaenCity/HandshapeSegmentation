#include <ufbx.h>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>

#include <cstdio>
#include <string>
#include <vector>

#include "math/math.h"
#include "anim/anim.h"

using namespace math;
using namespace anim;

int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Gebruik: handseg <bestand.fbx>"); return 1; }

    // Load the FBX file
    ufbx_error error;
    ufbx_scene* scene;
    if (Anim::load_fbx_scene(argv[1], &scene))
        return 1;

    // Find the finger bones
    std::vector<ufbx_node*> nodes;
    std::vector<JointTrack> tracks;
    int num_fingers = Anim::get_finger_bones_from_scene(scene, &nodes, &tracks);
    if (num_fingers == 0) {
        std::puts("No finger bones found, please adjust the filter.");
        ufbx_free_scene(scene);
        return 1;
    }

    // Sample the joint tracks for each finger bone
    const ufbx_anim* anim = scene->anim;
    Anim::fill_joint_tracks(anim, nodes, tracks);

    // Compute the angular speed of each finger bone track and segment the handshapes
    const double fps = anim->time_end > 0 ? 24.0 : 24.0;
    const double dt = 1.0 / fps;

    // Filter right hand tracks and apply preprocessing
    std::vector<std::vector<Eigen::Quaterniond>> right_hand_quaternion_tracks;
    for (const auto& tr : tracks)
        if (tr.name.find("_r") != std::string::npos)
            right_hand_quaternion_tracks.push_back(tr.q);

    for (auto& quaternion_track : right_hand_quaternion_tracks) { 
        Math::align_quaternion_signs(quaternion_track);
        Math::smooth_quaternions(quaternion_track, 2.0);
    }

    auto speed = Math::aggregate_speed(right_hand_quaternion_tracks, dt);

    // Plot the speed values
    std::vector<double> x(speed.size());
    for (size_t i = 0; i < speed.size(); ++i) x[i] = i * dt;

    matplotlibcpp::figure();
    matplotlibcpp::plot(x, speed);

    // Show handmade lines for self input segments, each segment its own colour
    auto plotsegs = {
        Segment{0, 83},
        Segment{86, 89},
        Segment{92, 95},
        Segment{100, 132},
        Segment{136, 138},
        Segment{142, 147},
        Segment{152, 155},
        Segment{163, 169},
        Segment{175, 177},
        Segment{180, 182},
        Segment{191, 216}
    };

    for (const auto& s : plotsegs) {
        std::vector<double> seg_x, seg_y;
        for (int i = s.start; i <= s.end; ++i) {
            seg_x.push_back(i * dt);
            seg_y.push_back(speed[i]);
        }
        matplotlibcpp::plot(seg_x, seg_y);
    }

    // Plot smoothed values seperately
    std::vector<double> smoothed_speed = speed;
    Math::smooth_values(smoothed_speed, 1.5);
    matplotlibcpp::figure();
    matplotlibcpp::plot(x, smoothed_speed);

    for (const auto& s : plotsegs) {
        std::vector<double> seg_x, seg_y;
        for (int i = s.start; i <= s.end; ++i) {
            seg_x.push_back(i * dt);
            seg_y.push_back(smoothed_speed[i]);
        }
        matplotlibcpp::plot(seg_x, seg_y);
    }

    matplotlibcpp::show();

    // Segment the handshapes based on speed thresholds
    std::vector<double> sorted = speed; std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];
    double low  = 1.0 * median;  // Value above which we consider a handshape to be in transition
    double high = 1.5 * median;  // Value below which we consider a handshape to be stable

    auto segs = Math::segment_handshapes(speed, low, high, 2);
    for (auto& s : segs) std::printf("handshape: %d - %d\n", s.start, s.end);

    ufbx_free_scene(scene);
    return 0;
}