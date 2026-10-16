#include <ufbx.h>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>

#include <cstdio>
#include <string>
#include <vector>

#include "math/math.h"
#include "anim/anim.h"
#include "config/config.h"
#include "matching/handshape.h"
#include "handseg.h"

using namespace math;
using namespace anim;
using namespace handseg;
using namespace handshape;

int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Usage: .\\handseg.exe <file.fbx>"); return 1; }

    auto segs = Handseg::get_segments(argv, true, true);

    printf("Segmentation complete. Results saved to output/handshape_segments.csv and output/acceleration_plot.png\n");

    //---------------------------------------------------------------------------------------------
    // Load config
    Config config = Config::load_from_file("config.json");

    // Load the quaternion tracks from the FBX file
    double fps;
    std::vector<JointTrack> tracks;
    Handseg::get_quaternion_tracks_from_fbx(argv[1], config.bone_filter, tracks, fps);
    if (tracks.empty()) return 1;

    // Filter right hand tracks and apply preprocessing
    std::vector<JointTrack> right_hand_quaternion_tracks;
    for (const auto& tr : tracks)
        if (tr.name.find("_r") != std::string::npos)
            right_hand_quaternion_tracks.push_back(tr);

    //---------------------------------------------------------------------------------------------
    // LOAD ALL HANSHAPE FILES AND EXTRACT THEIR QUATERNION TRACKS
    std::vector<std::string> handshape_files = Handshape::get_all_handshape_files();
    std::vector<Hand> handshapes_average_quaternion_tracks;

    for (const auto& file : handshape_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Handseg::get_quaternion_tracks_from_fbx(file, config.bone_filter, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Filter right hand tracks and apply preprocessing
        std::vector<JointTrack> right_hand_tracks;
        for (const auto& tr : handshape_tracks)
            if (tr.name.find("_r") != std::string::npos)
                right_hand_tracks.push_back(tr);

        const Segment& segment = Segment(0, right_hand_tracks[0].q.size() - 1);
        handshapes_average_quaternion_tracks.push_back(Handshape::get_average_handshape_from_segment(right_hand_tracks, segment));
    }

    //---------------------------------------------------------------------------------------------
    // MATCH THE INPUT HANDSHAPE TO THE PREDEFINED HANSHAPES
    Hand average_input_handshape = Handshape::get_average_handshape_from_segment(right_hand_quaternion_tracks, segs[1]);
    std::vector<Match> matches = Handshape::match_handshapes(handshapes_average_quaternion_tracks, average_input_handshape);

    for (const Match& match : matches) {
        std::string handshape_name = handshape_files[match.handshape_id];
        double distance = match.distance;
        printf("Handshape: %s, Distance: %.4f\n", handshape_name.c_str(), distance);
    }

    return 0;
}