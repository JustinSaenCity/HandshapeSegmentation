#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <string>
#include <vector>

#include "lib/types.h"

#include "lib/math/math.h"
#include "lib/anim/anim.h"
#include "lib/config/segmentation_config.h"
#include "lib/config/matching_config.h"
#include "matching/handshape.h"
#include "matching/handskeleton.h"
#include "segmentation/segmentation.h"

using namespace math;
using namespace anim;
using namespace segmentation;
using namespace handshape;
using namespace handskel;

int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Usage: .\\handseg.exe <file.fbx>"); return 1; }

    // Load configuration files
    Config config = Config::load_from_file("segmentation_config.json");
    MatchingConfig match_config = MatchingConfig::load_from_file("matching_config.json");

    // Get the quaternion tracks from the FBX file
    double fps;
    std::vector<JointTrack> tracks;
    Anim::get_quaternion_tracks_from_fbx(argv[1], {}, tracks, fps);

    /// ----------------------------------------------
    /// SEGMENTATION
    /// ----------------------------------------------

    // Get pre-match info, to set if we want to get the data for matching from the segmentation
    PreMatchInfo pre_match_info = PreMatchInfo();
    pre_match_info.do_pre = true;
    pre_match_info.bone_mapping = match_config.bone_mapping;
    pre_match_info.hand_location_tracks.clear();
    Anim::get_location_tracks_from_fbx(argv[1], {}, pre_match_info.hand_location_tracks, fps);

    // Get the segments from the joint tracks
    std::vector<SegmentData> segments = Segmentation::get_segments(
        tracks, fps, "./segmentation_config.json", true, true, pre_match_info);

    printf("Segmentation complete. Results saved to output/\n");

    /// ----------------------------------------------
    /// MATCHING
    /// ----------------------------------------------

    // Get handshape files from the handshapes folder
    std::vector<std::string> handshape_files = Anim::get_all_handshape_files(match_config.handshape_dir);

    // Get all HandSkeletons from the handshape files
    std::vector<HandSkeleton> handshape_handskeletons;
    for (const std::string& handshape_file : handshape_files) {
        HandSkeletonInfo handskeleton_info = Handskel::get_handskeletoninfo_from_file(
            handshape_file, {}, match_config.bone_mapping);
        
        handshape_handskeletons.push_back(handskeleton_info.handskeleton);
    }

    // Match
    int segment_index = 0;  // Example segment, take 0 for demo
    std::vector<Match> matches = Handskel::match_handskeletons(
        handshape_handskeletons, segments[segment_index].hand_skeleton, match_config.matching_method);

    printf("Matches for segment %d:\n", segment_index);
    int k = 10, i = 0;
    for (const Match& match : matches) {
        printf("Handshape ID: %d, File: %s \t Distance: %f\n",
            match.handshape_id, handshape_files[match.handshape_id].c_str(), match.distance);
        if (++i >= k) break;  // Limit to top k matches
    }

    /// ----------------------------------------------
    /// DISTANCE MATRIX
    /// ----------------------------------------------

    // Get all fbx files from the fbx folder
    std::vector<std::string> fbx_files;
    std::string fbx_dir = "fbx";
    for (const auto& entry : std::filesystem::directory_iterator(fbx_dir)) {
        if (entry.is_regular_file() &&
        (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            fbx_files.push_back(entry.path().string());
        }
    }

    // Create distance matrix from handshape files and fbx files
    Handskel::create_distance_matrix_from_handshapefiles(
        handshape_files, fbx_files, {}, match_config.bone_mapping,
        match_config.matching_method, "output/handshape_distance_matrix_handskel.csv");

    printf("Distance matrix created. Results saved to output/\n");

    return 0;
}