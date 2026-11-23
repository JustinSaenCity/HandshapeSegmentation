#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <string>
#include <vector>

#include "lib/types.h"
#include <filesystem>
#include <fstream>

#include "lib/math/math.h"
#include "lib/anim/anim.h"
#include "lib/config/matching_config.h"
#include "matching/handskeleton.h"
#include "segmentation/segmentation.h"

using namespace math;
using namespace anim;
using namespace segmentation;
using namespace handskel;

std::vector<SegmentData> segment(std::string fbx_file, std::string segmentation_config_file, std::string matching_config_file)
{
    MatchingConfig match_config = MatchingConfig::load_from_file(matching_config_file);

    // Get the quaternion tracks from the FBX file
    double fps;
    std::vector<JointTrack> tracks;
    Anim::get_quaternion_tracks_from_fbx(fbx_file, {}, tracks, fps);

    // Get pre-match info, to set if we want to get the data for matching from the segmentation
    PreMatchInfo pre_match_info = PreMatchInfo();
    pre_match_info.do_pre = true;
    pre_match_info.bone_mapping = match_config.bone_mapping;
    pre_match_info.use_virtual_leaf_bones = match_config.use_virtual_leaf_bones;
    pre_match_info.leaf_bone_distance = match_config.leaf_bone_distance;
    pre_match_info.hand_location_tracks.clear();
    Anim::get_location_tracks_from_fbx(fbx_file, {}, pre_match_info.hand_location_tracks, fps);

    // Get the segments from the joint tracks
    std::vector<SegmentData> segments = Segmentation::get_segments(
        tracks, fps, segmentation_config_file, true, true, pre_match_info);

    printf("Segmentation complete. Results saved to output/\n");

    return segments;
}

void match(std::string matching_config_file, const std::vector<SegmentData>& segments, int segment_index, int k = 10)
{
    MatchingConfig match_config = MatchingConfig::load_from_file(matching_config_file);

    // Get handshape files from the handshapes folder
    std::vector<std::string> handshape_files = Anim::get_all_handshape_files(match_config.handshape_dir);

    // Get all HandSkeletons from the handshape files
    std::vector<HandSkeleton> handshape_handskeletons;
    for (const std::string& handshape_file : handshape_files) {
        HandSkeletonInfo handskeleton_info = Handskel::get_handskeletoninfo_from_file(
            handshape_file, {}, match_config.bone_mapping, match_config.use_virtual_leaf_bones, match_config.leaf_bone_distance);
        
        handshape_handskeletons.push_back(handskeleton_info.handskeleton);
    }

    // Match
    std::vector<Match> matches = Handskel::match_handskeletons(
        handshape_handskeletons, segments[segment_index].hand_skeleton, match_config.matching_method);

    printf("Matches for segment %d:\n", segment_index);
    int i = 0;
    for (const Match& match : matches) {
        printf("Handshape ID: %d, File: %s \t Distance: %f\n",
            match.handshape_id, handshape_files[match.handshape_id].c_str(), match.distance);
        if (++i >= k) break;  // Limit to top k matches
    }

    // Write matches to handshape_matches.csv
    std::filesystem::create_directory("output");
    std::ofstream csv_file("output/handshape_matches.csv");
    csv_file << "rank,handshape_file,distance\n";
    for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
        const Match& match = matches[i];
        csv_file << (i + 1) << "," << handshape_files[match.handshape_id] << "," << match.distance << "\n";
    }
    csv_file.close();

    printf("Matching complete. Results saved to output/handshape_matches.csv\n");
}

void dist_mat(std::string matching_config_file)
{
    MatchingConfig match_config = MatchingConfig::load_from_file(matching_config_file);

    // Get all fbx files from the fbx folder
    std::vector<std::string> fbx_files;
    std::string fbx_dir = "fbx";
    for (const auto& entry : std::filesystem::directory_iterator(fbx_dir)) {
        if (entry.is_regular_file() &&
        (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            fbx_files.push_back(entry.path().string());
        }
    }

    // Get handshape files from the handshapes folder
    std::vector<std::string> handshape_files = Anim::get_all_handshape_files(match_config.handshape_dir);

    // Create distance matrix from handshape files and fbx files
    Handskel::create_distance_matrix_from_handshapefiles(
        handshape_files, fbx_files, {}, match_config.bone_mapping,
        match_config.use_virtual_leaf_bones, match_config.leaf_bone_distance,
        match_config.matching_method, "output/handshape_distance_matrix_handskel.csv");

    printf("Distance matrix created. Results saved to output/\n");
}


int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Usage: .\\shapeseg.exe <option> {...}"); return 1; }

    switch (argv[1][0]) {
        case 's':  // Segment
            if (argc < 3) { std::puts("Usage: .\\shapeseg.exe s <file.fbx>"); return 1; }
            segment(argv[2], "segmentation_config.json", "matching_config.json");
            break;
        case 'm':  // Match
            if (argc < 4) { std::puts("Usage: .\\shapeseg.exe m <file.fbx> <segment_idx> {k}"); return 1; }
            {
                std::vector<SegmentData> segments = segment(argv[2], "segmentation_config.json", "matching_config.json");
                match("matching_config.json", segments, std::stoi(argv[3]), argc > 4 ? std::stoi(argv[4]) : 10);
            }
            break;
        case 'd':  // Distance matrix
            dist_mat("matching_config.json");
            break;
        default:
            std::puts("Unknown option. Use 's' for segmentation, 'm' for matching, or 'd' for distance matrix.");
            return 1;
    }

    return 0;
}
