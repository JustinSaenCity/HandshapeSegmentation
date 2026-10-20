#include <ufbx.h>
#include <Eigen/Geometry>

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

struct HandInfo {
    std::string name;
    Hand handshape;
};

void create_distance_matrix_from_handshapes(const std::vector<HandInfo>& handshapes, std::string output_file)
{
    int N = handshapes.size();
    std::vector<std::vector<double>> distance_matrix(N, std::vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double distance = Handshape::pose_distance(handshapes[i].handshape, handshapes[j].handshape);
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

int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Usage: .\\handseg.exe <file.fbx>"); return 1; }

    std::vector<Segment> segs = std::get<1>(Handseg::get_segments(argv[1], true));
    printf("Segmentation complete. Results saved to output/\n");

    //---------------------------------------------------------------------------------------------
    Config config = Config::load_from_file("config.json");

    std::vector<std::string> fbx_files;
    std::string fbx_dir = "fbx";
    
    // Get all fbx files from the fbx folder
    for (const auto& entry : std::filesystem::directory_iterator(fbx_dir)) {
        if (entry.is_regular_file() &&
        (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            fbx_files.push_back(entry.path().string());
        }
    }
    // Add handshape files from the handshapes folder
    std::vector<std::string> handshape_files = Handshape::get_all_handshape_files();
    fbx_files.insert(fbx_files.end(), handshape_files.begin(), handshape_files.end());

    // Get the (right) handshape data for each file
    std::vector<HandInfo> handshapes_for_clustering;
    for (const auto& file : fbx_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Handseg::get_quaternion_tracks_from_fbx(file, config.bone_filter, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Filter right hand tracks
        std::vector<JointTrack> right_hand_tracks;
        for (const auto& tr : handshape_tracks)
            if (tr.name.find("RightHand") != std::string::npos)
                right_hand_tracks.push_back(tr);

        // Process the tracks
        for (JointTrack& quaternion_track : right_hand_tracks) { 
            Math::align_quaternion_signs(quaternion_track.q);
            Math::smooth_quaternions(quaternion_track.q, 2.0);
        }

        std::vector<Segment> segs = std::get<1>(Handseg::get_segments(file));

        for (int i = 0; i < segs.size(); ++i) {
            const Segment& segment = segs[i];
            HandInfo average_handshape;
            average_handshape.name = file + "_segment_" + std::to_string(i);
            average_handshape.handshape = Handshape::get_average_handshape_from_segment(right_hand_tracks, segment);
            handshapes_for_clustering.push_back(average_handshape);
        }
    }

    create_distance_matrix_from_handshapes(handshapes_for_clustering, "output/handshape_distance_matrix.csv");

    return 0;
}