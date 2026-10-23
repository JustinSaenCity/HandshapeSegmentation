#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <string>
#include <vector>

#include "math/math.h"
#include "anim/anim.h"
#include "config/config.h"
#include "matching/poseshape.h"
#include "segmentation.h"

using namespace math;
using namespace anim;
using namespace segmentation;
using namespace poseshape;

struct PoseInfo {
    std::string name;
    Pose handshape;
};

void create_distance_matrix_from_handshapes(const std::vector<PoseInfo>& handshapes, std::string output_file)
{
    int N = handshapes.size();
    std::vector<std::vector<double>> distance_matrix(N, std::vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double distance = Poseshape::pose_distance(handshapes[i].handshape, handshapes[j].handshape);
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

    // Get the quaternion tracks from the FBX file
    double fps;
    std::vector<JointTrack> tracks;
    Segmentation::get_quaternion_tracks_from_fbx(argv[1], {}, tracks, fps);
    printf("Loaded %zu quaternion tracks from %s at %.2f fps\n", tracks.size(), argv[1], fps);

    // Get the segments from the quaternion tracks
    std::vector<SegmentData> segments = Segmentation::get_segments(tracks, fps, "config.json", true);

    // std::vector<Segment> segs = std::get<1>(Handseg::get_segments(argv[1], true));
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
    std::vector<std::string> handshape_files = Poseshape::get_all_handshape_files();
    fbx_files.insert(fbx_files.end(), handshape_files.begin(), handshape_files.end());

    // Get the (right) handshape data for each file
    std::vector<PoseInfo> handshapes_for_clustering;
    for (const auto& file : fbx_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Segmentation::get_quaternion_tracks_from_fbx(file, config.segmentation_bones, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Process the tracks
        for (JointTrack& quaternion_track : handshape_tracks) { 
            Math::align_quaternion_signs(quaternion_track.q);
            Math::smooth_quaternions(quaternion_track.q, 2.0);
        }

        std::vector<SegmentData> segs = Segmentation::get_segments(handshape_tracks, handshape_fps, "config.json", false);

        for (int i = 0; i < segs.size(); ++i) {
            const SegmentData& segment = segs[i];
            PoseInfo average_handshape;
            average_handshape.name = file + "_segment_" + std::to_string(i);
            average_handshape.handshape = segment.average_pose;
            handshapes_for_clustering.push_back(average_handshape);
        }
    }

    create_distance_matrix_from_handshapes(handshapes_for_clustering, "output/handshape_distance_matrix.csv");

    std::vector<Pose> poses_for_matching;
    for (const PoseInfo& info : handshapes_for_clustering) {
        poses_for_matching.push_back(info.handshape);
    }
    std::vector<Match> matches = Poseshape::match_poses(poses_for_matching, handshapes_for_clustering[4].handshape);

    for (int i = 0; i < std::min(10, static_cast<int>(matches.size())); ++i) {
        std::printf("Handshape: %s\n", handshapes_for_clustering[i].name.c_str());
    }

    // Print names of top 10 matches
    std::printf("Top 10 matches for %s:\n", handshapes_for_clustering[4].name.c_str());
    for (int i = 0; i < std::min(10, static_cast<int>(matches.size())); ++i) {
        const Match& match = matches[i];
        std::printf("%d: %s (distance: %.4f)\n", i + 1, handshapes_for_clustering[match.handshape_id].name.c_str(), match.distance);
    }

    return 0;
}