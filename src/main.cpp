#include <ufbx.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <string>
#include <vector>

#include "types.h"

#include "math/math.h"
#include "anim/anim.h"
#include "config/segmentation_config.h"
#include "config/matching_config.h"
#include "matching/poseshape.h"
#include "segmentation.h"

using namespace math;
using namespace anim;
using namespace segmentation;
using namespace poseshape;

void create_distance_matrix_from_handshapes(const std::vector<PoseInfo>& handshapes, const MatchingMethod method, std::string output_file)
{
    int N = handshapes.size();
    std::vector<std::vector<double>> distance_matrix(N, std::vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double distance = Poseshape::pose_distance(handshapes[i].handshape, handshapes[j].handshape, method);
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
    std::vector<SegmentData> segments = Segmentation::get_segments(tracks, fps, "./segmentation_config.json", true);

    // std::vector<Segment> segs = std::get<1>(Handseg::get_segments(argv[1], true));
    printf("Segmentation complete. Results saved to output/\n");

    //---------------------------------------------------------------------------------------------
    Config config = Config::load_from_file("segmentation_config.json");
    MatchingConfig match_config = MatchingConfig::load_from_file("matching_config.json");

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
    std::vector<std::string> handshape_files = Poseshape::get_all_handshape_files(match_config.handshape_dir);
    // fbx_files.insert(fbx_files.end(), handshape_files.begin(), handshape_files.end());

    // Get the (right) handshape data for each file
    std::vector<PoseInfo> handshapes_for_clustering;
    for (const auto& file : fbx_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Segmentation::get_quaternion_tracks_from_fbx(file, config.bones, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        std::vector<SegmentData> segs = Segmentation::get_segments(handshape_tracks, handshape_fps, "./segmentation_config.json");

        for (int i = 0; i < segs.size(); ++i) {
            const SegmentData& segment = segs[i];
            PoseInfo average_handshape;
            average_handshape.name = file + "_segment_" + std::to_string(i);
            average_handshape.handshape = segment.average_pose;
            handshapes_for_clustering.push_back(average_handshape);
        }
    }

    for (const auto& handshape_file : handshape_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Segmentation::get_quaternion_tracks_from_fbx(handshape_file, config.bones, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Process the tracks
        for (JointTrack& quaternion_track : handshape_tracks) { 
            Math::align_quaternion_signs(quaternion_track.q);
            Math::smooth_quaternions(quaternion_track.q, 2.0);
        }

        PoseInfo average_handshape;
        average_handshape.name = handshape_file;
        average_handshape.handshape = Poseshape::get_average_pose_from_segment(handshape_tracks, {0, static_cast<int>(handshape_tracks[0].q.size()) - 1}, false);
        handshapes_for_clustering.push_back(average_handshape);
    }

    create_distance_matrix_from_handshapes(handshapes_for_clustering, match_config.matching_method, "output/handshape_distance_matrix.csv");

    SegmentData segment_data = segments[0];
    std::vector<MatchInfo> matches = Poseshape::get_matches(segment_data, "segmentation_config.json", "matching_config.json");

    printf("Matches for segment from %d to %d:\n", segment_data.segment.start, segment_data.segment.end);
    for (const auto& match_info : matches) {
        printf("Handshape: %s, Distance: %.6f\n", match_info.handshape_name.c_str(), match_info.match.distance);
    }

    return 0;
}