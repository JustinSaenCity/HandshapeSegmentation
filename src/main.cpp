#include <ufbx.h>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>

#include <cstdio>
#include <string>
#include <vector>
#include "tsne.h"

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

void cluster_and_plot_handshapes(const std::vector<HandInfo>& handshapes)
{
    int N = handshapes.size();
    int D = handshapes[0].handshape.joints.size() * 4; // Each quaternion has 4 components
    int target_dim = 2; // 2D plotting
    double perplexity = 1.0;
    double theta = 0.5;
    int max_iter = 1000;

    std::vector<double> data(N * D);
    for (int i = 0; i < N; ++i) {
        for (size_t j = 0; j < handshapes[i].handshape.joints.size(); ++j) {
            const Eigen::Quaterniond& q = handshapes[i].handshape.joints[j];
            data[i * D + j * 4 + 0] = q.w();
            data[i * D + j * 4 + 1] = q.x();
            data[i * D + j * 4 + 2] = q.y();
            data[i * D + j * 4 + 3] = q.z();
        }
    }

    std::vector<double> Y(N * target_dim, 0.0);

    // TSNE* tsne = new TSNE();
    printf("Running t-SNE on %d handshapes with %d dimensions...\n", N, D);
    TSNE::run(data.data(), N, D, Y.data(), target_dim, perplexity, theta, 42, false, max_iter, 250, 250);
    printf("t-SNE completed.\n");

    std::vector<double> x_group1, y_group1, x_group2, y_group2;
    for (int i = 0; i < N; ++i) {
        double x = Y[i * target_dim + 0];
        double y = Y[i * target_dim + 1];

        if (i < N / 2) {
            x_group1.push_back(x);
            y_group1.push_back(y);
        } else {
            x_group2.push_back(x);
            y_group2.push_back(y);
        }
    }

    matplotlibcpp::figure_size(800, 600);
    matplotlibcpp::scatter(x_group1, y_group1, 40.0, {{"color", "orange"}, {"label", "Groep A"}});
    matplotlibcpp::scatter(x_group2, y_group2, 40.0, {{"color", "purple"}, {"label", "Groep B"}});
    
    matplotlibcpp::title("BHTSNE Resultaat in C++");
    matplotlibcpp::xlabel("Component 1");
    matplotlibcpp::ylabel("Component 2");
    matplotlibcpp::legend();

    matplotlibcpp::show();
}

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

    std::vector<Segment> segs = std::get<1>(Handseg::get_segments(argv[1], true, true));

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
        if (tr.name.find("RightHand") != std::string::npos)
            right_hand_quaternion_tracks.push_back(tr);


    printf("1-------------------------------------------------------------\n");
    //---------------------------------------------------------------------------------------------
    // LOAD ALL HANSHAPE FILES AND EXTRACT THEIR QUATERNION TRACKS
    std::vector<std::string> handshape_files = Handshape::get_all_handshape_files();
    std::vector<Hand> handshapes_average_quaternion_tracks;

    for (const auto& file : handshape_files) {
        double handshape_fps;
        std::vector<JointTrack> handshape_tracks;
        Handseg::get_quaternion_tracks_from_fbx(file, config.handshape_bone_filter, handshape_tracks, handshape_fps);
        if (handshape_tracks.empty()) continue;

        // Filter right hand tracks and apply preprocessing
        std::vector<JointTrack> right_hand_tracks;
        for (const auto& tr : handshape_tracks)
            if (tr.name.find(config.handshape_right_hand_filter) != std::string::npos)
                right_hand_tracks.push_back(tr);

        // const Segment& segment = Segment(0, right_hand_tracks[0].q.size() - 1);
        const std::vector<Segment>& handshape_segs = std::get<1>(Handseg::get_segments(file));
        // handshapes_average_quaternion_tracks.push_back(Handshape::get_average_handshape_from_segment(right_hand_tracks, segment));
        for (const Segment& handshape_seg : handshape_segs) {
            Hand average_handshape = Handshape::get_average_handshape_from_segment(right_hand_tracks, handshape_seg);
            handshapes_average_quaternion_tracks.push_back(average_handshape);
        }
    }

    printf("2-------------------------------------------------------------\n");
    //---------------------------------------------------------------------------------------------
    // MATCH THE INPUT HANDSHAPE TO THE PREDEFINED HANSHAPES
    Hand average_input_handshape = Handshape::get_average_handshape_from_segment(right_hand_quaternion_tracks, segs[1]);
    printf("2.5-------------------------------------------------------------\n");
    std::vector<Match> matches = Handshape::match_handshapes(handshapes_average_quaternion_tracks, average_input_handshape);
    printf("2.5.5-------------------------------------------------------------\n");
    for (const Match& match : matches) {
        std::string handshape_name = handshape_files[match.handshape_id];
        double distance = match.distance;
        printf("Handshape: %s, Distance: %.4f\n", handshape_name.c_str(), distance);
    }

    printf("3-------------------------------------------------------------\n");
    // ---------------------------------------------------------------------------------------------
    // CLUSTER AND PLOT THE HANSHAPES
    // get all fbx's from the fbx folder
    std::vector<std::string> fbx_files;
    std::string fbx_dir = "fbx";

    for (const auto& entry : std::filesystem::directory_iterator(fbx_dir)) {
        if (entry.is_regular_file() &&
           (entry.path().extension().string() == ".fbx" || entry.path().extension().string() == ".FBX")) {
            fbx_files.push_back(entry.path().string());
        }
    }
    fbx_files.insert(fbx_files.end(), handshape_files.begin(), handshape_files.end());

    printf("4-------------------------------------------------------------\n");

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

        // Process the quaternion tracks
        for (JointTrack& quaternion_track : right_hand_quaternion_tracks) { 
            Math::align_quaternion_signs(quaternion_track.q);
            Math::smooth_quaternions(quaternion_track.q, 2.0);
        }

        std::vector<Segment> segs = std::get<1>(Handseg::get_segments(file));

        // for (const auto& segment : segs) {
        for (int i = 0; i < segs.size(); ++i) {
            const Segment& segment = segs[i];
            HandInfo average_handshape;
            average_handshape.name = file + "_segment_" + std::to_string(i);
            average_handshape.handshape = Handshape::get_average_handshape_from_segment(right_hand_tracks, segment);
            handshapes_for_clustering.push_back(average_handshape);
        }
    }

    printf("5-------------------------------------------------------------\n");

    cluster_and_plot_handshapes(handshapes_for_clustering);

    create_distance_matrix_from_handshapes(handshapes_for_clustering, "output/handshape_distance_matrix.csv");

    return 0;
}