#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <ufbx.h>

#include "lib/types.h"

namespace anim
{
    class Anim
    {
    public:
        static std::vector<std::string> get_all_handshape_files(std::string handshape_dir);
        static bool load_fbx_scene(const char* filename, ufbx_scene** out_scene);
        static int get_bones_from_scene(ufbx_scene* scene, const std::vector<std::string>& bone_filter, std::vector<ufbx_node*>* out_nodes, std::vector<JointTrack>* out_tracks, std::vector<JointLocationTrack>* out_location_tracks);
        static void fill_joint_tracks(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, const double fps, std::vector<JointTrack>& out_tracks);
        static void fill_joint_locations(const ufbx_scene* scene, const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, const double fps, std::vector<JointLocationTrack>& out_location_tracks);

        static void get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps);
        static void get_location_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointLocationTrack>& out_tracks, double& out_fps);
    };
}
