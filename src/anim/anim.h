#pragma once

#include <Eigen/Geometry>

struct JointTrack {
    std::string name;
    std::vector<Eigen::Quaterniond> q;   // lokale rotatie per frame
};

namespace anim
{
    class Anim
    {
    public:
        static bool load_fbx_scene(const char* filename, ufbx_scene** out_scene);
        static int get_finger_bones_from_scene(ufbx_scene* scene, std::vector<ufbx_node*>* out_nodes, std::vector<JointTrack>* out_tracks);
        static void fill_joint_tracks(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, std::vector<JointTrack>& out_tracks);
    };
}