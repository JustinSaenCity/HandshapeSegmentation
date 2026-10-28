#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <ufbx.h>

#include "types.h"

namespace anim
{
    class Anim
    {
    public:
        static bool load_fbx_scene(const char* filename, ufbx_scene** out_scene);
        static int get_bones_from_scene(ufbx_scene* scene, const std::vector<std::string>& bone_filter, std::vector<ufbx_node*>* out_nodes, std::vector<JointTrack>* out_tracks);
        static void fill_joint_tracks(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, const double fps, std::vector<JointTrack>& out_tracks);
    };
}