#include <cstdio>

#include "anim.h"
#include "config/config.h"

using namespace anim;

bool Anim::load_fbx_scene(const char* filename, ufbx_scene** out_scene) {
    ufbx_load_opts opts = {};
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.target_unit_meters = 1.0f;

    ufbx_error error;
    *out_scene = ufbx_load_file(filename, &opts, &error);
    if (!(*out_scene)) {
        std::fprintf(stderr, "Error loading fbx file %s: %s\n", filename, error.description.data);
        return 1;
    }
    return 0;
}

int Anim::get_bones_from_scene(ufbx_scene* scene, const std::vector<std::string>& bone_filter, std::vector<ufbx_node*>* out_nodes, std::vector<JointTrack>* out_tracks) {
    auto isInBoneFilter = [bone_filter](const std::string& n) {
        for (const std::string& key : bone_filter)
            if (n.find(key) != std::string::npos) return true;
        return false;
    };

    for (size_t i = 0; i < scene->nodes.count; ++i) {
        ufbx_node* node = scene->nodes.data[i];
        std::string name(node->name.data, node->name.length);
        if (isInBoneFilter(name)) {
            out_nodes->push_back(node);
            out_tracks->push_back({name, {}});
        }
    }
    return out_nodes->size();
}

void Anim::fill_joint_tracks(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, std::vector<JointTrack>& out_tracks) {
    double t0 = anim->time_begin;
    double t1 = anim->time_end;
    double dt = 1.0 / 24.0;  // TODO: PAS AAN ZODAT DIT NIET GEHARDCODED IS

    for (double t = t0; t <= t1; t += dt) {
        for (size_t j = 0; j < nodes.size(); ++j) {
            ufbx_transform tr = ufbx_evaluate_transform(anim, nodes[j], t);
            out_tracks[j].q.emplace_back(tr.rotation.w, tr.rotation.x,
                                         tr.rotation.y, tr.rotation.z);
        }
    }
}