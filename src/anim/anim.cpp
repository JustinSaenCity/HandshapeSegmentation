#include <cstdio>

#include "anim.h"

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

int Anim::get_bones_from_scene(ufbx_scene* scene, const std::vector<std::string>& bone_filter, std::vector<ufbx_node*>* out_nodes, std::vector<JointTrack>* out_tracks, std::vector<JointLocationTrack>* out_location_tracks) {
    auto isInBoneFilter = [bone_filter](const std::string& n) {
        for (const std::string& key : bone_filter)
            if (n == key) return true;
        return false;
    };

    bool has_bone_filter = !bone_filter.empty();

    for (size_t i = 0; i < scene->nodes.count; ++i) {
        ufbx_node* node = scene->nodes.data[i];
        std::string name(node->name.data, node->name.length);
        if (!has_bone_filter || isInBoneFilter(name)) {
            out_nodes->push_back(node);
            out_tracks->push_back({name, {}});
            out_location_tracks->push_back({name, {}});
        }
    }
    return out_nodes->size();
}

void Anim::fill_joint_tracks(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, const double fps, std::vector<JointTrack>& out_tracks) {
    double t0 = anim->time_begin;
    double t1 = anim->time_end;
    double dt = 1.0 / fps;

    for (double t = t0; t <= t1; t += dt) {
        for (size_t j = 0; j < nodes.size(); ++j) {
            ufbx_transform tr = ufbx_evaluate_transform(anim, nodes[j], t);
            out_tracks[j].q.emplace_back(tr.rotation.w, tr.rotation.x,
                tr.rotation.y, tr.rotation.z);
        }
    }
}

// void Anim::fill_joint_locations(const ufbx_anim* anim, const std::vector<ufbx_node*>& nodes, const double fps, std::vector<JointLocationTrack>& out_tracks) {
//     double t0 = anim->time_begin;
//     double t1 = anim->time_end;
//     double dt = 1.0 / fps;

//     for (double t = t0; t <= t1; t += dt) {
//         for (size_t j = 0; j < nodes.size(); ++j) {
//             ufbx_transform tr = ufbx_evaluate_transform(anim, nodes[j], t);
//             Eigen::Vector3d v(tr.translation.x, tr.translation.y, tr.translation.z);
//             out_tracks[j].locations.emplace_back(v);
//         }
//     }
// }

void Anim::fill_joint_locations(
    const ufbx_scene* scene,
    const ufbx_anim* anim,
    const std::vector<ufbx_node*>& nodes,
    const double fps,
    std::vector<JointLocationTrack>& out_tracks)
{
    double t0 = anim->time_begin;
    double t1 = anim->time_end;
    double dt = 1.0 / fps;

    ufbx_error error;

    for (double t = t0; t <= t1; t += dt) {
        ufbx_scene* eval_scene = ufbx_evaluate_scene(scene, anim, t, nullptr, &error);
        if (!eval_scene) continue;

        for (size_t j = 0; j < nodes.size(); ++j) {
            ufbx_node* eval_node = eval_scene->nodes.data[nodes[j]->typed_id];

            ufbx_vec3 pos = ufbx_transform_position(&eval_node->node_to_world, ufbx_zero_vec3);

            Eigen::Vector3d v(pos.x, pos.y, pos.z);
            out_tracks[j].locations.emplace_back(v);
        }

        ufbx_free_scene(eval_scene);
    }
}

void Anim::get_quaternion_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointTrack>& out_tracks, double& out_fps)
{
    // Load the FBX file
    ufbx_error error;
    ufbx_scene* scene;
    if (Anim::load_fbx_scene(fbx_file.c_str(), &scene))
        return;

    // Find the finger bones
    std::vector<ufbx_node*> nodes;
    std::vector<JointTrack> tracks;
    std::vector<JointLocationTrack> _location_tracks;
    int num_fingers = Anim::get_bones_from_scene(scene, bone_filter, &nodes, &tracks, &_location_tracks);
    if (num_fingers == 0) {
        printf("No finger bones found in %s, please adjust the filter.\n", fbx_file.c_str());
        ufbx_free_scene(scene);
        return;
    }

    // Sample the joint tracks for each finger bone
    const ufbx_anim* anim = scene->anim;
    const double fps = scene->settings.frames_per_second;
    Anim::fill_joint_tracks(anim, nodes, fps, tracks);

    out_tracks = tracks;
    out_fps = scene->settings.frames_per_second;

    ufbx_free_scene(scene);
}

void Anim::get_location_tracks_from_fbx(const std::string& fbx_file, const std::vector<std::string>& bone_filter, std::vector<JointLocationTrack>& out_tracks, double& out_fps)
{
    // Load the FBX file
    ufbx_error error;
    ufbx_scene* scene;
    if (Anim::load_fbx_scene(fbx_file.c_str(), &scene))
        return;

    // Find the finger bones
    std::vector<ufbx_node*> nodes;
    std::vector<JointTrack> _tracks;
    std::vector<JointLocationTrack> location_tracks;
    int num_fingers = Anim::get_bones_from_scene(scene, bone_filter, &nodes, &_tracks, &location_tracks);
    if (num_fingers == 0) {
        printf("No finger bones found in %s, please adjust the filter.\n", fbx_file.c_str());
        ufbx_free_scene(scene);
        return;
    }

    // Sample the joint tracks for each finger bone
    const ufbx_anim* anim = scene->anim;
    const double fps = scene->settings.frames_per_second;
    Anim::fill_joint_locations(scene, anim, nodes, fps, location_tracks);

    out_tracks = location_tracks;
    out_fps = scene->settings.frames_per_second;

    ufbx_free_scene(scene);
}