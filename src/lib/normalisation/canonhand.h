#pragma once
 
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <optional>
 
#include "lib/types.h"
 
namespace hand_canon {
 
// Joint in order of the paper, kStart is the index of each metacarpal and kCount the number of data points per finger.
constexpr std::array<int, 5> kStart = {0, 4, 9, 14, 19};
constexpr std::array<int, 5> kCount = {4, 5, 5, 5, 5};
 
// Struct is made of 24 Vector3d's, so we can also read it as an array.
// NOTE: This only works as long as the order of the members in HandSkeleton remains the same, which it should.
inline const Eigen::Vector3d* joints(const HandSkeleton& s) { return &s.thumb_metacarpal; }  // Return the first joint as a pointer, allowing access as an array.
inline Eigen::Vector3d* joints(HandSkeleton& s) { return &s.thumb_metacarpal; }
 
// Keeps the bone directions of `skel`, but uses thepositions and bone lengths of `ref`.
// Returns nullopt if any NaN's or a bone with length 0 is encountered.
inline std::optional<HandSkeleton> canonicalise(const HandSkeleton& skel, const HandSkeleton& ref) {
    const Eigen::Vector3d* in = joints(skel);
    const Eigen::Vector3d* r = joints(ref);
    HandSkeleton out;
    Eigen::Vector3d* o = joints(out);
 
    for (int f = 0; f < 5; ++f) {
        const int st = kStart[f];
        if (!r[st].allFinite()) return std::nullopt;

        o[st] = r[st];  // use reference base position for the metacarpal
        for (int b = 0; b < kCount[f] - 1; ++b) {
            const Eigen::Vector3d d = in[st + b + 1] - in[st + b];  // bone direction
            const double dn = d.norm();
            const double r_len = (r[st + b + 1] - r[st + b]).norm();
            if (!d.allFinite() || !(dn > 1e-9) || !std::isfinite(r_len)) return std::nullopt;

            o[st + b + 1] = o[st + b] + r_len * (d / dn);
        }
    }
    return out;
}
 
}