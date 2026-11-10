#pragma once

#include "types.h"

namespace handshape
{
    class Handshape
    {
    public:
        static std::vector<double> compute_handshape_representation(const HandSkeleton& skeleton);
        static std::vector<double> compute_handshape_representation_from_pose(const Pose& pose);
    };
}