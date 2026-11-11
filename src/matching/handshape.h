#pragma once

#include "lib/types.h"

namespace handshape
{
    class Handshape
    {
    public:
        static std::vector<double> compute_handshape_representation(const HandSkeleton& skeleton);
    };
}