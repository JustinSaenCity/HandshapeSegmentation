#pragma once

#include <vector>
#include <tuple>
#include "math/math.h"

namespace handseg
{
    class Handseg
    {
    public:
        static std::vector<Segment> get_segments(char** argv, bool save_to_csv = false, bool plot = false);
        static std::vector<std::tuple<int, int>> get_segments_as_tuples(char** argv);
    };
}