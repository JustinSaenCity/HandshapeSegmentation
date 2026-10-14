#include <ufbx.h>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>

#include <cstdio>
#include <string>
#include <vector>

#include "math/math.h"
#include "anim/anim.h"
#include "config/config.h"
#include "handseg.h"

using namespace math;
using namespace anim;
using namespace handseg;

int main(int argc, char** argv)
{
    if (argc < 2) { std::puts("Usage: .\\handseg.exe <file.fbx>"); return 1; }

    auto segs = Handseg::get_segments(argv, true, true);

    printf("Segmentation complete. Results saved to output/handshape_segments.csv and output/acceleration_plot.png\n");
    return 0;
}