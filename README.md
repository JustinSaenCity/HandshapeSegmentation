# HandshapeSegmentation

## Installation

1. Install [CMake](https://cmake.org/download/) and add it to your `PATH` environment variable.
2. Install [vcpkg](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell) and set the `VCPKG_ROOT` environment variable to its install directory.
3. (optional) Install [Python 3.12](https://www.python.org/downloads/) (other versions may work).
4. (optional) Install the required Python packages:

```bash
   py -m pip install numpy pandas scikit-learn plotly
```
5. Clone this repository
6. Build the project:

```bash
   .\scripts\build-clean.bat
```

## Configuration

1. bones - A list of bone names, these bones are used to detect segmentation.
2. min_segment_length - The minimal number of frames the detected segments will be.
3. min_acceleration_threshold - The absolute minimal value the peaks and valleys of the acceleration have to be before being detected in rad/s².
4. smoothing_sigma - The sigma used in the Gaussian smoothing of the quaternion values found in the animation.
5. threshold_factor - How many standard deviations a peak or valley has to be from 0 before being detected.

Example `segmentation_config.json`:
```json
{
    "bones": ["RightHandIndex",
              "RightHandIndex1",
              "RightHandIndex2",
              "RightHandIndex3",
              "RightHandThumb1",
              "RightHandThumb2",
              "RightHandThumb3",
              "RightHandMiddle1",
              "RightHandMiddle2",
              "RightHandMiddle3",
              "RightHandRing",
              "RightHandPinky",
              "RightHandPinky1",
              "RightHandPinky2",
              "RightHandPinky3",
              "RightHandRing1",
              "RightHandRing2",
              "RightHandRing3"],
    "min_segment_length": 2,
    "min_acceleration_threshold": 0.25,
    "smoothing_sigma": 1.5,
    "threshold_factor": 0.5
}
```

## Usage

```bash
.\build\Release\shapeseg.exe <path_to_fbx_file>
```

or get segmentation data through the C++ function in `src/segmentation.h`:

```cpp
#include "segmentation.h"

int main() {
   double fps;
   std::vector<JointTrack> tracks;
   Segmentation::get_quaternion_tracks_from_fbx("path/to/file.fbx", {}, tracks, fps);
   std::vector<SegmentData> segments = Segmentation::get_segments(tracks, fps, "path/to/config.json");
}
```

## Troubleshooting

#### **The program doesn't build.**

Before running the build-clean script, manually delete the `build` folder.

#### **`vcpkg install` fails or CMake cannot find the toolchain file.**

Check that `VCPKG_ROOT` is set and restart your terminal (or IDE).
