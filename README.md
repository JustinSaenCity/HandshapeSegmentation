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

### Segmentation

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
    "min_acceleration_threshold": 0.5,
    "smoothing_sigma": 1.5,
    "threshold_factor": 0.5
}
```

### Matching

1. handshape_dir - The directory containing the handshape FBX files.
2. matching_method - The method used to match the handshapes to the input hand skeleton. Options are: "Euclidean", "Cosine", "Manhattan".
3. bones - A mapping of the bone names in the handshape FBX files to the bone names in the input hand skeleton.

Example `matching_config.json`:
```json
{
    "handshape_dir": "../HandformSession/HandformsTrimmed",
    "matching_method": "Euclidean",
    "bones": {
        "thumb":  { "metacarpal": "RightHandThumb1", "proximal": "RightHandThumb2",
                    "distal": "RightHandThumb3", "tip": "RightHandThumb4" },
        "index":  { "metacarpal": "RightHandIndex", "proximal": "RightHandIndex1",
                    "middle": "RightHandIndex2", "distal": "RightHandIndex3",
                    "tip": "RightHandIndex4" },
        "middle": { "metacarpal": "RightHand", "proximal": "RightHandMiddle1",
                    "middle": "RightHandMiddle2", "distal": "RightHandMiddle3",
                    "tip": "RightHandMiddle4" },
        "ring":   { "metacarpal": "RightHandRing", "proximal": "RightHandRing1",
                    "middle": "RightHandRing2", "distal": "RightHandRing3",
                    "tip": "RightHandRing4" },
        "pinky":  { "metacarpal": "RightHandPinky", "proximal": "RightHandPinky1",
                    "middle": "RightHandPinky2", "distal": "RightHandPinky3",
                    "tip": "RightHandPinky4" }
    }
}
```

## Usage

```bash
.\build\Release\shapeseg.exe <path_to_fbx_file>
```

or get segmentation data through the C++ function in `src/segmentation/segmentation.h`:

```cpp
#include "segmentation/segmentation.h"

int main() {
   double fps;
   std::vector<JointTrack> tracks;
   Segmentation::get_quaternion_tracks_from_fbx("path/to/file.fbx", {}, tracks, fps);
   std::vector<SegmentData> segments = Segmentation::get_segments(tracks, fps, "path/to/segmentation_config.json");
}
```

and get the matches through the C++ function in `src/matching/handskeleton.h`:

```cpp
#include "matching/handskeleton.h"

// NOTE: This assumes SegmenData is filled through using the pre-match functionality of get_segments(), see main.cpp for an example.
int function(const SegmentData& segment_data, const std::vector<HandSkeleton>& handshape_handskeletons, const MatchingConfig& match_config) {
   HandSkeleton input = segment_data.handskeleton;
   std::vector<Match> matches = Handskel::match_handskeletons(handshape_handskeletons, input, match_config.matching_method);
}
```

## Troubleshooting

#### **The program doesn't build.**

Before running the build-clean script, manually delete the `build` folder.

#### **`vcpkg install` fails or CMake cannot find the toolchain file.**

Check that `VCPKG_ROOT` is set and restart your terminal (or IDE).
