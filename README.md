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

TODO

## Usage

```bash
.\build\Release\handseg.exe <path_to_fbx_file>
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
