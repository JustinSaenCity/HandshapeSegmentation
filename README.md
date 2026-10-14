# HandshapeSegmentation

## Installation

1. Install [CMake](https://cmake.org/download/) and add it to your `PATH` environment variable.
2. Install [vcpkg](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell) and set the `VCPKG_ROOT` environment variable to its install directory.
3. Install [Python 3.12](https://www.python.org/downloads/) and add it to your `PATH` environment variable.
4. Install the required Python packages:

```bash
   py -m pip install matplotlib numpy
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

## Troubleshooting

#### **The program exits immediately without output.**

Windows cannot find `python312.dll`. Add the Python install directory to your `PATH` environment variable. You can check which Python version CMake found by running:

```bash
py -c "import sys; print(sys.executable)"
```

#### **`vcpkg install` fails or CMake cannot find the toolchain file.**

Check that `VCPKG_ROOT` is set and restart your terminal (or IDE).

#### **Plotting fails with a Python error.**

Make sure `matplotlib` and `numpy` are installed for the same Python version that CMake found.
