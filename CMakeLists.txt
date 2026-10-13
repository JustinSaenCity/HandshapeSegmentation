cmake_minimum_required(VERSION 3.20)
project(handseg LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 CONFIG REQUIRED)

find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h" REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# Fetch ufbx from GitHub and build it as a static library
include(FetchContent)
FetchContent_Declare(ufbx
  GIT_REPOSITORY https://github.com/ufbx/ufbx.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  does_not_exist)
FetchContent_MakeAvailable(ufbx)

add_library(ufbx_lib STATIC ${ufbx_SOURCE_DIR}/ufbx.c)
target_include_directories(ufbx_lib PUBLIC ${ufbx_SOURCE_DIR})

# Add the executable target and link libraries
add_executable(handseg
              src/main.cpp
              src/anim/anim.cpp
              src/math/math.cpp)

if(MSVC)
    target_compile_definitions(handseg PRIVATE WITHOUT_NUMPY)
endif()

target_link_libraries(handseg PRIVATE Eigen3::Eigen
                      ufbx_lib
                      Python3::Python
                      Python3::NumPy)

target_include_directories(handseg PRIVATE
                          src
                          ${MATPLOTLIB_CPP_INCLUDE_DIRS})
