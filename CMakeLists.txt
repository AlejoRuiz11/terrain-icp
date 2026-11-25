cmake_minimum_required(VERSION 3.25)

project(terrain_icp LANGUAGES CXX)

# C++20 exactly: no compiler extensions, no fallback to an older standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings as errors on our targets only (third-party headers are included as SYSTEM).
function(terrain_icp_set_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

# Third-party code: release archives pinned by SHA256, downloaded once at configure time.
include(FetchContent)

# Eigen and nanoflann are header-only. SOURCE_SUBDIR points at a directory without a CMakeLists.txt, so
# FetchContent only downloads them (their own CMake projects, tests and install rules are not needed).
FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz
  URL_HASH SHA256=b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR header-only
)
FetchContent_Declare(
  nanoflann
  URL https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.7.1.tar.gz
  URL_HASH SHA256=887e4e57e9c5fbf1c2937f9f5a9bc461c4786d54729b57a9c19547bdedb46986
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR header-only
)
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SYSTEM
)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
  URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SYSTEM
)
# Use the same (dynamic) C runtime as our targets on MSVC.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen nanoflann nlohmann_json googletest)

# Imported targets: their include directories are treated as system headers (no warnings from them).
add_library(terrain_icp_eigen INTERFACE IMPORTED)
target_include_directories(terrain_icp_eigen INTERFACE ${eigen_SOURCE_DIR})
add_library(terrain_icp_nanoflann INTERFACE IMPORTED)
target_include_directories(terrain_icp_nanoflann INTERFACE ${nanoflann_SOURCE_DIR}/include)

# Core library: PLY I/O, normals, ICP, results JSON.
add_library(terrain_icp_core STATIC
  src/terrain_icp/ply.cpp
  src/terrain_icp/icp.cpp
  src/terrain_icp/align_files.cpp)
target_include_directories(terrain_icp_core PUBLIC src)
target_link_libraries(terrain_icp_core PUBLIC terrain_icp_eigen PRIVATE terrain_icp_nanoflann nlohmann_json::nlohmann_json)
terrain_icp_set_warnings(terrain_icp_core)

add_executable(terrain-icp app/main.cpp)
target_link_libraries(terrain-icp PRIVATE terrain_icp_core)
terrain_icp_set_warnings(terrain-icp)

enable_testing()
include(GoogleTest)

add_executable(terrain_icp_tests tests/test_ply.cpp tests/test_icp.cpp tests/test_align_files.cpp)
target_link_libraries(terrain_icp_tests PRIVATE terrain_icp_core nlohmann_json::nlohmann_json GTest::gtest_main)
terrain_icp_set_warnings(terrain_icp_tests)
gtest_discover_tests(terrain_icp_tests DISCOVERY_TIMEOUT 60)
