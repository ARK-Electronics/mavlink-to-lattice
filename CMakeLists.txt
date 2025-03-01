cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavlink-to-lattice VERSION 0.9 LANGUAGES CXX)

add_compile_options(-Wall -Wunused)

# Download the SDK from github and add it as part of the project
include(FetchContent)
FetchContent_Declare(
    lattice-sdk-cpp
    GIT_REPOSITORY https://github.com/anduril/lattice-sdk-cpp.git
    GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(lattice-sdk-cpp)

# Assumes MAVSDK system wide install
list(APPEND CMAKE_PREFIX_PATH "/usr/local/MAVSDK/install")
find_package(MAVSDK REQUIRED)

include_directories(third_party/tomlplusplus/)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/MavlinkToLattice.cpp)

target_link_libraries(${PROJECT_NAME}
    pthread
    lattice-sdk-cpp
    MAVSDK::mavsdk)
