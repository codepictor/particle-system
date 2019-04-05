cmake_minimum_required(VERSION 3.8)

project(particle-system VERSION 0.1 DESCRIPTION "simple prototype of a particle system")

# Use C++11 standard for compiling
set(CMAKE_CXX_STANDARD 11)

# Choose dynamic or static linking of SFML
option(SFML_STATIC_LIBRARIES "Build the executable with static linking" OFF)

# Find SFML library
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Bring source files in src/ directory into the project
set(SOURCE_FILES src/main.cpp src/game.cpp src/particle_system.cpp src/utils.cpp src/window.cpp)

# Bring header files in include/ directory into the project
set(HEADER_FILES include/game.h include/particle_system.h include/utils.h include/window.h)

# Add all sources to target
add_executable(particle-system ${SOURCE_FILES} ${HEADER_FILES})

# Add paths to includes
target_include_directories(particle-system PUBLIC include)

# Linking
target_link_libraries(particle-system sfml-graphics sfml-window sfml-system)
