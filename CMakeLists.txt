cmake_minimum_required(VERSION 3.18)
project(MuJoCoTrainer LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CUDA
enable_language(CUDA)

# Include MuJoCo and other dependencies
set(MUJOCO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/mujoco/include")
set(MUJOCO_LIB_DIR "${CMAKE_SOURCE_DIR}/mujoco/lib")
include_directories(${MUJOCO_INCLUDE_DIR})
link_directories(${MUJOCO_LIB_DIR})

# Add source files
set(SOURCES
    src/main.cpp
    src/controller.cpp
    src/evolution.cpp
    src/kernel.cu
)

# Add header files
include_directories(include)

# Add executable
add_executable(MuJoCoTrainer ${SOURCES})

# Link libraries
target_link_libraries(MuJoCoTrainer mujoco)

# CUDA flags
set_target_properties(MuJoCoTrainer PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "75"
)

# Output directories
set_target_properties(MuJoCoTrainer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Additional configurations
if (MSVC)
    target_compile_options(MuJoCoTrainer PRIVATE /W4 /WX)
else()
    target_compile_options(MuJoCoTrainer PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()