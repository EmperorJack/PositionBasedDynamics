cmake_minimum_required(VERSION 3.7)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME)
project(PositionBasedDynamics)

# Include OpenGL
find_package(OpenGL REQUIRED)

# Include common files
include_directories(${PROJECT_SOURCE_DIR}/common)
file(GLOB COMMON
    common/*.hpp
    common/*.cpp
)

# Include source files
include_directories(${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SOURCES
    src/*.h
    src/*.cpp
)

# Include and make glew subproject
include_directories(${PROJECT_SOURCE_DIR}/lib/glew/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/glew)

# Include and make glfw subproject
include_directories(${PROJECT_SOURCE_DIR}/lib/glfw/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/glfw)

# Incldue the imgui library
include_directories(${PROJECT_SOURCE_DIR}/lib/imgui)

# Include any external library files
file(GLOB EXTERNAL
    lib/glm/glm/glm.hpp
    lib/kissfft/*.h
    lib/kissfft/*.c
    lib/imgui/*.h
    lib/imgui/*.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${COMMON} ${EXTERNAL})

# Link the external libraries
target_link_libraries(${PROJECT_NAME} glfw glew ${OPENGL_LIBRARIES})

# Copy resources to the build directory
set(RESOURCES_SUBDIRECTORY resources)
file(COPY ${CMAKE_SOURCE_DIR}/${RESOURCES_SUBDIRECTORY} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
