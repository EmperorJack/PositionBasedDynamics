cmake_minimum_required(VERSION 2.8)

project(GLEW C)

SET(headers
	"include/GL/glew.h"
	"include/GL/glxew.h"
	"include/GL/wglew.h"
)

SET(sources
	"src/glew.c"
)

# Setup library target
add_library(glew ${headers} ${sources})
set_property(TARGET glew PROPERTY FOLDER "GLEW")

# Include the include directory
target_include_directories(glew PUBLIC "${PROJECT_SOURCE_DIR}/include")

# Link libaries
target_link_libraries(glew PRIVATE ${OPENGL_LIBRARIES})

# System-specific library usage requirements
if(WIN32)
	target_link_libraries(glew PRIVATE kernel32)
endif()

# Usage requirements as shared/static lib
if(BUILD_SHARED_LIBS)
	# Define macro for building DLL
	target_compile_definitions(glew PRIVATE GLEW_BUILD)
else()
	# Define macro for static linking
	target_compile_definitions(glew PUBLIC GLEW_STATIC)
endif()


