#ifndef MAIN_HPP
#define MAIN_HPP

// Screen settings
#define FULL_SCREEN false
#define BORDERLESS false

#if FULL_SCREEN || BORDERLESS
#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080
#else
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720
#endif

// Constants
const float EPSILON = 0.000001f;

// Paths
static const char* SHADER_PATH = "resources/shaders/";

#endif
