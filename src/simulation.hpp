//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>
#include <camera.hpp>

class Simulation {

public:
    Simulation();
    ~Simulation();

    void update();
    void render();

private:

    // Shaders
    GLuint shader;

    // VBOs
    GLuint planeVBO;

    // Camera
    Camera* camera;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
