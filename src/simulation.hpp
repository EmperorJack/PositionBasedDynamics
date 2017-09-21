//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>

class Simulation {

public:
    Simulation();
    void update();
    void render();

private:

    // Shaders
    GLuint shader;

    // VBOs
    GLuint planeVBO;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
