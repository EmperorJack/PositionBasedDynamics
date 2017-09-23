//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>
#include <camera.hpp>
#include <mesh.hpp>

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

    // Objects
    Mesh* mesh;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
