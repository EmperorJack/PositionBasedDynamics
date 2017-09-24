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

    float pitch, yaw, roll;

private:

    // Camera
    Camera* camera;

    // Objects
    Mesh* mesh;
    Mesh* plane;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
