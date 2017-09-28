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

    void reset();

    void buildEdgeConstraints(Mesh* mesh, float stiffness);
    Constraint buildFixedConstraint(Mesh* mesh, int index, Vector3f target);
    Constraint buildDistanceConstraint(int indexA, int indexB, float distance, float stiffness);

    void update();
    void simulate(Mesh* mesh);
    void render();
    void renderGUI();

    // Constants
    const float EPSILON = 0.000001f;

    // Variables
    int solverIterations = 10;
    float timeStep = 0.05f;
    float gravity = 0.981f;
    float windSpeed = 1.0f;
    float velocityDamping = 1.0f;

    // Camera variables
    float pitch, yaw, roll;

private:

    // Camera
    Camera* camera;

    // Objects
    Mesh* testCube;
    Mesh* plane;
    Mesh* flagPole;
    Mesh* flag;
    Mesh* flagPole2;
    Mesh* flagHigh;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
