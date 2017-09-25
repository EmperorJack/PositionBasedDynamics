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

    Constraint buildFixedConstraint(int index);
    Constraint buildDistanceConstraint(int indexA, int indexB, float distance);

    void update();
    void simulate(Mesh* mesh);
    void render();
    void renderGUI();

    // Variables
    int solverIterations = 10;
    float timeStep = 0.1f;
    float gravity = 0.981f;
    float velocityDamping = 1.0f;

    // Camera variables
    float pitch, yaw, roll;

private:

    // Camera
    Camera* camera;

    // Objects
    Mesh* cubeMesh;
    Mesh* planeMesh;
    Mesh* flagPoleMesh;
    Mesh* flagMesh;

    // Solver
    SparseLU<SparseMatrix<float>> solver;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
