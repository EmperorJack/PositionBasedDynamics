//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>
#include <camera.hpp>
#include <mesh.hpp>

class Simulation {

    float COLLISION_THRESHOLD = 0.1f;

public:
    Simulation();
    ~Simulation();

    void reset();

    void update();
    void render();
    void renderGUI();

    // Variables
    int solverIterations = 6;
    float timeStep = 0.08f;
    float gravity = 0.981f;
    float windSpeed = 2.5f;
    float velocityDamping = 1.0f;
    float stretchFactor = 0.999f;
    float bendFactor = 0.3f;
    bool wireframe = false;

    // Camera variables
    float pitch, yaw, roll;

private:

    void simulate(Mesh* mesh);
    void generateCollisionConstraints(Mesh* mesh, int index, vector<CollisionConstraint*> &constraints);
    void updateCollisionVelocities(CollisionConstraint* constraint);

    // Camera
    Camera* camera;

    // Objects
    vector<Mesh*> staticObjects;
    vector<Mesh*> simulatedObjects;

    // Forces
    float windOscillation = 0.0f;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
