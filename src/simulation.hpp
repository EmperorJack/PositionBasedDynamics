//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>
#include <mesh.hpp>
#include <scene.hpp>
#include <constraint.hpp>

class Simulation {

    float COLLISION_THRESHOLD = 0.1f;

public:
    Simulation();
    ~Simulation();

    void reset();

    void update();
    void renderGUI();

    // Variables
    int solverIterations = 4;
    float timeStep = 0.03f;
    float gravity = 0.981f;
    float windSpeed = 1.5f;
    float velocityDamping = 0.999f;
    float stretchFactor = 0.999f;
    float bendFactor = 0.3f;
    bool wireframe = false;

    // Scene
    Scene* scene;

private:

    void simulate(Configuration *configuration);
    void generateCollisionConstraints(Configuration *configuration, Mesh *mesh, int index);
    bool planeIntersection(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal);
    void updateCollisionVelocities(CollisionConstraint* constraint);

    // Forces
    float windOscillation = 0.0f;

};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
