//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SCENE_HPP
#define POSITIONBASEDDYNAMICS_SCENE_HPP

#include <GL/glew.h>
#include <camera.hpp>
#include <constraint.hpp>
#include <mesh.hpp>

class Constraint;
class CollisionConstraint;

struct Configuration {
    vector<Mesh*> staticObjects;
    vector<Mesh*> simulatedObjects;
    vector<Vector3f> estimatePositions;
    vector<float> inverseMasses;
    vector<Constraint*> constraints;
    vector<CollisionConstraint*> collisionConstraints;

    ~Configuration() {
        for (Mesh* mesh : staticObjects) delete mesh;
        for (Mesh* mesh : simulatedObjects) delete mesh;
        for (Constraint* constraint : constraints) delete constraint;
        for (CollisionConstraint* constraint : collisionConstraints) delete constraint;
    }
};

class Scene {

public:
    Scene();
    ~Scene();

    void reset();
    void setConfiguration(int index);
    void translateInteraction(Vector3f translate);
    void render(bool wireframe);

    // Camera
    Camera* camera;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float roll = 0.0f;

    // Objects
    Configuration* currentConfiguration;

private:
    void setupConfigurationA();
    void setupConfigurationB();
    void setupConfigurationC();
    void addPlaneToConfiguration(Configuration* configuration);
    void setupEstimatePositionOffsets(Configuration* configuration);
    void buildTwoWayCouplingConstraints(Configuration* configuration);

    Configuration* configurationA;
    Configuration* configurationB;
    Configuration* configurationC;

};

#endif //POSITIONBASEDDYNAMICS_SCENE_HPP
