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

struct Configuration {
    vector<Mesh*> staticObjects;
    vector<Mesh*> simulatedObjects;
    vector<Vector3f> estimatePositions;
    vector<Constraint*> constraints;

    ~Configuration() {
        for (Mesh* mesh : staticObjects) delete mesh;
        for (Mesh* mesh : simulatedObjects) delete mesh;
        for (Constraint* constraint : constraints) delete constraint;
    }
};

class Scene {

public:
    Scene();
    ~Scene();

    void reset();
    void setConfiguration(int index);
    void render(bool wireframe);

    // Camera
    Camera* camera;
    float pitch, yaw, roll;

    // Objects
    Configuration* configuration;

private:
    void setupConfigurationA();
    void setupConfigurationB();
    void setupConfigurationC();

    Configuration* configurationA;
    Configuration* configurationB;
    Configuration* configurationC;

};

#endif //POSITIONBASEDDYNAMICS_SCENE_HPP
