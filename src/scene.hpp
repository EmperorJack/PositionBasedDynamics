//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SCENE_HPP
#define POSITIONBASEDDYNAMICS_SCENE_HPP

#include <GL/glew.h>
#include <camera.hpp>
#include <mesh.hpp>

class Scene {

public:
    Scene();
    ~Scene();

    void render(bool wireframe);

    // Camera
    Camera* camera;
    float pitch, yaw, roll;

    // Objects
    vector<Mesh*> staticObjects;
    vector<Mesh*> simulatedObjects;

};

#endif //POSITIONBASEDDYNAMICS_SCENE_HPP
