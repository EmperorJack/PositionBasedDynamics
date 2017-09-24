//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {

    // Setup camera
    camera = new Camera();

    // Setup objects
    Vector4f meshColour = { 0.15f, 0.45f, 0.8f, 1.0f };
    mesh = new Mesh("../resources/objects/cube.obj", meshColour);

    Vector4f planeColour = { 1.0f, 1.0f, 1.0f, 1.0f };
    plane = new Mesh("../resources/objects/plane.obj", planeColour);
    plane->position = Vector3f(0, -1, 0);
}

Simulation::~Simulation() {
    delete camera;
    delete mesh;
    delete plane;
}

void Simulation::update() {

}

void Simulation::render() {
    camera->setPerspective(45.0f, (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT, 0.1f, 100.0f);
    camera->lookAt(Vector3f(0, 0, 10), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    mesh->render(camera, modelMatrix);
    plane->render(camera, modelMatrix);
}
