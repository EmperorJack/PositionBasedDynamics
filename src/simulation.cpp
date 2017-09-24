//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <main.hpp>
#include <shaderLoader.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup camera
    camera = new Camera();

    // Setup objects
    mesh = new Mesh("../resources/objects/cube.obj");
}

Simulation::~Simulation() {
    delete camera;
    delete mesh;
}

void Simulation::update() {

}

void Simulation::render() {
    camera->setPerspective(45.0f, (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT, 0.1f, 100.0f);
    camera->lookAt(Vector3f(0, 0, 5), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    AngleAxisf pitchAngle(mouseX / 360.0f, Vector3f::UnitX());
    AngleAxisf yawAngle(mouseY / 360.0f, Vector3f::UnitY());
    AngleAxisf rollAngle(0.0f, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    glUseProgram(shader);

    float colour[] = {0.15f, 0.45f, 0.8f, 1.0f};
    glUniform4fv(glGetUniformLocation(shader, "fillColor"), 1, colour);

    glUniformMatrix4fv(glGetUniformLocation(3, "projection"), 1, GL_FALSE, camera->projectionMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "view"), 1, GL_FALSE, camera->viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "model"), 1, GL_FALSE, modelMatrix.data());

    mesh->render();
}
