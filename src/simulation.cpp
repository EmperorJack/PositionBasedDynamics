//
// Created by Jack Purvis
//

#include <iostream>
#include <string>
#include <Eigen>
#include <main.hpp>
#include <shaderLoader.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup VBOs
    float planeVertices[] = {
            -1, -1, 0,
            1, -1, 0,
            1, 1, 0,
            -1, 1, 0,
    };

    glGenBuffers(1, &planeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);

    // Setup camera
    camera = new Camera();
}

Simulation::~Simulation() {
    delete camera;
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

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glVertexAttribPointer(
            0,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableVertexAttribArray(0);
}
