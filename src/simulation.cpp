//
// Created by Jack Purvis
//

#include <string>
#include <Eigen>
#include <shaderLoader.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup VBOs
    float planeVertices[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
           - 1.0f, 1.0f, 0.0f,
    };

    glGenBuffers(1, &planeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
}

void Simulation::update() {

}

void Simulation::render() {
    float angleOfView = 90.0f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    float scale = (float) (1 / tan(angleOfView * 0.5 * M_PI / 180));

    Matrix4f projection;
    projection << scale, 0, 0, 0,
                  0, scale, 0, 0,
                  0, 0, -(zFar + zNear) / (zFar - zNear), -1,
                  0, 0, -2 * (zNear * zFar) / (zFar - zNear), 0;
    Affine3f view(Translation3f(0.0f, 0.0f, 0.0f));
    Matrix4f mvp = projection * view.matrix();

    glUseProgram(shader);

    float colour[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLint fillColorLocation = glGetUniformLocation(shader, "fillColor");
    glUniform4fv(fillColorLocation, 1, colour);

    GLint transformID = glGetUniformLocation(3, "MVP");
    glUniformMatrix4fv(transformID, 1, GL_FALSE, mvp.data());

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
