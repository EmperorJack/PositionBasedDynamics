//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <imgui.h>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {

    // Setup camera
    camera = new Camera();

    // Setup objects
    Vector3f meshColour = { 0.15f, 0.45f, 0.8f };
    testCube = new Mesh("../resources/models/cube.obj", meshColour);
    testCube->gravityAffected = true;

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    plane = new Mesh("../resources/models/plane.obj", planeColour);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    flagPole = new Mesh("../resources/models/flagPole.obj", flagPoleColour);
    flagPole2 = new Mesh("../resources/models/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    flag = new Mesh("../resources/models/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    flagHigh = new Mesh("../resources/models/flagHigh.obj", flagColour);
    flagHigh->gravityAffected = true;
    flagHigh->windAffected = true;

    // Setup constraints
    testCube->constraints.push_back(buildFixedConstraint(testCube, 3, testCube->initialVertices[3]));
    buildEdgeConstraints(testCube, 1.0f);

    for (int i = 0; i < 7; i++) {
        flag->constraints.push_back(buildFixedConstraint(flag, i, flag->initialVertices[i]));
    }
    buildEdgeConstraints(flag, 0.95f);
    buildBendConstraints(flag, 0.5f);

    for (int i = 0; i < 14; i++) {
        flagHigh->constraints.push_back(buildFixedConstraint(flagHigh, i, flagHigh->initialVertices[i]));
    }
    buildEdgeConstraints(flagHigh, 0.95f);
    buildBendConstraints(flagHigh, 0.5f);
}

Simulation::~Simulation() {
    delete camera;
    delete testCube;
    delete plane;
    delete flagPole;
    delete flag;
    delete flagPole2;
    delete flagHigh;
}

void Simulation::reset() {
    testCube->reset();
    flag->reset();
    flagHigh->reset();
}

void Simulation::update() {
    simulate(testCube);
    simulate(flag);
    simulate(flagHigh);
}

void Simulation::simulate(Mesh* mesh) {

    // Apply external forces
    for (int i = 0; i < mesh->numVertices; i++) {
        if (mesh->gravityAffected) mesh->velocities[i] += timeStep * Vector3f(0, -gravity, 0);
        if (mesh->windAffected) mesh->velocities[i] += timeStep * Vector3f(0, 0, -windSpeed);
    }

    // Dampen velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        // TODO
        mesh->velocities[i] *= velocityDamping;
    }

    mesh->estimatePositions.clear();
    mesh->estimatePositions.resize((size_t) mesh->numVertices, Vector3f::Zero());

    // Initialise estimate positions
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->estimatePositions[i] = mesh->vertices[i] + timeStep * mesh->velocities[i];
    }

    // Generate collision constraints
    // TODO

    // Project constraints iteratively
    for (int iteration = 0; iteration < solverIterations; iteration++) {
        for (Constraint* constraint : mesh->constraints) {
            constraint->project(solverIterations);
        }
    }

    // Update positions and velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] = (mesh->estimatePositions[i] - mesh->vertices[i]) / timeStep;
        mesh->vertices[i] = mesh->estimatePositions[i];
    }

    // Update velocities of colliding vertices
    // TODO
}

void Simulation::render() {
    camera->setPerspective(45.0f, (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT, 0.1f, 100.0f);
    camera->lookAt(Vector3f(0, 0, 20), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    testCube->render(camera, modelMatrix);
    plane->render(camera, modelMatrix);
    flagPole->render(camera, modelMatrix);
    flag->render(camera, modelMatrix);
    flagPole2->render(camera, modelMatrix);
    flagHigh->render(camera, modelMatrix);
}

void Simulation::renderGUI() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    ImGui::Begin("Simulator");

    ImGui::Text("Solver Iterations");
    ImGui::SliderInt("##solverIterations", &solverIterations, 1, 50, "%.0f");

    ImGui::Text("Timestep");
    ImGui::SliderFloat("##timeStep", &timeStep, 0.001f, 0.1f, "%.3f");

    ImGui::Text("Gravity");
    ImGui::SliderFloat("##gravity", &gravity, 0.01f, 10.0f, "%.2f");

    ImGui::Text("WindSpeed");
    ImGui::SliderFloat("##windSpeed", &windSpeed, 0.01f, 10.0f, "%.2f");

    ImGui::Text("Velocity Damping");
    ImGui::SliderFloat("##velocityDamping", &velocityDamping, 0.5f, 1.0f, "%.3f");

    ImGui::Text("Wireframe");
    ImGui::Checkbox("##wireframe", &wireframe);

    ImGui::End();
}