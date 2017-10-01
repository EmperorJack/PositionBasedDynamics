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
    Mesh* testCube = new Mesh("../resources/models/cube.obj", meshColour);
    testCube->gravityAffected = true;

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    Mesh* plane = new Mesh("../resources/models/plane.obj", planeColour);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    Mesh* flagPole = new Mesh("../resources/models/flagPole.obj", flagPoleColour);
    Mesh* flagPole2 = new Mesh("../resources/models/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    Mesh* flag = new Mesh("../resources/models/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    Mesh* flagHigh = new Mesh("../resources/models/flagHigh.obj", flagColour);
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

    // Build object lists
    staticObjects.push_back(plane);
    staticObjects.push_back(flagPole);
    staticObjects.push_back(flagPole2);

    simulatedObjects.push_back(testCube);
    simulatedObjects.push_back(flag);
    simulatedObjects.push_back(flagHigh);

    //omp_set_num_threads(8);
}

Simulation::~Simulation() {
    delete camera;

    for (Mesh* mesh : staticObjects) {
        delete mesh;
    }

    for (Mesh* mesh : simulatedObjects) {
        delete mesh;
    }
}

void Simulation::reset() {
    for (Mesh* mesh : simulatedObjects) {
        mesh->reset();
    }
}

void Simulation::update() {
    //#pragma omp parallel for
    //for (int i = 0; i < simulatedObjects.size(); i++) {
    //    simulate(simulatedObjects[i]);
    //}
    for (Mesh* mesh : simulatedObjects) {
        simulate(mesh);
    }
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
        //#pragma omp parallel for
        //for (int c = 0; c < mesh->constraints.size(); c++) {
        //    mesh->constraints[c]->project(solverIterations);
        //}
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

    for (Mesh* mesh : staticObjects) {
        mesh->render(camera, modelMatrix);
    }

    for (Mesh* mesh : simulatedObjects) {
        mesh->render(camera, modelMatrix);
    }
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