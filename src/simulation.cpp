//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

float timeStep = 0.05f;
Vector3f gravity(0, -0.981f, 0);
int solverIterations = 10;

Simulation::Simulation() {

    // Setup camera
    camera = new Camera();

    // Setup objects
    Vector4f meshColour = { 0.15f, 0.45f, 0.8f, 1.0f };
    mesh = new Mesh("../resources/objects/cube.obj", meshColour);

    Vector4f planeColour = { 1.0f, 1.0f, 1.0f, 1.0f };
    plane = new Mesh("../resources/objects/plane.obj", planeColour);
    plane->position = Vector3f(0, -2, 0);

    // Initialise
    Vector3f initialVelocity(0.0f, 0.0f, 0.0f);
    float vertexMass = 1.0f;
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities.push_back(initialVelocity);
        mesh->masses.push_back(1.0f / vertexMass);
    }
}

Simulation::~Simulation() {
    delete camera;
    delete mesh;
    delete plane;
}

void Simulation::update() {

    // Apply external forces
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] += timeStep * gravity;
    }

    // Dampen velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        // TODO
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
        // TODO
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
