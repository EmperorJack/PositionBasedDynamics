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
int solverIterations = 1;
Vector3f pos;
int constraintedVertex = 3;

Simulation::Simulation() {

    // Setup camera
    camera = new Camera();

    // Setup objects
    Vector4f meshColour = { 0.15f, 0.45f, 0.8f, 1.0f };
    mesh = new Mesh("../resources/objects/cube.obj", meshColour);

    Vector4f planeColour = { 1.0f, 1.0f, 1.0f, 1.0f };
    plane = new Mesh("../resources/objects/plane.obj", planeColour);
    plane->position = Vector3f(0, -2, 0);

    reset();
}

Simulation::~Simulation() {
    delete camera;
    delete mesh;
    delete plane;
}

void Simulation::reset() {
    mesh->vertices = mesh->initialVertices;

    mesh->velocities.clear();
    mesh->masses.clear();

    Vector3f initialVelocity(0.0f, 0.0f, 0.0f);
    float vertexMass = 1.0f;
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities.push_back(initialVelocity);
        mesh->masses.push_back(1.0f / vertexMass);
    }

//    mesh->masses[constraintedVertex] = 0.000000000000000001f;
    pos = mesh->vertices[constraintedVertex];
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
        SparseMatrix<float> A(mesh->numVertices, mesh->numVertices);

        for (int i = 0; i < mesh->numVertices; i++) {
            A.coeffRef(i, i) = mesh->masses[i];
        }

//        cout << A << endl;

        solver.compute(A);

        if(solver.info() != Success) {
            std::cout << "Factorisation failed" << std::endl;
            exit(-1);
        }

        MatrixXf B(mesh->numVertices, 3);
        for (int i = 0; i < mesh->numVertices; i++) {
            if (i == constraintedVertex) {
                B.row(i) = pos;
            } else {
                B.row(i) = mesh->estimatePositions[i];
            }
        }

        MatrixXf X = solver.solve(B);

        if(solver.info() != Success) {
            std::cout << "Solving failed" << std::endl;
            exit(-1);
        }

//        cout << A << "~~~" << endl;
//        cout << B << endl << "~~~" << endl;
//        cout << X << endl;

//        exit(0);

//        int index = constraintedVertex;
//        cout << "~~~~~~~~~~" << endl;
//        cout << "[" << A.row(index) << "] * [" << X.row(index) << "] = [" << B.row(index) << "]" << endl;

        for (int i = 0; i < mesh->numVertices; i++) {
            mesh->estimatePositions[i] = X.row(i);
        }
    }

    // Update positions and velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] = (mesh->estimatePositions[i] - mesh->vertices[i]) / timeStep;
        mesh->vertices[i] = mesh->estimatePositions[i];
    }

    // Update velocities of colliding vertices
    // TODO

    // Ensure attachments are positioned correctly
    //mesh->vertices[constraintedVertex] = pos;
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
    //plane->render(camera, modelMatrix);
}
