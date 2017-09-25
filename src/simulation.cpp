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
    cubeMesh = new Mesh("../resources/models/cube.obj", meshColour);
    cubeMesh->position = Vector3f(-1.0f - 5.0f, 0.0f, -1.0f - 5.0f);

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    planeMesh = new Mesh("../resources/models/plane.obj", planeColour);
    planeMesh->position = Vector3f(0, -3, 0);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    flagPoleMesh = new Mesh("../resources/models/flagPole.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    flagMesh = new Mesh("../resources/models/flag.obj", flagColour);

    // Setup constraints
    cubeMesh->constraints.push_back(buildFixedConstraint(3));
    buildEdgeConstraints(cubeMesh, 1.0f);

    reset();
}

Simulation::~Simulation() {
    delete camera;
    delete cubeMesh;
    delete planeMesh;
}

void Simulation::reset() {
    cubeMesh->vertices = cubeMesh->initialVertices;

    cubeMesh->velocities.clear();
    cubeMesh->inverseMasses.clear();

    Vector3f initialVelocity(1.0f, 0.0f, -1.0f);
    float vertexMass = 1.0f;
    for (int i = 0; i < cubeMesh->numVertices; i++) {
        cubeMesh->velocities.push_back(initialVelocity);
        cubeMesh->inverseMasses.push_back(1.0f / vertexMass);
    }
}

void Simulation::buildEdgeConstraints(Mesh* mesh, float stiffness) {
    for (Edge edge : mesh->edges) {
        int v0 = edge.v[0].p;
        int v1 = edge.v[1].p;

        mesh->constraints.push_back(buildDistanceConstraint(
                v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm(), stiffness
        ));
    }
}

Constraint Simulation::buildFixedConstraint(int index) {
    Constraint constraint;
    constraint.type = FIXED;
    constraint.indices.push_back(index);
    constraint.cardinatlity = 1;
    constraint.target = cubeMesh->initialVertices[index];
    constraint.stiffness = 1.0f;
    return constraint;
}

Constraint Simulation::buildDistanceConstraint(int indexA, int indexB, float distance, float stiffness) {
    Constraint constraint;
    constraint.type = DISTANCE;
    constraint.indices.push_back(indexA);
    constraint.indices.push_back(indexB);
    constraint.cardinatlity = 2;
    constraint.distance = distance;
    constraint.stiffness = stiffness;
    return constraint;
}

void Simulation::update() {
    simulate(cubeMesh);
}

void Simulation::simulate(Mesh* mesh) {
    // Apply external forces
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] += timeStep * Vector3f(0, -gravity, 0);
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
        for (Constraint constraint : cubeMesh->constraints) {

            SparseMatrix<float> coefficients(constraint.cardinatlity, constraint.cardinatlity);

            if (constraint.type == FIXED) {
                coefficients.coeffRef(0, 0) = -1.0f;
            } else if (constraint.type == DISTANCE) {
                float w1 = mesh->inverseMasses[constraint.indices[0]];
                float w2 = mesh->inverseMasses[constraint.indices[1]];
                coefficients.coeffRef(0, 0) = -1 / (w1 / (w1 + w2));
                coefficients.coeffRef(1, 1) = 1 / (w2 / (w1 + w2));
            }

            solver.compute(coefficients);

            if (solver.info() != Success) {
                std::cout << "Factorisation failed" << std::endl;
                exit(-1);
            }

            MatrixXf RHS(constraint.cardinatlity, 3);
            if (constraint.type == FIXED) {
                Vector3f p1 = mesh->estimatePositions[constraint.indices[0]];
                Vector3f p2 = constraint.target;

                float a = (p1 - p2).norm();
                Vector3f b = (p1 - p2) / (p1 - p2).norm();

                RHS.row(0) = a * b;
            } else if (constraint.type == DISTANCE) {
                Vector3f p1 = mesh->estimatePositions[constraint.indices[0]];
                Vector3f p2 = mesh->estimatePositions[constraint.indices[1]];

                float a = ((p1 - p2).norm() - constraint.distance);
                Vector3f b = (p1 - p2) / (p1 - p2).norm();

                RHS.row(0) = a * b;
                RHS.row(1) = a * b;
            }

            MatrixXf displacements = solver.solve(RHS);

            if (solver.info() != Success) {
                std::cout << "Solving failed" << std::endl;
                exit(-1);
            }

            for (int i = 0; i < constraint.cardinatlity; i++) {
                int vertexIndex = constraint.indices[i];
                Vector3f displacement = displacements.row(i);
                float stiffness = 1.0f - pow(1.0f - constraint.stiffness, 1.0f / solverIterations);
                mesh->estimatePositions[vertexIndex] += stiffness * displacement;
            }
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
    camera->lookAt(Vector3f(0, 0, 12), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    cubeMesh->render(camera, modelMatrix);
    planeMesh->render(camera, modelMatrix);
    flagPoleMesh->render(camera, modelMatrix);
    flagMesh->render(camera, modelMatrix);
}

void Simulation::renderGUI() {
    ImGui::Begin("Simulator");

    ImGui::Text("Solver Iterations");
    ImGui::SliderInt("##solverIterations", &solverIterations, 1, 50, "%.0f");

    ImGui::Text("Timestep");
    ImGui::SliderFloat("##timeStep", &timeStep, 0.01f, 1.0f, "%.2f");

    ImGui::Text("Gravity");
    ImGui::SliderFloat("##gravity", &gravity, 0.01f, 10.0f, "%.2f");

    ImGui::Text("Velocity Damping");
    ImGui::SliderFloat("##velocityDamping", &velocityDamping, 0.5f, 1.0f, "%.3f");

    ImGui::End();
}