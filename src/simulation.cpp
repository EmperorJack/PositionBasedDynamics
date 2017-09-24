//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

const float TIME_STEP = 0.05f;
const Vector3f GRAVITY(0, -0.981f, 0);
const int SOLVER_ITERATIONS = 1;

Simulation::Simulation() {

    // Setup camera
    camera = new Camera();

    // Setup objects
    Vector4f meshColour = { 0.15f, 0.45f, 0.8f, 1.0f };
    mesh = new Mesh("../resources/objects/cube.obj", meshColour);

    Vector4f planeColour = { 1.0f, 1.0f, 1.0f, 1.0f };
    plane = new Mesh("../resources/objects/plane.obj", planeColour);
    plane->position = Vector3f(0, -2, 0);

    // Setup fixed constraints
    constraints.push_back(buildFixedConstraint(3));
    constraints.push_back(buildFixedConstraint(5));

    // Setup distance constraints
    for (Triangle face : mesh->triangles) {
        int v0 = face.v[0].p;
        int v1 = face.v[1].p;
        int v2 = face.v[2].p;

//        constraints.push_back(buildDistanceConstraint(
//                v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm()
//        ));
//        constraints.push_back(buildDistanceConstraint(
//                v0, v2, (mesh->vertices[v0] - mesh->vertices[v2]).norm()
//        ));
//        constraints.push_back(buildDistanceConstraint(
//                v1, v2, (mesh->vertices[v1] - mesh->vertices[v2]).norm()
//        ));
    }

    cout << constraints.size() << " constraints" << endl;

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
    mesh->inverseMasses.clear();

    Vector3f initialVelocity(0.0f, 0.0f, 0.0f);
    float vertexMass = 1.0f;
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities.push_back(initialVelocity);
        mesh->inverseMasses.push_back(1.0f / vertexMass);
    }
}

Constraint Simulation::buildFixedConstraint(int index) {
    Constraint constraint;
    constraint.type = FIXED;
    constraint.indices.push_back(index);
    constraint.cardinatlity = 1;
    constraint.target = mesh->initialVertices[index];
    return constraint;
}

Constraint Simulation::buildDistanceConstraint(int indexA, int indexB, float distance) {
    Constraint constraint;
    constraint.type = DISTANCE;
    constraint.indices.push_back(indexA);
    constraint.indices.push_back(indexB);
    constraint.cardinatlity = 2;
    constraint.distance = distance;
    return constraint;
}

void Simulation::update() {

    // Apply external forces
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] += TIME_STEP * GRAVITY;
    }

    // Dampen velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        // TODO
    }

    mesh->estimatePositions.clear();
    mesh->estimatePositions.resize((size_t) mesh->numVertices, Vector3f::Zero());

    // Initialise estimate positions
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->estimatePositions[i] = mesh->vertices[i] + TIME_STEP * mesh->velocities[i];
    }

    // Generate collision constraints
    // TODO

    // Project constraints iteratively
    for (int iteration = 0; iteration < SOLVER_ITERATIONS; iteration++) {
        for (Constraint constraint : constraints) {

            SparseMatrix<float> A(constraint.cardinatlity, constraint.cardinatlity);

//            for (int i = 0; i < constraint.cardinatlity; i++) {
            if (constraint.type == FIXED) {
                A.coeffRef(0, 0) = -1.0f;
            } else if (constraint.type == DISTANCE) {
                float w1 = mesh->inverseMasses[constraint.indices[0]];
                float w2 = mesh->inverseMasses[constraint.indices[1]];
                A.coeffRef(0, 0) = -w1 / (w1 + w2);
                A.coeffRef(1, 1) = w2 / (w1 + w2);
            }
//            }

//            cout << A << endl;

            solver.compute(A);

            if (solver.info() != Success) {
                std::cout << "Factorisation failed" << std::endl;
                exit(-1);
            }

            MatrixXf B(constraint.cardinatlity, 3);
//            for (int i = 0; i < constraint.cardinatlity; i++) {
            if (constraint.type == FIXED) {
                Vector3f p1 = mesh->estimatePositions[constraint.indices[0]];
                Vector3f p2 = constraint.target;

                float a = (p1 - p2).norm();
                Vector3f b = (p1 - p2) / (p1 - p2).norm();

//                if (fabs(a) < 0.00001f) a = 0.0f;

                B.row(0) = a * b;
            } else if (constraint.type == DISTANCE) {
                Vector3f p1 = mesh->estimatePositions[constraint.indices[0]];
                Vector3f p2 = mesh->estimatePositions[constraint.indices[1]];
//                cout << p1 << " , " << p2 << endl;

                float a = ((p1 - p2).norm() - constraint.distance);
                Vector3f b = (p1 - p2) / (p1 - p2).norm();

//                if (fabs(a) < 0.00001f) a = 0.0f;

//                cout << "a: " << a << ", b: " << b << endl;

                B.row(0) = a * b;
                B.row(1) = a * b;
            }
//            }

            MatrixXf X = solver.solve(B);

            if (solver.info() != Success) {
                std::cout << "Solving failed" << std::endl;
                exit(-1);
            }

            if (constraint.type == FIXED) {
                cout << "P1: " << mesh->estimatePositions[constraint.indices[0]] << endl;
//                cout << "P2: " << mesh->estimatePositions[constraint.indices[1]] << endl;
                cout << "Target: " << constraint.target << endl;
                cout << "A: " << endl << A << "~~~" << endl;
                cout << "B: " << endl << B << endl << "~~~" << endl;
                cout << "X: " << endl << X << endl << "~~~" << endl << endl;
            }

            for (int i = 0; i < constraint.cardinatlity; i++) {
                int vertexIndex = constraint.indices[i];;
                mesh->estimatePositions[vertexIndex] += X.row(i);
            }
        }
    }

    // Update positions and velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] = (mesh->estimatePositions[i] - mesh->vertices[i]) / TIME_STEP;
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
