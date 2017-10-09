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
    testCube->isRigidBody = true;

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

    Mesh* simple = new Mesh("../resources/models/simple.obj", planeColour);
    simple->gravityAffected = true;

    // Setup constraints
    //testCube->constraints.push_back(buildFixedConstraint(testCube, 3, testCube->initialVertices[3]));
    buildEdgeConstraints(testCube);

    for (int i = 0; i < 7; i++) {
        flag->constraints.push_back(buildFixedConstraint(flag, i, flag->initialVertices[i]));
    }
    buildEdgeConstraints(flag);
    buildBendConstraints(flag);

    for (int i = 0; i < 14; i++) {
        flagHigh->constraints.push_back(buildFixedConstraint(flagHigh, i, flagHigh->initialVertices[i]));
    }
    buildEdgeConstraints(flagHigh);
    buildBendConstraints(flagHigh);

    simple->constraints.push_back(buildFixedConstraint(simple, 0, simple->initialVertices[0]));
    simple->constraints.push_back(buildFixedConstraint(simple, 1, simple->initialVertices[1]));
    simple->constraints.push_back(buildFixedConstraint(simple, 2, simple->initialVertices[2]));

    // Build object lists
    staticObjects.push_back(plane);
    staticObjects.push_back(flagPole);
    staticObjects.push_back(flagPole2);
//    staticObjects.push_back(simple);

//    simulatedObjects.push_back(testCube);
//    simulatedObjects.push_back(flag);
//    simulatedObjects.push_back(flagHigh);
    simulatedObjects.push_back(simple);

    //omp_set_num_threads(4);

    reset();
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

    //simulatedObjects[0]->applyImpulse(Vector3f(1.0f, 2.5f, -0.5f));
    //simulatedObjects[0]->vertices[0] += Vector3f(1.50f, 0.0f, 0.0f);
}
void Simulation::update() {
    //#pragma omp parallel for
    //for (int i = 0; i < simulatedObjects.size(); i++) {
    //    simulate(simulatedObjects[i]);
    //}
    for (Mesh* mesh : simulatedObjects) {
        simulate(mesh);
    }

    windOscillation += 0.05f;
}

void Simulation::simulate(Mesh* mesh) {

    // Apply external forces
    if (mesh->gravityAffected) mesh->applyImpulse(timeStep * Vector3f(0, -gravity, 0));
    if (mesh->windAffected) mesh->applyImpulse(timeStep * Vector3f(0, 0, -windSpeed + (sinf(windOscillation) * windSpeed / 2.0f)));

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
    vector<CollisionConstraint*> collisionConstraints;
    for (int i = 0; i < mesh->numVertices; i++) {
        generateCollisionConstraints(mesh, i, collisionConstraints);
    }

    // Setup params
    Params params;
    params.solverIterations = solverIterations;
    params.stretchFactor = stretchFactor;
    params.bendFactor = bendFactor;

    // Project constraints iteratively
    for (int iteration = 0; iteration < solverIterations; iteration++) {
        //#pragma omp parallel for
        //for (int c = 0; c < mesh->constraints.size(); c++) {
        //    mesh->constraints[c]->project(solverIterations);
        //}
        for (Constraint* constraint : mesh->constraints) {
            constraint->project(params);
        }

        for (CollisionConstraint* constraint : collisionConstraints) {
            constraint->project(params);
        }
    }

    // Update positions and velocities
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] = (mesh->estimatePositions[i] - mesh->vertices[i]) / timeStep;
        mesh->vertices[i] = mesh->estimatePositions[i];
    }

    // Update velocities of colliding vertices
    for (CollisionConstraint* constraint : collisionConstraints) {
        updateCollisionVelocities(constraint);
    }
}

void Simulation::generateCollisionConstraints(Mesh* mesh, int index, vector<CollisionConstraint*> &constraints) {

    // Setup ray
    Vector3f rayOrigin = mesh->vertices[index];
    Vector3f rayDirection = mesh->vertices[index] - mesh->estimatePositions[index];
    rayDirection.normalize();

    // Setup intersection variables
    float t;
    Vector3f normal;
    int triangleIndex;

    // Check for self intersection
    if (!mesh->isRigidBody && index == 3) {
        bool meshCollision = mesh->intersect(rayOrigin, rayDirection, t, normal, index, triangleIndex);
//        if (meshCollision) cout << t << endl;
        if (meshCollision && 0 >= t && t >= -CLOTH_THICKNESS) {
//            cout << "c" << normal << endl << endl;
            constraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, triangleIndex, CLOTH_THICKNESS));
        }
    }

    // Check for plane collision
    bool planeCollision = planeIntersection(rayOrigin, rayDirection, t, normal);
    if (planeCollision && 1 / timeStep * COLLISION_THRESHOLD >= t) {
        Vector3f intersectionPoint = rayOrigin + t * rayDirection;
        constraints.push_back(buildStaticCollisionConstraint(mesh, index, normal, intersectionPoint));
    }
}

bool Simulation::planeIntersection(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal) {

    // Hardcoded plane position for now
    Vector3f position = Vector3f(0.0f, -3.0f, 0.0f);
    normal = Vector3f(0.0f, 1.0f, 0.0f);

    float num = (position - rayOrigin).dot(normal);
    float denom = normal.dot(rayDirection);

    // Ray is parallel with plane
    if (fabs(denom) < 0.000001f) return false;

    t = num / denom;

    return t >= 0.0f;
}

void Simulation::updateCollisionVelocities(CollisionConstraint* constraint) {
    Mesh* mesh = constraint->mesh;
    int index = constraint->indices[0];
    Vector3f updatedVelocity = mesh->velocities[index];

    // Reflect the velocity vector around the collision normal
    updatedVelocity = updatedVelocity - 2 * updatedVelocity.dot(constraint->normal) * constraint->normal;

    mesh->velocities[index] = updatedVelocity;

    // TODO Friction / restitution
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
    ImGui::SliderFloat("##gravity", &gravity, -10.01f, 10.0f, "%.2f");

    ImGui::Text("WindSpeed");
    ImGui::SliderFloat("##windSpeed", &windSpeed, 0.01f, 10.0f, "%.2f");

    ImGui::Text("Velocity Damping");
    ImGui::SliderFloat("##velocityDamping", &velocityDamping, 0.5f, 1.0f, "%.3f");

    ImGui::Text("Stretch Factor");
    ImGui::SliderFloat("##stretchFactor", &stretchFactor, 0.0f, 1.0f, "%.3f");

    ImGui::Text("Bend Factor");
    ImGui::SliderFloat("##bendFactor", &bendFactor, 0.0f, 1.0f, "%.3f");

    ImGui::Text("Wireframe");
    ImGui::Checkbox("##wireframe", &wireframe);

    ImGui::End();
}