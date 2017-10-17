//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <imgui.h>
#include <omp.h>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {

    // Setup scene
    scene = new Scene();

    omp_set_num_threads(4);

    reset();
}

Simulation::~Simulation() {
    delete scene;
}

void Simulation::reset() {
    for (Mesh* mesh : scene->simulatedObjects) {
        mesh->reset();
    }
}
void Simulation::update() {

    // Update bounding boxes
    for (Mesh* mesh : scene->simulatedObjects) {
        //mesh->updateBoundingBox();
    }

    for (Mesh* mesh : scene->simulatedObjects) {
        simulate(mesh);
    }

    windOscillation += 0.05f;

    // Render scene
    scene->render(wireframe);
}

void Simulation::simulate(Mesh* mesh) {

    // Apply external forces
    if (mesh->gravityAffected) mesh->applyImpulse(timeStep * Vector3f(0, -gravity, 0));
    if (mesh->windAffected) mesh->applyImpulse(timeStep * Vector3f(0, 0, -windSpeed + (sinf(windOscillation) * windSpeed / 2.0f)));

    // Dampen velocities TODO
    //for (int i = 0; i < mesh->numVertices; i++) {
    //    mesh->velocities[i] *= velocityDamping;
    //}

    mesh->estimatePositions.clear();
    mesh->estimatePositions.resize((size_t) mesh->numVertices, Vector3f::Zero());

    // Initialise estimate positions
    #pragma omp parallel for
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->estimatePositions[i] = mesh->vertices[i] + timeStep * mesh->velocities[i];
    }

    // Generate collision constraints
    vector<CollisionConstraint*> collisionConstraints;
    //#pragma omp parallel for
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
        for (int c = 0; c < mesh->constraints.size(); c++) {
            mesh->constraints[c]->project(params);
        }

        //#pragma omp parallel for
        for (int c = 0; c < collisionConstraints.size(); c++) {
            collisionConstraints[c]->project(params);
        }
    }

    // Update positions and velocities
    #pragma omp parallel for
    for (int i = 0; i < mesh->numVertices; i++) {
        mesh->velocities[i] = (mesh->estimatePositions[i] - mesh->vertices[i]) / timeStep;
        mesh->vertices[i] = mesh->estimatePositions[i];
    }

    // Update velocities of colliding vertices
    #pragma omp parallel for
    for (int c = 0; c < collisionConstraints.size(); c++) {
        updateCollisionVelocities(collisionConstraints[c]);
    }
}

void Simulation::generateCollisionConstraints(Mesh* mesh, int index, vector<CollisionConstraint*> &constraints) {

    // Setup ray
    Vector3f rayOrigin = mesh->vertices[index];
    Vector3f rayDirection = mesh->vertices[index] - mesh->estimatePositions[index];
    rayDirection.normalize();

    // Setup intersection variables
    float t = INFINITY;
    Vector3f normal;
    int triangleIndex;

    // Multiple dynamic object collision
//    if (false && !mesh->isRigidBody) {
//        bool meshCollision = mesh->intersect(rayOrigin, rayDirection, t, normal, index, triangleIndex);
//
//        t = fabs(t);
//
//        if (meshCollision && 0 < t && t <= CLOTH_THICKNESS) {
//            Triangle triangle = mesh->triangles[triangleIndex];
//
//            if ((mesh->vertices[triangle.v[0].p] - mesh->vertices[index]).dot(normal) > 0.0f) {
//                constraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, CLOTH_THICKNESS, triangle.v[0].p, triangle.v[1].p, triangle.v[2].p));
//            } else {
//                constraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, CLOTH_THICKNESS, triangle.v[0].p, triangle.v[2].p, triangle.v[1].p));
//            }
//        }
//    }

    for (Mesh* staticMesh : scene->staticObjects) {
        if (!staticMesh->isRigidBody) continue;

        bool meshCollision = staticMesh->intersect(rayOrigin, rayDirection, t, normal, index, triangleIndex);

        if (meshCollision && (fabs(t) - CLOTH_THICKNESS) <= (mesh->vertices[index] - mesh->estimatePositions[index]).norm()) {

            // Fix weird negative 0 issue
            if (normal[0] == -0) normal[0] = 0;
            if (normal[1] == -0) normal[1] = 0;
            if (normal[2] == -0) normal[2] = 0;

            if (t > 0.0f) t -= CLOTH_THICKNESS;
            else t += CLOTH_THICKNESS;

            Vector3f intersectionPoint = (rayOrigin + t * rayDirection);

            constraints.push_back(buildStaticCollisionConstraint(mesh, index, normal, intersectionPoint));
        }
    }
}

// Parametric plane collision
bool Simulation::planeIntersection(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal) {

    // Hardcoded plane position for now
    Vector3f position = Vector3f(0.0f, -3.0f, 0.0f);
    normal = Vector3f(0.0f, 1.0f, 0.0f);

    float num = (position - rayOrigin).dot(normal);
    float denom = normal.dot(rayDirection);

    // Ray is parallel with plane
    if (fabs(denom) < 0.000001f) return false;

    t = num / denom;

    return true;
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

void Simulation::renderGUI() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    ImGui::Begin("Simulator");

    ImGui::Text("Solver Iterations");
    ImGui::SliderInt("##solverIterations", &solverIterations, 1, 50, "%.0f");

    ImGui::Text("Timestep");
    ImGui::SliderFloat("##timeStep", &timeStep, 0.001f, 0.1f, "%.3f");

    ImGui::Text("Gravity");
    ImGui::SliderFloat("##gravity", &gravity, -10.0f, 10.0f, "%.2f");

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