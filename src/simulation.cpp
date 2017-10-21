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
    scene->reset();
}
void Simulation::update() {

    // Update bounding boxes TODO for multi-dynamic object collision
    //for (Mesh* mesh : scene->currentConfiguration->simulatedObjects) {
    //    mesh->updateBoundingBox();
    //}

    simulate(scene->currentConfiguration);

    windOscillation += 0.05f;

    // Render scene
    scene->render(wireframe);
}

void Simulation::simulate(Configuration *configuration) {

    // Apply external forces
    for (Mesh* mesh : configuration->simulatedObjects) {
        if (mesh->gravityAffected) mesh->applyImpulse(2.0f * timeStep * Vector3f(0, -gravity, 0));
        if (mesh->windAffected) mesh->applyImpulse(2.0f * timeStep * Vector3f(0, 0, -windSpeed + (sinf(windOscillation) * windSpeed / 2.0f)));
    }

    // Dampen velocities TODO better velocity damping
    for (Mesh* mesh : configuration->simulatedObjects) {
        #pragma omp parallel for
        for (int i = 0; i < mesh->numVertices; i++) {
            mesh->velocities[i] *= velocityDamping;
        }
    }

    // Initialise estimate positions
    for (Mesh* mesh : configuration->simulatedObjects) {
        #pragma omp parallel for
        for (int i = 0; i < mesh->numVertices; i++) {
            configuration->estimatePositions[i + mesh->estimatePositionsOffset] = mesh->vertices[i] + timeStep * mesh->velocities[i];
        }
    }

    // Clear existing collision constraints
    for (int c = 0; c < configuration->collisionConstraints.size(); c++) delete configuration->collisionConstraints[c];
    configuration->collisionConstraints.clear();

    // Generate collision constraints
    for (Mesh* mesh : configuration->simulatedObjects) {
        for (int i = 0; i < mesh->numVertices; i++) {
            generateCollisionConstraints(configuration, mesh, i);
        }
    }

    // Setup constraint parameters
    Params params;
    params.solverIterations = solverIterations;
    params.stretchFactor = stretchFactor;
    params.bendFactor = bendFactor;

    // Project constraints iteratively
    for (int iteration = 0; iteration < solverIterations; iteration++) {
        //#pragma omp parallel for // Improves performance but constraint solving order is not deterministic
        for (int c = 0; c < configuration->constraints.size(); c++) {
            configuration->constraints[c]->project(configuration, params);
        }

        //#pragma omp parallel for // Improves performance but constraint solving order is not deterministic
        for (int c = 0; c < configuration->collisionConstraints.size(); c++) {
            configuration->collisionConstraints[c]->project(configuration, params);
        }
    }

    // Update positions and velocities
    for (Mesh* mesh : configuration->simulatedObjects) {
        #pragma omp parallel for
        for (int i = 0; i < mesh->numVertices; i++) {
            mesh->velocities[i] = (configuration->estimatePositions[mesh->estimatePositionsOffset + i] - mesh->vertices[i]) / timeStep;
            mesh->vertices[i] = configuration->estimatePositions[mesh->estimatePositionsOffset + i];
        }
    }

    // Update velocities of colliding vertices
    #pragma omp parallel for
    for (int c = 0; c < configuration->collisionConstraints.size(); c++) {
        updateCollisionVelocities(configuration->collisionConstraints[c]);
    }
}

void Simulation::generateCollisionConstraints(Configuration* configuration, Mesh *mesh, int index) {

    // Setup ray
    Vector3f rayOrigin = mesh->vertices[index];
    Vector3f vertexToEstimate = mesh->vertices[index] - configuration->estimatePositions[mesh->estimatePositionsOffset + index];
    Vector3f rayDirection = Vector3f(vertexToEstimate);
    rayDirection.normalize();

    // Setup intersection variables
    float t = INFINITY;
    Vector3f normal;
    int triangleIndex;

    // TODO Dynamic object collision
//    if (false && !mesh->isRigidBody) {
//        bool meshCollision = mesh->intersect(rayOrigin, rayDirection, t, normal, index, triangleIndex);
//
//        t = fabs(t);
//
//        if (meshCollision && 0 < t && t <= CLOTH_THICKNESS) {
//            Triangle triangle = mesh->triangles[triangleIndex];
//
//            if ((mesh->vertices[triangle.v[0].p] - mesh->vertices[index]).dot(normal) > 0.0f) {
//                configuration->collisionConstraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, CLOTH_THICKNESS, triangle.v[0].p, triangle.v[1].p, triangle.v[2].p)));
//            } else {
//                configuration->collisionConstraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, CLOTH_THICKNESS, triangle.v[0].p, triangle.v[2].p, triangle.v[1].p)));
//            }
//        }
//    }

    // Static mesh collision
    for (Mesh* staticMesh : configuration->staticObjects) {
        if (!staticMesh->isRigidBody) continue;

        bool meshCollision = staticMesh->intersect(rayOrigin, rayDirection, t, normal, index, triangleIndex);

        // If a collision occured
        if (meshCollision && fabs(t) * 0.5f <= (vertexToEstimate).norm() + COLLISION_THRESHOLD) {

            // Fix weird negative 0 issue
            if (normal[0] == -0) normal[0] = 0;
            if (normal[1] == -0) normal[1] = 0;
            if (normal[2] == -0) normal[2] = 0;

            // Account for the offset threshold
            if (t >= 0.0f) t -= COLLISION_THRESHOLD;
            else t += COLLISION_THRESHOLD;

            Vector3f intersectionPoint = (rayOrigin + t * rayDirection);

            // Build the collision constraint
            CollisionConstraint* constraint = buildStaticCollisionConstraint(mesh, index, normal, intersectionPoint);
            configuration->collisionConstraints.push_back(constraint);
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
    if (fabs(denom) < EPSILON) return false;

    t = num / denom;

    return true;
}

void Simulation::updateCollisionVelocities(CollisionConstraint* constraint) {
    Mesh* mesh = constraint->mesh;
    int index = constraint->indices[0] - mesh->estimatePositionsOffset;

    Vector3f updatedVelocity = mesh->velocities[index];

    // Reflect the velocity vector around the collision normal
    updatedVelocity = updatedVelocity - 2 * updatedVelocity.dot(constraint->normal) * constraint->normal;

    mesh->velocities[index] = updatedVelocity;

    // TODO Friction / restitution
}

void Simulation::renderGUI() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    ImGui::Begin("Simulator");

    ImGui::Text("Scene Selection");
    if (ImGui::Button("Show Scene A")) scene->setConfiguration(0);
    if (ImGui::Button("Show Scene B")) scene->setConfiguration(1);
    if (ImGui::Button("Show Scene C")) scene->setConfiguration(2);
    if (ImGui::Button("Show Scene D")) scene->setConfiguration(3);

    ImGui::Text("Solver Iterations");
    ImGui::SliderInt("##solverIterations", &solverIterations, 1, 50, "%.0f");

    ImGui::Text("Timestep");
    ImGui::SliderFloat("##timeStep", &timeStep, 0.001f, 0.1f, "%.3f");

    ImGui::Text("Gravity");
    ImGui::SliderFloat("##gravity", &gravity, -10.0f, 10.0f, "%.2f");

    ImGui::Text("WindSpeed");
    ImGui::SliderFloat("##windSpeed", &windSpeed, 0.01f, 10.0f, "%.2f");

    //ImGui::Text("Velocity Damping");
    //ImGui::SliderFloat("##velocityDamping", &velocityDamping, 0.5f, 1.0f, "%.3f");

    ImGui::Text("Stretch Factor");
    ImGui::SliderFloat("##stretchFactor", &stretchFactor, 0.01f, 1.0f, "%.3f");

    ImGui::Text("Bend Factor");
    ImGui::SliderFloat("##bendFactor", &bendFactor, 0.0f, 1.0f, "%.3f");

    ImGui::Text("Wireframe");
    ImGui::Checkbox("##wireframe", &wireframe);

    ImGui::End();
}
