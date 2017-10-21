//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>

void buildEdgeConstraints(Configuration* configuration, Mesh* mesh) {

    // Build a distance constraint along each edge
    for (Edge edge : mesh->edges) {
        int v0 = edge.v[0].p;
        int v1 = edge.v[1].p;

        buildDistanceConstraint(configuration, mesh, v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm());
    }
}

void buildRigidBodyConstraints(Configuration* configuration, Mesh* mesh) {

    // Build a distance constraint between each vertex
    for (int i = 0; i < mesh->numVertices; i++) {
        for (int j = i + 1; j < mesh->numVertices; j++) {
            buildDistanceConstraint(configuration, mesh, i, j, (mesh->vertices[i] - mesh->vertices[j]).norm());
        }
    }
}

void buildBendConstraints(Configuration* configuration, Mesh* mesh) {

    // Build a bend constraint between each pair of triangles that share an edge
    for (Edge edge : mesh->edges) {

        // Skip edges with only one adjacent triangle
        if (mesh->adjacentTriangles[edge].size() != 2) continue;

        Triangle t1 = mesh->adjacentTriangles[edge][0];
        Triangle t2 = mesh->adjacentTriangles[edge][1];

        int p1 = edge.v[0].p; // Shared vertex 1
        int p2 = edge.v[1].p; // Shared vertex 2

        // Determine vertex 3
        int p3;
        for (Vertex v : t1.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p3 = p;
        }

        // Determine vertex 4
        int p4;
        for (Vertex v : t2.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p4 = p;
        }

        // Compute the initial dot product between the two triangles
        Vector3f n1 = mesh->vertices[p2].cross(mesh->vertices[p3]) / mesh->vertices[p2].cross(mesh->vertices[p3]).norm();
        Vector3f n2 = mesh->vertices[p2].cross(mesh->vertices[p4]) / mesh->vertices[p2].cross(mesh->vertices[p4]).norm();
        float d = n1.dot(n2);

        buildBendConstraint(configuration, mesh, p1, p2, p3, p4, acosf(d));
    }
}

void buildTwoWayCouplingConstraints(Configuration* configuration, Mesh* meshA) {

    // Build a distance constraints between objects if they have vertices that overlap
    for (Mesh* meshB : configuration->simulatedObjects) {
        if (meshA == meshB) continue;

        for (int i = 0; i < meshA->numVertices; i++) {
            for (int j = 0; j < meshB->numVertices; j++) {
                if (meshA->vertices[i] == meshB->vertices[j]) {
                    buildDistanceConstraint(configuration, meshA, i, j, 0.0f, meshB);
                }
            }
        }
    }
}

void buildFixedConstraint(Configuration* configuration, Mesh* mesh, int index, Vector3f target) {
    configuration->inverseMasses[index + mesh->estimatePositionsOffset] = EPSILON;

    Constraint* constraint = new FixedConstraint(mesh, 1, target);
    constraint->indices.push_back(index + mesh->estimatePositionsOffset);

    configuration->constraints.push_back(constraint);
}

void buildDistanceConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, float distance, Mesh* secondMesh) {
    Constraint* constraint = new DistanceConstraint(mesh, 2, distance);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);

    // If a second mesh is provided we are building a constraint between two dynamic objects
    if (secondMesh != nullptr) {
        constraint->indices.push_back(indexB + secondMesh->estimatePositionsOffset);
    } else {
        constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    }

    constraint->preCompute(configuration);

    configuration->constraints.push_back(constraint);
}

void buildBendConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float angle) {
    Constraint* constraint = new BendConstraint(mesh, 4, angle);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexD + mesh->estimatePositionsOffset);

    configuration->constraints.push_back(constraint);
}

CollisionConstraint* buildStaticCollisionConstraint(Mesh* mesh, int index, Vector3f normal, Vector3f position) {
    CollisionConstraint* constraint = new StaticCollisionConstraint(mesh, 1, normal, position);
    constraint->indices.push_back(index + mesh->estimatePositionsOffset);

    return constraint;
}

CollisionConstraint* buildTriangleCollisionConstraint(Mesh *mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC) {
    CollisionConstraint* constraint = new TriangleCollisionConstraint(mesh, 1, normal, height);
    constraint->indices.push_back(vertexIndex + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + mesh->estimatePositionsOffset);

    return constraint;
}

void FixedConstraint::project(Configuration* configuration, Params params) {
    configuration->estimatePositions[indices[0]] = target;
}

void DistanceConstraint::preCompute(Configuration* configuration) {
    coefficients.resize(cardinality, cardinality);
    coefficients.setZero();

    float w1 = configuration->inverseMasses[indices[0]];
    float w2 = configuration->inverseMasses[indices[1]];
    coefficients.coeffRef(0, 0) = 1.0f / (w1 / (w1 + w2));
    coefficients.coeffRef(1, 1) = 1.0f / (w2 / (w1 + w2));
}

void DistanceConstraint::project(Configuration* configuration, Params params) {
    MatrixXf RHS = MatrixXf::Zero(cardinality, 3);

    Vector3f p1 = configuration->estimatePositions[indices[0]];
    Vector3f p2 = configuration->estimatePositions[indices[1]];

    float a = (p1 - p2).norm() - distance;
    Vector3f b = (p1 - p2) / ((p1 - p2).norm() + EPSILON);

    RHS.row(0) = -a * b;
    RHS.row(1) = a * b;

    MatrixXf displacements = coefficients.llt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        Vector3f displacement = displacements.row(i);
        float k;
        if (mesh->isRigidBody) k = 0.25f;
        else k = 1.0f - pow(1.0f - params.stretchFactor, 1.0f / params.solverIterations);
        configuration->estimatePositions[indices[i]] += k * displacement;
    }
}

void BendConstraint::project(Configuration* configuration, Params params) {
    MatrixXf RHS = MatrixXf::Zero(cardinality, 3);

    Vector3f p1 = configuration->estimatePositions[indices[0]];
    Vector3f p2 = configuration->estimatePositions[indices[1]];
    Vector3f p3 = configuration->estimatePositions[indices[2]];
    Vector3f p4 = configuration->estimatePositions[indices[3]];

    Vector3f p2Xp3 = p2.cross(p3);
    Vector3f p2Xp4 = p2.cross(p4);

    // Compute normals
    Vector3f n1 = p2Xp3 / p2Xp3.norm();
    Vector3f n2 = p2Xp4 / p2Xp4.norm();
    float d = n1.dot(n2);

    Vector3f q3 = (p2.cross(n2) + d * n1.cross(p2)) / (p2Xp3.norm());
    Vector3f q4 = (p2.cross(n1) + d * n2.cross(p2)) / (p2Xp4.norm());
    Vector3f q2 = -(p3.cross(n2) + d * n1.cross(p3)) / (p2Xp3.norm()) - (p4.cross(n1) + d * n2.cross(p4)) / (p2Xp4.norm());
    Vector3f q1 = -q2 - q3 - q4;

    float qSum = q1.squaredNorm() + q2.squaredNorm() + q3.squaredNorm() + q4.squaredNorm();

    // Compute coefficient matrix
    coefficients.resize(cardinality, cardinality);
    coefficients.setZero();

    float denominator = 0.0f;
    for (int i = 0; i < cardinality; i++) {
        denominator += configuration->inverseMasses[indices[i]] * qSum;
    }

    for (int i = 0; i < cardinality; i++) {
        float wi = configuration->inverseMasses[indices[i]];
        coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
    }

    // Prevent issue where d falls out of range -1 to 1
    d = fmax(fmin(d, 1.0f), -1.0f);

    float a = sqrtf(1.0f - d * d) * (acosf(d) - angle);
    RHS.row(0) = -a * q1;
    RHS.row(1) = -a * q2;
    RHS.row(2) = -a * q3;
    RHS.row(3) = -a * q4;

    MatrixXf displacements = coefficients.llt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        Vector3f displacement = displacements.row(i);
        float k;
        if (mesh->isRigidBody) k = 0.25f;
        else k = 1.0f - pow(1.0f - params.bendFactor, 1.0f / params.solverIterations);
        configuration->estimatePositions[indices[i]] += k * displacement;
    }
}

void StaticCollisionConstraint::project(Configuration* configuration, Params params) {
    Vector3f p = configuration->estimatePositions[indices[0]];

    Vector3f pointToPosition = p - position;
    pointToPosition.normalize();

    // Check if constraint is already satisfied
    if (pointToPosition.dot(normal) >= 0.0f) return;

    float a = (p - position).dot(normal);
    Vector3f b = (p - position) / ((p - position).norm());

    Vector3f displacement = a * b;

    configuration->estimatePositions[indices[0]] += displacement;
}

void TriangleCollisionConstraint::project(Configuration* configuration, Params params) {
    Vector3f q = configuration->estimatePositions[indices[0]];
    Vector3f p1 = configuration->estimatePositions[indices[1]];
    Vector3f p2 = configuration->estimatePositions[indices[2]];
    Vector3f p3 = configuration->estimatePositions[indices[3]];

    // Check if constraint is already satisfied
    Vector3f n = (p2 - p1).cross(p3 - p1);
    n /= n.norm();

    normal = n;

    Vector3f qToP1 = q - p1;
    qToP1.normalize();

    if (qToP1.dot(n) - height >= 0.0f) return;

    float a = (q - p1).dot(n) - height;
    Vector3f b = n;

    Vector3f displacement = a * b;

    configuration->estimatePositions[indices[0]] -= displacement;
}
