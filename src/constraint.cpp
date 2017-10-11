//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>

void buildEdgeConstraints(Mesh* mesh) {
    for (Edge edge : mesh->edges) {
        int v0 = edge.v[0].p;
        int v1 = edge.v[1].p;

        mesh->constraints.push_back(buildDistanceConstraint(mesh, v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm()));
    }
}

void buildBendConstraints(Mesh* mesh) {
    for (Edge edge : mesh->edges) {
        if (mesh->adjacentTriangles[edge].size() != 2) continue;

        Triangle t1 = mesh->adjacentTriangles[edge][0];
        Triangle t2 = mesh->adjacentTriangles[edge][1];

        int p1 = edge.v[0].p; // Shared vertex 1
        int p2 = edge.v[1].p; // Shared vertex 2

        int p3;
        for (Vertex v : t1.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p3 = p;
        }

        int p4;
        for (Vertex v : t2.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p4 = p;
        }

        Vector3f n1 = mesh->vertices[p2].cross(mesh->vertices[p3]) / mesh->vertices[p2].cross(mesh->vertices[p3]).norm();
        Vector3f n2 = mesh->vertices[p2].cross(mesh->vertices[p4]) / mesh->vertices[p2].cross(mesh->vertices[p4]).norm();
        float d = n1.dot(n2);

        mesh->constraints.push_back(buildBendConstraint(mesh, p1, p2, p3, p4, acosf(d)));
    }
}

FixedConstraint* buildFixedConstraint(Mesh* mesh, int index, Vector3f target) {
    mesh->inverseMasses[index] = EPSILON;

    FixedConstraint* constraint = new FixedConstraint(mesh, 1, target);
    constraint->indices.push_back(index);
    return constraint;
}

DistanceConstraint* buildDistanceConstraint(Mesh* mesh, int indexA, int indexB, float distance) {
    DistanceConstraint* constraint = new DistanceConstraint(mesh, 2, distance);
    constraint->indices.push_back(indexA);
    constraint->indices.push_back(indexB);

    constraint->preCompute();
    return constraint;
}

BendConstraint* buildBendConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float angle) {
    BendConstraint* constraint = new BendConstraint(mesh, 4, angle);
    constraint->indices.push_back(indexA);
    constraint->indices.push_back(indexB);
    constraint->indices.push_back(indexC);
    constraint->indices.push_back(indexD);

    return constraint;
}

StaticCollisionConstraint* buildStaticCollisionConstraint(Mesh* mesh, int index, Vector3f normal, Vector3f position) {
    StaticCollisionConstraint* constraint = new StaticCollisionConstraint(mesh, 1, normal, position);
    constraint->indices.push_back(index);
    return constraint;
}

TriangleCollisionConstraint *
buildTriangleCollisionConstraint(Mesh *mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC) {
    TriangleCollisionConstraint* constraint = new TriangleCollisionConstraint(mesh, 1, normal, height);
    constraint->indices.push_back(vertexIndex);
    constraint->indices.push_back(indexA);
    constraint->indices.push_back(indexB);
    constraint->indices.push_back(indexC);
    return constraint;
}

void FixedConstraint::project(Params params) {
    mesh->estimatePositions[indices[0]] = target;
}

void DistanceConstraint::preCompute() {
    coefficients.resize(cardinality, cardinality);
    coefficients.setZero();

    float w1 = mesh->inverseMasses[indices[0]];
    float w2 = mesh->inverseMasses[indices[1]];
    coefficients.coeffRef(0, 0) = 1.0f / (w1 / (w1 + w2));
    coefficients.coeffRef(1, 1) = 1.0f / (w2 / (w1 + w2));
}

void DistanceConstraint::project(Params params) {
    MatrixXf RHS = MatrixXf::Zero(cardinality, 3);

    Vector3f p1 = mesh->estimatePositions[indices[0]];
    Vector3f p2 = mesh->estimatePositions[indices[1]];

    float a = (p1 - p2).norm() - distance;
    Vector3f b = (p1 - p2) / ((p1 - p2).norm() + EPSILON);

    RHS.row(0) = -a * b;
    RHS.row(1) = a * b;

    MatrixXf displacements = coefficients.llt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        int vertexIndex = indices[i];
        Vector3f displacement = displacements.row(i);
        float k = 1.0f;
        if (!mesh->isRigidBody) k -= pow(1.0f - params.stretchFactor, 1.0f / params.solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}

void BendConstraint::project(Params params) {
    MatrixXf RHS = MatrixXf::Zero(cardinality, 3);

    Vector3f p1 = mesh->estimatePositions[indices[0]];
    Vector3f p2 = mesh->estimatePositions[indices[1]];
    Vector3f p3 = mesh->estimatePositions[indices[2]];
    Vector3f p4 = mesh->estimatePositions[indices[3]];

    Vector3f p2Xp3 = p2.cross(p3);
    Vector3f p2Xp4 = p2.cross(p4);

    // Compute normals
    Vector3f n1 = p2Xp3 / p2Xp3.norm();
    Vector3f n2 = p2Xp4 / p2Xp4.norm();
    float d = n1.transpose() * n2;

    Vector3f q3 = (p2.cross(n2) + d * n1.cross(p2)) / (p2Xp3.norm());
    Vector3f q4 = (p2.cross(n1) + d * n2.cross(p2)) / (p2Xp4.norm());
    Vector3f q2 = -(p3.cross(n2) + d * n1.cross(p3)) / (p2Xp3.norm()) - (p4.cross(n1) + d * n2.cross(p4)) / (p2Xp4.norm());
    Vector3f q1 = -q2 - q3 - q4;

    // Compute coefficient matrix
    coefficients.resize(cardinality, cardinality);
    coefficients.setZero();

    float denominator = 0.0f;
    for (int i = 0; i < cardinality; i++) {
        denominator += mesh->inverseMasses[indices[i]];
    }

    for (int i = 0; i < cardinality; i++) {
        float wi = mesh->inverseMasses[indices[i]];
        coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
    }

    float qSum = q1.squaredNorm() + q2.squaredNorm() + q3.squaredNorm() + q4.squaredNorm();

    for (int i = 0; i < cardinality; i++) {
        coefficients.coeffRef(i, i) *= qSum;
    }

    float a = sqrtf(1.0f - d * d) * (acosf(d) - angle);
    RHS.row(0) = -a * q1;
    RHS.row(1) = -a * q2;
    RHS.row(2) = -a * q3;
    RHS.row(3) = -a * q4;

    MatrixXf displacements = coefficients.llt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        int vertexIndex = indices[i];
        Vector3f displacement = displacements.row(i);
        float k = 1.0f;
        if (!mesh->isRigidBody) k -= pow(1.0f - params.bendFactor, 1.0f / params.solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}

void StaticCollisionConstraint::project(Params params) {
    Vector3f p = mesh->estimatePositions[indices[0]];

    Vector3f pointToPosition = p - position;
    pointToPosition.normalize();

    // Check if constraint is already satisfied
    if (pointToPosition.dot(normal) >= 0.0f) return;

    float a = (p - position).dot(normal);
    Vector3f b = (p - position) / ((p - position).norm());

    Vector3f displacement = a * b;

    mesh->estimatePositions[indices[0]] += displacement;
}

void TriangleCollisionConstraint::project(Params params) {
    Vector3f q = mesh->estimatePositions[indices[0]];
    Vector3f p1 = mesh->estimatePositions[indices[1]];
    Vector3f p2 = mesh->estimatePositions[indices[2]];
    Vector3f p3 = mesh->estimatePositions[indices[3]];

    // Check if constraint is already satisfied
    Vector3f n = (p2 - p1).cross(p3 - p1);
    n /= n.norm();

//    cout << "~" << normal << "vs" << n << "~~~" << endl << endl;

    normal = n;

    Vector3f qToP1 = q - p1;
    qToP1.normalize();

    if (qToP1.dot(n) - height >= 0.0f) return;

    float a = (q - p1).dot(n) - height;
    Vector3f b = n;

    Vector3f displacement = a * b;

    mesh->estimatePositions[indices[0]] -= displacement;

//    MatrixXf RHS = MatrixXf::Zero(1, 3);
//
//    Vector3f p2Xp3 = p2.cross(p3);
//
//    // Compute normals
//    Vector3f n1 = p2Xp3 / p2Xp3.norm();
//
//    // Compute coefficient matrix
//    coefficients.resize(1, 1);
//    coefficients.setZero();
//
//    float denominator = 0.0f;
//    denominator += mesh->inverseMasses[indices[0]];
//
//    float wi = mesh->inverseMasses[indices[0]];
//    coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
//
//    float qSum = q1.squaredNorm() + q2.squaredNorm() + q3.squaredNorm() + q4.squaredNorm();
//
//    coefficients.coeffRef(0, 0) *= qSum;
//
//    float a = (q - p1).dot(n) - height;
//    RHS.row(0) = -a * q1;
//
//    MatrixXf displacements = coefficients.llt().solve(RHS);
//
//    Vector3f displacement = displacements.row(0);
//    mesh->estimatePositions[indices[0]] += displacement;
}
