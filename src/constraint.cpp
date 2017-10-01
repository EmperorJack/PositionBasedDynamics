//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>

void buildEdgeConstraints(Mesh* mesh, float stiffness) {
    for (Edge edge : mesh->edges) {
        int v0 = edge.v[0].p;
        int v1 = edge.v[1].p;

        mesh->constraints.push_back(buildDistanceConstraint(mesh, v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm(), stiffness));
    }
}

void buildBendConstraints(Mesh* mesh, float stiffness) {
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

        mesh->constraints.push_back(buildBendConstraint(mesh, p1, p2, p3, p4, stiffness, acosf(d)));
    }
}

FixedConstraint* buildFixedConstraint(Mesh* mesh, int index, Vector3f target) {
    mesh->inverseMasses[index] = EPSILON;

    FixedConstraint* constraint = new FixedConstraint(mesh, 1, target);
    constraint->indices.push_back(index);
    return constraint;
}

DistanceConstraint* buildDistanceConstraint(Mesh* mesh, int indexA, int indexB, float distance, float stiffness) {
    DistanceConstraint* constraint = new DistanceConstraint(mesh, 2, stiffness, distance);
    constraint->indices.push_back(indexA);
    constraint->indices.push_back(indexB);

    constraint->preCompute();
    return constraint;
}

BendConstraint* buildBendConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float stiffness, float angle) {
    BendConstraint* constraint = new BendConstraint(mesh, 4, stiffness, angle);
    constraint->indices.push_back(indexA);
    constraint->indices.push_back(indexB);
    constraint->indices.push_back(indexC);
    constraint->indices.push_back(indexD);

    constraint->preCompute();
    return constraint;
}

void FixedConstraint::project(int solverIterations) {
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

void DistanceConstraint::project(int solverIterations) {
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
        float k = 1.0f - pow(1.0f - stiffness, 1.0f / solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}

void BendConstraint::preCompute() {
//    coefficients.resize(cardinality, cardinality);
//    coefficients.setZero();
//
//    float denominator = 0.0f;
//    for (int i = 0; i < cardinality; i++) {
//        denominator += mesh->inverseMasses[indices[i]];
//    }
//
//    for (int i = 0; i < cardinality; i++) {
//        float wi = mesh->inverseMasses[indices[i]];
//        coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
//    }
}

void BendConstraint::project(int solverIterations) {
    MatrixXf RHS = MatrixXf::Zero(cardinality, 3);

    Vector3f p1 = mesh->estimatePositions[indices[0]];
    Vector3f p2 = mesh->estimatePositions[indices[1]];
    Vector3f p3 = mesh->estimatePositions[indices[2]];
    Vector3f p4 = mesh->estimatePositions[indices[3]];

    // Compute normals
    Vector3f n1 = p2.cross(p3) / p2.cross(p3).norm();
    Vector3f n2 = p2.cross(p4) / p2.cross(p4).norm();
    float d = n1.dot(n2); // n1.transpose() * n2

    Vector3f q3 = (p2.cross(n2) + d * n1.cross(p2)) / (p2.cross(p3).norm());
    Vector3f q4 = (p2.cross(n1) + d * n2.cross(p2)) / (p2.cross(p4).norm());
    Vector3f q2 = -(p3.cross(n2) + d * n1.cross(p3)) / (p2.cross(p3).norm()) - (p4.cross(n1) + d * n2.cross(p4)) / (p2.cross(p4).norm());
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
        float k = 1.0f - pow(1.0f - stiffness, 1.0f / solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}
