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

void DistanceConstraint::preCompute() {
    coefficients.resize(cardinality, cardinality);

    float denominator = 0.0f;
    for (int i = 0; i < cardinality; i++) {
        denominator += mesh->inverseMasses[indices[i]];
    }

    for (int i = 0; i < cardinality; i++) {
        float wi = mesh->inverseMasses[indices[i]];
        coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
    }
}

void FixedConstraint::project(int solverIterations) {
    mesh->estimatePositions[indices[0]] = target;
}

void DistanceConstraint::project(int solverIterations) {
    MatrixXf RHS(cardinality, 3);

    Vector3f p1 = mesh->estimatePositions[indices[0]];
    Vector3f p2 = mesh->estimatePositions[indices[1]];

    float a = (p1 - p2).norm() - distance;
    Vector3f b = (p1 - p2) / ((p1 - p2).norm() + EPSILON);

    RHS.row(0) = -a * b;
    RHS.row(1) = a * b;

    MatrixXf displacements = coefficients.ldlt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        int vertexIndex = indices[i];
        Vector3f displacement = displacements.row(i);
        float k = 1.0f - pow(1.0f - stiffness, 1.0f / solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}

void BendConstraint::preCompute() {
    coefficients.resize(cardinality, cardinality);

    float denominator = 0.0f;
    for (int i = 0; i < cardinality; i++) {
        denominator += mesh->inverseMasses[indices[i]];
    }

    for (int i = 0; i < cardinality; i++) {
        float wi = mesh->inverseMasses[indices[i]];
        coefficients.coeffRef(i, i) = 1.0f / (wi / denominator);
    }
}

void BendConstraint::project(int solverIterations) {
    MatrixXf RHS(cardinality, 3);

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
    float qSum = q1.squaredNorm() + q2.squaredNorm() + q3.squaredNorm() + q4.squaredNorm();
    for (int i = 0; i < cardinality; i++) {
        coefficients.coeffRef(i, i) *= qSum;
    }

    float a = sqrtf(1.0f - d * d) * (acosf(d) - angle);
    RHS.row(0) = -a * q1;
    RHS.row(1) = -a * q2;
    RHS.row(2) = -a * q3;
    RHS.row(3) = -a * q4;

    MatrixXf displacements = coefficients.ldlt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        int vertexIndex = indices[i];
        Vector3f displacement = displacements.row(i);
        float k = 1.0f - pow(1.0f - stiffness, 1.0f / solverIterations);
        mesh->estimatePositions[vertexIndex] += k * displacement;
    }
}
