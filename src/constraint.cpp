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

        mesh->constraints.push_back(buildDistanceConstraint(
                mesh, v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm(), stiffness
        ));
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
    return constraint;
}

void FixedConstraint::project(int solverIterations) {
    mesh->estimatePositions[indices[0]] = target;
}

void DistanceConstraint::project(int solverIterations) {
    MatrixXf coefficients(cardinality, cardinality);

    float w1 = mesh->inverseMasses[indices[0]];
    float w2 = mesh->inverseMasses[indices[1]];
    coefficients.coeffRef(0, 0) = 1.0f / (w1 / (w1 + w2));
    coefficients.coeffRef(1, 1) = 1.0f / (w2 / (w1 + w2));

    MatrixXf RHS(cardinality, 3);

    Vector3f p1 = mesh->estimatePositions[indices[0]];
    Vector3f p2 = mesh->estimatePositions[indices[1]];

    float a = ((p1 - p2).norm() - distance);
    Vector3f b = (p1 - p2) / ((p1 - p2).norm() + EPSILON);

    RHS.row(0) = -a * b;
    RHS.row(1) = a * b;

    MatrixXf displacements = coefficients.ldlt().solve(RHS);

    for (int i = 0; i < cardinality; i++) {
        int vertexIndex = indices[i];
        Vector3f displacement = displacements.row(i);
        float s = 1.0f - pow(1.0f - stiffness, 1.0f / solverIterations);
        mesh->estimatePositions[vertexIndex] += s * displacement;
    }
}