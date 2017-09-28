//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_CONSTRAINT_HPP
#define POSITIONBASEDDYNAMICS_CONSTRAINT_HPP

#include <vector>
#include <Eigen>
#include <mesh.hpp>

using namespace std;
using namespace Eigen;

// Forward declaration
class Mesh;

class Constraint {

public:
    Constraint(Mesh* mesh, int cardinality) :
            mesh(mesh), cardinality(cardinality) {}
    virtual void preCompute() {}
    virtual void project(int solverIterations) {}

    int cardinality;
    vector<int> indices;
    Mesh* mesh;
    MatrixXf coefficients;

};

class FixedConstraint : public Constraint {

public:
    FixedConstraint(Mesh* mesh, int cardinality, Vector3f target) :
            Constraint(mesh, cardinality), target(target) {}
    void project(int solverIterations);

private:
    Vector3f target;

};

class DistanceConstraint : public Constraint {

public:
    DistanceConstraint(Mesh* mesh, int cardinality, float stiffness, float distance) :
            Constraint(mesh, cardinality), stiffness(stiffness), distance(distance) {}
    void preCompute();
    void project(int solverIterations);

private:
    float stiffness;
    float distance;

};

class BendConstraint : public Constraint {

public:
    BendConstraint(Mesh* mesh, int cardinality, float stiffness, float angle) :
            Constraint(mesh, cardinality), stiffness(stiffness), angle(angle) {}
    void preCompute();
    void project(int solverIterations);

private:
    float stiffness;
    float angle;

};

// Constraint building
void buildEdgeConstraints(Mesh* mesh, float stiffness);
FixedConstraint* buildFixedConstraint(Mesh* mesh, int index, Vector3f target);
DistanceConstraint* buildDistanceConstraint(Mesh* mesh, int indexA, int indexB, float distance, float stiffness);
BendConstraint* buildBendConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float stiffness, float angle);

#endif //POSITIONBASEDDYNAMICS_CONSTRAINT_HPP
