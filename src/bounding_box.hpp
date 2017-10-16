//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_BOUNDINGBOX_HPP
#define POSITIONBASEDDYNAMICS_BOUNDINGBOX_HPP

#include <Eigen>

using namespace Eigen;

class BoundingBox {

public:
    bool intersect(Vector3f rayOrigin, Vector3f rayDirection);

    float xMin, xMax;
    float yMin, yMax;
    float zMin, zMax;

};

#endif //POSITIONBASEDDYNAMICS_BOUNDINGBOX_HPP
