//
// Created by Jack Purvis
//

#include <iostream>
using namespace std;

#include <bounding_box.hpp>

bool BoundingBox::intersect(Vector3f rayOrigin, Vector3f rayDirection) {

    // Compute x slab intersection interval
    float xtNear = (xMin - rayOrigin[0]) / rayDirection[0];
    float xtFar = (xMax - rayOrigin[0]) / rayDirection[0];

    if (xtNear > xtFar) std::swap(xtNear, xtFar);

    // Compute y slab intersection interval
    float ytNear = (yMin - rayOrigin[1]) / rayDirection[1];
    float ytFar = (yMax - rayOrigin[1]) / rayDirection[1];

    if (ytNear > ytFar) std::swap(ytNear, ytFar);

    // Compute z slab intersection interval
    float ztNear = (zMin - rayOrigin[2]) / rayDirection[2];
    float ztFar = (zMax - rayOrigin[2]) / rayDirection[2];

    if (ztNear > ztFar) std::swap(ztNear, ztFar);

    // Check x and y overlap
    bool xyOverlap = xtNear <= ytFar && ytNear <= xtFar;

    // Check x and z overlap
    bool xzOverlap = xtNear <= ztFar && ztNear <= xtFar;

    // Check y and z overlap
    bool yzOverlap = ytNear <= ztFar && ztNear <= ytFar;

    return xyOverlap && xzOverlap && yzOverlap;
}
