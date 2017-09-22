//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_CAMERA_HPP
#define POSITIONBASEDDYNAMICS_CAMERA_HPP

#include <Eigen>

class Camera {

public:
    void setPerspective(float angleOfView, float aspectRatio, float zNear, float zFar) ;
    void lookAt(Eigen::Vector3f position, Eigen::Vector3f target, Eigen::Vector3f up);

    Eigen::Matrix4f projectionMatrix;
    Eigen::Matrix4f viewMatrix;

};

#endif //POSITIONBASEDDYNAMICS_CAMERA_HPP
