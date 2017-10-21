//
// Created Jack Purvis
//

#define _USE_MATH_DEFINES
#include <cmath>
#include <camera.hpp>

void Camera::setPerspective(float angleOfView, float aspectRatio, float zNear, float zFar) {
    float invTan = 1.0f / (float) tan(angleOfView * 0.5 * M_PI / 180);

    projectionMatrix(0, 0) = invTan / aspectRatio;
    projectionMatrix(1, 1) = invTan;
    projectionMatrix(2, 2) = -(zNear + zFar) / (zFar - zNear);
    projectionMatrix(3, 2) = -1.0f;
    projectionMatrix(2, 3) = -2.0f * zNear * zFar / (zFar - zNear);
    projectionMatrix(3, 3) = 0.0f;
}

void Camera::lookAt(Eigen::Vector3f position, Eigen::Vector3f target, Eigen::Vector3f up) {
    Eigen::Matrix3f R;
    R.col(2) = (position - target).normalized();
    R.col(0) = up.cross(R.col(2)).normalized();
    R.col(1) = R.col(2).cross(R.col(0));
    viewMatrix.topLeftCorner<3, 3>() = R.transpose();
    viewMatrix.topRightCorner<3, 1>() = -R.transpose() * position;
    viewMatrix(3, 3) = 1.0f;
}
