#version 330 core
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

out vec3 position_worldspace;
out vec3 normal_cameraspace;
out vec3 eyeDirection_cameraspace;
out vec3 lightDirection_cameraspace;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 lightPosition;

void main() {
    gl_Position = projection * view * model * vec4(vertexPosition_modelspace, 1);

    position_worldspace = (model * vec4(vertexPosition_modelspace, 1)).xyz;

    vec3 vertexPosition_cameraspace = (view * model * vec4(vertexPosition_modelspace, 1)).xyz;
    eyeDirection_cameraspace = vec3(0, 0, 0) - vertexPosition_cameraspace;

    vec3 lightPosition_cameraspace = (view * vec4(lightPosition, 1)).xyz;
    lightDirection_cameraspace = lightPosition_cameraspace + eyeDirection_cameraspace;

    normal_cameraspace = (view * model * vec4(vertexNormal_modelspace, 0)).xyz;
}