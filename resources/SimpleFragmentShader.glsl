#version 330 core

out vec3 color;

uniform vec4 fillColor = vec4(0.0, 0.0, 0.0, 0.0);

void main() {
    color = fillColor.rgb;
}
