#version 330 core

out vec4 color;

uniform vec4 fillColor = vec4(1.0, 0.0, 0.0, 0.0);

void main() {
    color = fillColor;
}
