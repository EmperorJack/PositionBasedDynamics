#version 330 core

in vec3 position_worldspace;
in vec3 normal_cameraspace;
in vec3 eyeDirection_cameraspace;
in vec3 lightDirection_cameraspace;

out vec4 color;

uniform vec3 materialColour = vec3(1.0, 1.0, 1.0);
uniform vec3 lightPosition;

void main() {
    vec3 lightColor = vec3(1, 1, 1);
	float lightPower = 50.0f;

	float distance = length(lightPosition - position_worldspace);

	vec3 n = normalize(normal_cameraspace);
	vec3 l = normalize(lightDirection_cameraspace);

	float cosTheta = clamp(dot(n, l), 0, 1);

    vec3 ambient = 0.15 * materialColour;
	vec3 diffuse = (materialColour * lightColor * lightPower * cosTheta) / (distance * distance);

//    color = materialColour;
    color = vec4(ambient + diffuse, 1.0);
}
