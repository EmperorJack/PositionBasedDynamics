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
	float lightIntensity = 50.0;
	vec3 specularColor = vec3(1, 1, 1);
	int specularExponent = 5;

	float distance = length(lightPosition - position_worldspace);

	vec3 n = normalize(normal_cameraspace);
	vec3 l = normalize(lightDirection_cameraspace);
	float cosTheta = clamp(dot(n, l), 0, 1);

    vec3 e = normalize(eyeDirection_cameraspace);
    vec3 r = reflect(-l, n);
    float cosAlpha = clamp(dot(e, r), 0, 1);

    vec3 ambient = 0.1 * materialColour;
	vec3 diffuse = (materialColour * lightColor * lightIntensity * cosTheta) / (distance * distance);
    vec3 specular = (specularColor * lightColor * lightIntensity * pow(cosAlpha, specularExponent)) / (distance * distance);

    color = vec4(ambient + diffuse + specular, 1.0);
}
