#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec4 ourColor;

uniform mat4 projection;
uniform vec3 color;
uniform vec3 lightColor;
uniform vec3 lightPos;

void main()
{
	// ambient
	vec3 ambient = 0.1 * lightColor;

	// diffuse 
    vec3 norm = normalize(aNormal);
    vec3 lightDir = normalize(lightPos - aPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

	vec3 result = (ambient + diffuse) * color;

	ourColor = vec4(result, 1.0);

	gl_Position = projection * vec4(aPos, 1.0);
}