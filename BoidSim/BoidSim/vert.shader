#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
layout(location = 2) in vec3 aNormal;

out vec3 ourColor;

uniform vec3 bgColor;
uniform vec3 cameraPos;

uniform vec3 color;
uniform mat4 projection;

void main()
{
	float a = length(aPos);
	gl_Position = projection * vec4(aPos, 1.0);
	ourColor = aColor *((1000.0 - a) / 1000.0) + bgColor*(a / 1000.0);
}