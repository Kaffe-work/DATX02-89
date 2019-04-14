#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

out vec4 ourColor;

uniform mat4 projection;

void main()
{
	gl_Position = projection * vec4(aPos, 1.0);
	ourColor = length(aColor) > 3 ? vec4(aColor, 0.0f) : vec4(aColor, 1.0f);
}