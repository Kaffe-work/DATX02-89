#version 330 core
layout(location = 0) in vec3 aPos;

out vec4 ourColor;

uniform mat4 projection;
uniform vec3 color;

void main()
{
	gl_Position = projection * vec4(aPos, 1.0);
	ourColor = vec4(color, 1.0);
}