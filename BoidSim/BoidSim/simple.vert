#version 330 core
layout (location = 0) in vec3 aPos;

out vec3 ourColor;

uniform mat4 projection;

void main()
{
    gl_Position =  projection * vec4(aPos, 1.0);
    ourColor = vec3(0.2f, 0.2f, 0.2f);
}