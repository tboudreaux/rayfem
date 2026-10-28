#version 330

in vec3 vertexPosition;
in vec3 vertexTexCoord;
in vec4 vertexColor;

out vec3 fragPos;
out vec4 fragColor;

uniform mat4 mvp;
uniform mat4 matModel;

void main() {
    fragPos = vertexPosition;
    fragColor = vertexColor;

    gl_Position = mvp * vec4(vertexPosition, 1.0);
}