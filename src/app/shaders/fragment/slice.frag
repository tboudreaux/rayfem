#version 330

in vec3 fragPos;
in vec4 fragColor;

out vec4 finalColor;

// Uniforms sent from C++
uniform vec3 planeNormal;
uniform vec3 planePoint;

void main() {
    vec3 v = fragPos - planePoint;
    float side = dot(v, planeNormal);

    if (side > 0.0) {
        discard;
    }

    finalColor = fragColor;
}