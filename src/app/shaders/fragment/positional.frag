#version 330

in vec3 fragPos;
out vec4 finalColor;

void main() {
    // Color based only on the fragment's position
    finalColor = vec4(fragPos * 0.5 + 0.5, 1.0); // Normalize to [0,1] range
}