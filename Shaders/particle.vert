/**
 * particle.vert - Vertex shader for particle rendering
 * 
 * This shader transforms particles for my electric spark effects from the car antennas.
 * It supports position, color, and dynamic sizing based on distance from camera.
 * The point size scaling ensures particles look consistent in size regardless of 
 * distance from the viewing camera.
 */
// particle.vert
#version 430 core

// Input vertex attributes
layout (location = 0) in vec3 aPos;    // Particle position in world space
layout (location = 1) in vec4 aColor;   // RGBA color including alpha
layout (location = 2) in float aSize;   // Base size in pixels

// Output to fragment shader
out vec4 Color;  // Pass color to fragment shader

// Transformation matrices
uniform mat4 projection;  // Camera projection matrix
uniform mat4 view;        // Camera view matrix
uniform mat4 model;       // Model matrix (usually identity for particles)

void main() {
    // Transform particle position to clip space
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    
    // Scale point size based on distance from camera
    // This makes particles appear consistent in size regardless of distance
    gl_PointSize = aSize / gl_Position.w;
    
    // Pass color directly to fragment shader
    Color = aColor;
}