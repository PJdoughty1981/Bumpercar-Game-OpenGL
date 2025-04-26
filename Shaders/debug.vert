/**
 * debug.vert - Vertex shader for debug visualization
 * 
 * I created this shader for rendering debug elements like collision boxes,
 * position markers, and other visual aids that help me understand the state
 * of the physics system while developing. These visualizations are invaluable
 * for spotting issues with collision detection, rotation points, and other
 * geometric calculations.
 * 
 * The shader simply transforms vertices from model space to clip space
 * using the standard model-view-projection matrix chain, and passes
 * the vertex color to the fragment shader.
 */
// debug.vert
#version 430 core
layout (location = 0) in vec3 aPos;    // Vertex position
layout (location = 1) in vec4 aColor;   // Vertex color

uniform mat4 model;        // Model transformation
uniform mat4 view;         // View transformation
uniform mat4 projection;   // Projection transformation

out vec4 Color;            // Output color to fragment shader

void main() {
    // Transform position to clip space using the full MVP matrix chain
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    
    // Pass color to fragment shader unchanged
    Color = aColor;
}