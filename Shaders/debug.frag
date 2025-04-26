/**
 * debug.frag - Fragment shader for debug visualization
 * 
 * A simple fragment shader that outputs the color received from the vertex
 * shader without any modifications. I use this for rendering debug lines,
 * collision boxes, and other visual aids that help me understand what's
 * happening in the physics system.
 * 
 * The debug rendering can be toggled on/off with the 'C' key, which is
 * extremely useful when testing collision detection and physics calculations.
 */
// debug.frag
#version 430 core
out vec4 FragColor;    // Final output color

in vec4 Color;         // Input color from vertex shader

void main() {
    // Simply output the color passed from the vertex shader
    FragColor = Color;
}