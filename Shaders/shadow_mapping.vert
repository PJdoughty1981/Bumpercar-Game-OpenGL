/**
 * shadow_mapping.vert - Vertex shader for shadow depth map generation
 * 
 * This simple but crucial shader is used for the shadow mapping pass
 * in my rendering pipeline. It transforms vertices into light space
 * to generate a depth map from the light's perspective. This depth map
 * is later used to determine which fragments are in shadow.
 * 
 * Unlike rendering shaders, this one doesn't need to output colors or
 * calculate lighting - it only needs to accurately position vertices so
 * the depth buffer captures the correct distance from the light source.
 */
// shadow_mapping.vert
#version 430 core
layout (location = 0) in vec3 aPos;  // Vertex position

uniform mat4 lightSpaceMatrix;  // Combined light projection and view matrix
uniform mat4 model;             // Model transformation matrix

void main()
{
    // Transform vertex from model space directly to light's clip space
    // This creates the perspective from which shadows will be determined
    gl_Position = lightSpaceMatrix * model * vec4(aPos, 1.0);
}