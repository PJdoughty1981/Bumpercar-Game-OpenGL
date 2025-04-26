/**
 * skybox.vert - Vertex shader for skybox rendering
 * 
 * This shader handles cube skybox rendering, creating the illusion of a distant
 * environment surrounding the game arena. Key features:
 * 
 * - Uses view matrix with rotation only (no translation) to keep skybox centered
 * - Passes vertex positions as texture coordinates for cubemap sampling
 * - Uses a depth hack (.xyww) to ensure skybox is rendered behind everything else
 * 
 * The skybox adds tremendously to the visual appeal of my game by providing
 * a convincing backdrop that makes the arena feel like part of a larger world.
 */
#version 430 core
layout (location = 0) in vec3 aPos;  // Cube vertex positions

out vec3 TexCoords;  // Will be used for cubemap texture lookup

uniform mat4 projection;  // Projection matrix
uniform mat4 view;        // View (camera) matrix

void main()
{
    // Pass vertex positions directly as texture coordinates
    // This works because both are in the range [-1,1]
    TexCoords = aPos;
    
    // Remove translation from view matrix by converting to 3x3 then back to 4x4
    // This ensures the skybox moves with camera rotation but not translation
    vec4 pos = projection * mat4(mat3(view)) * vec4(aPos, 1.0);
    
    // Depth hack: Set z equal to w to ensure maximum depth value of 1.0
    // This guarantees the skybox renders behind all other objects
    gl_Position = pos.xyww;
}