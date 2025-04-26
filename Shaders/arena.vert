/**
 * arena.vert - Vertex shader for the bumper car arena
 * 
 * This shader handles the vertex transformations for the arena floor, walls,
 * and surrounding area. It's designed to work with arena.frag for complete
 * rendering of the environment. Besides standard transformation, it also 
 * calculates variables needed for lighting and shadow mapping.
 */
#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
layout (location = 2) in vec3 aNormal;

out vec2 TexCoords;
out vec3 Normal;
out vec3 FragPos;
out vec4 FragPosLightSpace;  // For shadow mapping calculations

// Transformation matrices
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;  // For transforming into light's view for shadow mapping

/**
 * Main vertex processing function
 * 
 * Transforms the vertex position through model, view, and projection matrices,
 * and calculates lighting-related outputs like normals, fragment position, etc.
 * I also compute the position in light space for shadow mapping.
 */
void main()
{
    // Calculate fragment position in world space
    FragPos = vec3(model * vec4(aPos, 1.0));
    
    // Calculate normal in world space
    // Using the transpose of the inverse to handle non-uniform scaling properly
    Normal = mat3(transpose(inverse(model))) * aNormal;
    
    // Pass texture coordinates unchanged
    TexCoords = aTexCoords;
    
    // Calculate position in light space for shadow mapping (main light only)
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);
    
    // Calculate final position
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}