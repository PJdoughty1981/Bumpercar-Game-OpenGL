/**
 * static_model.vert - Vertex shader for static environment models
 * 
 * I created this shader for rendering static environment elements like
 * the Ferris wheel and trees. It's similar to the model_loading shader
 * but includes texture coordinate handling. The shader calculates all the
 * necessary outputs for realistic lighting and shadows.
 * 
 * My static models need to cast and receive shadows just like dynamic
 * elements, which is why I include FragPosLightSpace calculation for
 * shadow mapping.
 */
#version 430 core
layout (location = 0) in vec3 aPos;        // Vertex position
layout (location = 1) in vec3 aNormal;      // Vertex normal
layout (location = 2) in vec2 aTexCoords;   // Texture coordinates

out vec3 FragPos;               // Fragment position in world space
out vec3 Normal;                // Normal vector in world space
out vec2 TexCoords;             // Texture coordinates
out vec4 FragPosLightSpace;     // Position in light space for shadow mapping

uniform mat4 model;             // Model transformation
uniform mat4 view;              // View transformation
uniform mat4 projection;        // Projection transformation
uniform mat4 lightSpaceMatrix;  // Transformation to light space for shadows

void main() {
    // Calculate world position
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = vec3(worldPos);
    
    // Transform normal to world space
    Normal = mat3(transpose(inverse(model))) * aNormal;
    
    // Pass texture coordinates to fragment shader unchanged
    TexCoords = aTexCoords;
    
    // Calculate position in light space for shadow mapping
    FragPosLightSpace = lightSpaceMatrix * worldPos;
    
    // Calculate final clip space position
    gl_Position = projection * view * worldPos;
}