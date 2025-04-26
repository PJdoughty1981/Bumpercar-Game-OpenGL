/**
 * model_loading.vert - Vertex shader for dynamic models
 * 
 * I designed this shader specifically for the bumper cars, which need
 * shadow casting and detailed lighting. It transforms vertices and computes
 * several outputs needed for advanced rendering effects:
 * 
 * - Standard position transformation through MVP matrices
 * - Fragment position in world space for lighting calculations
 * - Normal vectors transformed to world space
 * - Position in light space for shadow mapping
 * 
 * This shader passes all this information to the fragment shader to enable
 * proper lighting, shadowing, and material rendering for the cars.
 */
#version 430 core
layout (location = 0) in vec3 aPos;       // Vertex position
layout (location = 1) in vec3 aNormal;     // Vertex normal

out vec3 FragPos;              // Fragment position in world space
out vec3 Normal;               // Normal vector in world space
out vec4 FragPosLightSpace;    // Position in light space for shadow mapping

uniform mat4 model;            // Model transformation
uniform mat4 view;             // View transformation  
uniform mat4 projection;       // Projection transformation
uniform mat4 lightSpaceMatrix; // Transformation to light space for shadows

void main() {
    // Calculate world position for lighting calculations
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = vec3(worldPos);
    
    // Transform normal to world space
    // Using transpose of inverse to handle non-uniform scaling
    Normal = mat3(transpose(inverse(model))) * aNormal;
    
    // Calculate position in light space for shadow mapping
    FragPosLightSpace = lightSpaceMatrix * worldPos;
    
    // Calculate final clip space position
    gl_Position = projection * view * worldPos;
}