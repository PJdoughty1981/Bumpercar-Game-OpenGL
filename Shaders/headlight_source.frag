/**
 * headlight_source.frag - Fragment shader for car headlight visuals
 * 
 * This shader creates the glowing headlight effect for my bumper cars. It handles:
 * - Base light color with configurable intensity
 * - Rim lighting effect that makes the edges of the headlight glow more intensely
 * - HDR-like tone mapping for a realistic bloom effect
 * 
 * The shader takes the light color and intensity as uniforms and calculates
 * a rim effect based on the view angle to create a more realistic glow.
 */
// headlight_source.frag
#version 430 core

// Output final fragment color
out vec4 FragColor;

// Input from vertex shader
in vec3 FragPos;   // Fragment position in world space
in vec3 Normal;    // Surface normal

// Uniforms for configuring the light
uniform vec3 lightColor;   // Base color of the headlight
uniform float intensity;   // Overall brightness multiplier

void main() {
    // Normalize vectors for lighting calculations
    vec3 normal = normalize(Normal);
    vec3 viewDir = normalize(-FragPos);
    
    // Calculate rim factor - stronger when viewing at an angle
    // This creates a stronger glow at the edges of the headlight
    float rim = pow(1.0 - max(dot(normal, viewDir), 0.0), 2.0);
    
    // Combine base light color with rim effect
    // Adding the rim effect creates a glowing halo around the edges
    vec3 finalColor = lightColor * (1.0 + rim) * intensity;
    
    // Apply a simple tone mapping operation to create a bloom-like effect
    // This prevents harsh cutoffs in bright areas, creating a more realistic glow
    finalColor = finalColor / (finalColor + vec3(1.0));
    
    // Output final color with full opacity
    FragColor = vec4(finalColor, 1.0);
}