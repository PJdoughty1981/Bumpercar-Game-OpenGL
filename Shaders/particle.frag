/**
 * particle.frag - Fragment shader for particle rendering
 * 
 * This shader creates attractive circular particles with soft edges for my
 * electric spark effects. It implements:
 * - Circular shape with alpha discard for clean edges
 * - Smooth falloff at edges for a soft glow effect
 * - Alpha blending based on both particle color and distance from center
 * 
 * This shader works with particle.vert and receives the particle color
 * from that shader. The gl_PointCoord built-in variable provides the
 * coordinates within the particle quad.
 */
// particle.frag
#version 430 core

// Input from vertex shader
in vec4 Color;

// Output fragment color
out vec4 FragColor;

void main() {
    // Create a circular particle by measuring distance from center
    // gl_PointCoord gives us coordinates from (0,0) to (1,1) across the particle quad
    // We transform to (-1,-1) to (1,1) to make a centered circle
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    
    // Calculate squared distance from center (faster than using length())
    float circle = dot(circCoord, circCoord);
    
    // Discard fragments outside the circle to create a round particle
    if (circle > 1.0) {
        discard;
    }
    
    // Add soft glow effect by fading alpha toward the edges
    // The smoothstep creates a smooth transition from 0.5 to 1.0 radius
    float alpha = Color.a * (1.0 - smoothstep(0.5, 1.0, circle));
    
    // Output final color with adjusted alpha
    FragColor = vec4(Color.rgb, alpha);
}