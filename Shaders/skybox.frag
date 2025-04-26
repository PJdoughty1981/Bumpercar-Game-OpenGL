/**
 * skybox.frag - Fragment shader for skybox rendering
 * 
 * A simple fragment shader that samples from a cubemap texture using the
 * coordinates passed from the vertex shader. The result creates a convincing
 * illusion of a distant environment surrounding my bumper car arena.
 * 
 * The skybox uses seamless texture mapping across the six faces of a cube,
 * with different images for each direction (right, left, top, bottom, back, front).
 */
#version 430 core
out vec4 FragColor;  // Output fragment color

in vec3 TexCoords;   // Input texture coordinates from vertex shader

uniform samplerCube skybox;  // Cubemap texture sampler

void main()
{    
    // Sample the cubemap using the 3D texture coordinates
    // The GPU automatically picks the right face based on the direction
    FragColor = texture(skybox, TexCoords);
}