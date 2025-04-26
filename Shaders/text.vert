/**
 * text.vert - Vertex shader for text rendering
 * 
 * This is my vertex shader for rendering all the on-screen text elements
 * like the key bindings menu and UI information. It's designed to work with
 * FreeType-generated font textures and the accompanying fragment shader.
 * 
 * The shader takes a special vertex format that includes both position and
 * texture coordinates for the font glyphs. The position is transformed to
 * screen space using an orthographic projection matrix, while the texture
 * coordinates are passed unchanged to the fragment shader.
 */
#version 430 core
layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
out vec2 TexCoords;

uniform mat4 projection; // Orthographic projection matrix

void main()
{
    // Extract position and texture coordinates from the combined input
    // First 2 components (xy) are position, last 2 components (zw) are texture coordinates
    
    // Transform position to clip space using orthographic projection
    // This allows text to be positioned in 2D screen coordinates
    gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
    
    // Pass texture coordinates to fragment shader
    TexCoords = vertex.zw;
}