/**
 * text.frag - Fragment shader for text rendering
 * 
 * This fragment shader renders text from a font texture atlas created with FreeType.
 * The texture contains grayscale glyphs where the intensity represents the coverage
 * of each pixel by the font shape. I use this single-channel value as the alpha
 * of the final output color, which allows for proper transparency and antialiasing.
 * 
 * The shader takes a uniform text color and applies it to the font shape,
 * with transparency controlled by the sampled texture value. This allows me to
 * change text colors easily without needing different textures.
 */
#version 430 core
in vec2 TexCoords;
out vec4 color;

uniform sampler2D text;    // Font atlas texture
uniform vec3 textColor;    // Color to use for the text

void main()
{    
    // Sample the font texture - the red channel contains the coverage value
    // (for FreeType, which uses a single-channel grayscale texture)
    vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
    
    // Output color uses the provided color for RGB and the sampled value for A
    // This creates properly antialiased colored text with transparency
    color = vec4(textColor, sampled.a);
}