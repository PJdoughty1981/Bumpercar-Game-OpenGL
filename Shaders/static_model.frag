/**
 * static_model.frag - Fragment shader for static environment models
 * 
 * This fragment shader handles the lighting and shading for static environment
 * models like the Ferris wheel and trees. It's very similar to the model_loading
 * shader but is specialized for static objects that don't need as many dynamic
 * updates. I've tuned the lighting parameters specifically for my outdoor
 * daytime environment to make these elements look natural in the scene.
 * 
 * The lighting model includes ambient, diffuse, and specular components,
 * plus special effects like rim lighting and saturation adjustment to make
 * the objects stand out visually.
 */
#version 430 core
out vec4 FragColor;
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

/**
 * Material properties structure
 * 
 * Defines how a surface responds to light.
 */
struct Material {
    vec3 diffuse;     // Base/diffuse color
    vec3 specular;    // Specular/reflection color
    float shininess;  // Surface smoothness
    float opacity;    // Transparency (1.0 = opaque)
};

uniform Material material;  // Material properties
uniform vec3 viewPos;       // Camera position
uniform vec3 lightPos;      // Main light position
uniform vec3 lightColor;    // Main light color
uniform float lightIntensity = 1.0; // Light intensity
uniform sampler2D texture_diffuse1; // Diffuse texture
uniform int material_has_texture;   // Whether material has a texture

void main() {
    vec3 normal = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 viewDir = normalize(viewPos - FragPos);
    
    // Get base color from texture if available, otherwise use material diffuse color
    vec3 baseColor;
    float alpha = material.opacity;
    
    if (material_has_texture == 1) {
        vec4 texColor = texture(texture_diffuse1, TexCoords);
        
        // Use texture alpha
        alpha = texColor.a;
        
        // Skip fully transparent pixels
        if (alpha < 0.1) {
            discard;
        }
        
        baseColor = texColor.rgb;
    } else {
        baseColor = material.diffuse;
    }
    
    // Balanced ambient for natural lighting
    float ambientStrength = 0.4;
    vec3 ambient = ambientStrength * lightColor * baseColor;
    
    // Natural diffuse lighting
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor * baseColor;
    
    // Specular highlights
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = spec * lightColor * material.specular * 0.5; // Reduced specular
    
    // Combine all lighting components
    vec3 result = ambient + diffuse + specular;
    
    // Apply light intensity with a reasonable minimum
    result *= max(lightIntensity, 0.8);
    
    // Subtle tone mapping to prevent oversaturation
    result = result / (result + vec3(0.8));
    
    // Maintain vivid colors without excessive gamma correction
    float saturation = 1.1; // Slightly boost saturation
    vec3 luminance = vec3(dot(result, vec3(0.299, 0.587, 0.114)));
    result = mix(luminance, result, saturation);
    
    FragColor = vec4(result, alpha);
}