/**
 * model_loading.frag - Fragment shader for dynamic models
 * 
 * This is a key shader in my game that renders the bumper cars with
 * realistic lighting, materials, and shadow effects. I spent a lot of
 * time tuning the lighting parameters to get a visually appealing result
 * that works well in the daytime setting of the game.
 * 
 * Features:
 * - Phong lighting model with ambient, diffuse, and specular components
 * - Material system with configurable properties
 * - Shadow mapping for realistic shadows
 * - Tone mapping for better highlight preservation
 * - Saturation adjustment for more vibrant colors
 * - Rim lighting for better edge definition
 */
#version 430 core
out vec4 FragColor;
in vec3 FragPos;
in vec3 Normal;

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

void main() {
    vec3 normal = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 viewDir = normalize(viewPos - FragPos);
    
    // Increased ambient for daytime
    float ambientStrength = 0.35;
    vec3 ambient = ambientStrength * lightColor * material.diffuse;
    
    // Enhanced diffuse for daytime
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor * material.diffuse * 2.0; // Increased for daytime
    
    // Enhanced specular for shinier appearance
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess * 1.5); // Increased shininess
    vec3 specular = spec * lightColor * material.specular * 1.2; // Increased specular intensity
    
    // Combine with adjusted weights for daytime
    vec3 result = ambient + diffuse + specular;
    
    // Enhanced highlight preservation
    result = result / (result + vec3(0.8)); // Adjusted for brighter highlights
    
    // Increased saturation for daytime
    vec3 luminance = vec3(dot(result, vec3(0.299, 0.587, 0.114)));
    result = mix(luminance, result, 1.5); // More saturation
    
    // Add subtle rim lighting for better edge definition
    float rim = 1.0 - max(dot(normal, viewDir), 0.0);
    rim = pow(rim, 3.0);
    result += rim * material.specular * 0.3; // Subtle rim light
    
    FragColor = vec4(result, material.opacity);
}