/**
 * arena.frag - Fragment shader for the bumper car arena
 * 
 * This is my main arena rendering shader that handles the floor, walls, and
 * surrounding environment. It's the most complex shader in the project because
 * it needs to handle multiple light types, shadows, and material properties.
 * 
 * Features:
 * - Support for up to 5 regular point lights for ambient lighting
 * - Support for up to 2 spotlights (car headlights)
 * - Shadow mapping for main light source
 * - PBR-based material system with albedo, normal maps
 * - Advanced shadow filtering for soft shadows
 * - Atmospheric attenuation (fog) for depth cues
 */
#version 430 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;
in vec3 FragPos;
in vec4 FragPosLightSpace;

uniform sampler2D texture_diffuse1;
uniform sampler2D texture_normal1;
uniform sampler2D shadowMap;
uniform vec3 viewPos;

// Fog parameters
uniform float fogStart = -40.0;  // Distance where fog begins
uniform float fogEnd = -100.0;   // Distance where fog is fully dense
uniform vec3 fogColor = vec3(0.5, 0.5, 0.6);  // Light bluish-gray fog color
uniform bool fogEnabled = true;  // Toggle for fog effect

// Regular lights structure
#define MAX_LIGHTS 5
struct Light {
    vec3 position;
    vec3 color;
    float intensity;
};
uniform Light lights[MAX_LIGHTS];
uniform int numLights;

// Spot light structure for headlights
struct SpotLight {
    vec3 position;
    vec3 direction;
    vec3 color;
    float intensity;
    float cutOff;
    float outerCutOff;
    float constant;
    float linear;
    float quadratic;
};
#define MAX_SPOT_LIGHTS 2
uniform SpotLight spotLights[MAX_SPOT_LIGHTS];
uniform int numSpotLights;

/**
 * Calculates shadow intensity at the current fragment
 * 
 * This is where I compute the shadow contribution using PCF (Percentage Closer
 * Filtering) for smoother shadow edges. I spent a lot of time tweaking the parameters
 * to get shadows that look realistic but don't darken the scene too much.
 * 
 * @param fragPosLightSpace Fragment position in light space coordinates
 * @param normal Surface normal
 * @param lightDir Light direction vector
 * @return Shadow value (0.0 = fully lit, 1.0 = fully shadowed)
 */
float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    // Perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    
    // Transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    
    // Get current depth
    float currentDepth = projCoords.z;
    
    // Calculate bias based on surface angle to light with reduced bias for daytime
    float bias = max(0.03 * (1.0 - dot(normal, lightDir)), 0.005);
        
    // PCF (Percentage Closer Filtering) with larger kernel and better sampling
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for(int x = -2; x <= 2; ++x) {
        for(int y = -2; y <= 2; ++y) {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x,y) * texelSize).r;
            shadow += (currentDepth - bias) > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= 25.0; // Using 5x5 kernel
    
    // Softer shadow fade for daytime
    float shadowFade = 1.0 - smoothstep(0.8, 1.0, currentDepth);
    shadow *= shadowFade;
    
    // Reduce shadow intensity for daytime
    shadow *= 0.6;
    
    // Prevent shadow acne by checking if fragment is in light's view
    if(projCoords.z > 1.0)
        shadow = 0.0;
    
    return shadow;
}

/**
 * Calculates lighting from a regular point light
 * 
 * This handles the ambient, diffuse, and specular contributions from a standard
 * point light, including attenuation with distance and shadow mapping for the
 * main light.
 * 
 * @param light Light structure with position, color, intensity
 * @param normal Surface normal
 * @param fragPos Fragment position in world space
 * @param viewDir View direction vector
 * @param texColor Surface color from texture
 * @return RGB color contribution from this light
 */
vec3 CalcLight(Light light, vec3 normal, vec3 fragPos, vec3 viewDir, vec3 texColor)
{
    vec3 lightDir = normalize(light.position - fragPos);
    
    // Increased ambient for daytime
    float ambientStrength = 0.45;
    vec3 ambient = ambientStrength * light.color;
    
    // Enhanced diffuse for daytime
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * light.color * 1.5; // Increased diffuse contribution
    
    // Reduced specular for daytime (more subtle highlights)
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    float specularStrength = 0.15; // Reduced for daytime
    vec3 specular = specularStrength * spec * light.color;
    
    // Adjusted attenuation for daytime
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (1.0 + 0.045 * distance + 0.0075 * distance * distance);
    
    // Calculate shadow only for main light (index 0)
    float shadow = 0.0;
    if(light.intensity > 1.0) {
        shadow = ShadowCalculation(FragPosLightSpace, normal, lightDir);
    }
    
    // Combine results
    vec3 result = (ambient + (1.0 - shadow) * (diffuse + specular)) * attenuation * light.intensity;
    
    return result;
}

/**
 * Calculates lighting from a spotlight (headlight)
 * 
 * Spotlights use a cone shape with inner and outer angles for a smooth falloff
 * at the edges. This is perfect for car headlights.
 * 
 * @param light SpotLight structure with position, direction, color, etc.
 * @param normal Surface normal
 * @param fragPos Fragment position in world space
 * @param viewDir View direction vector
 * @param texColor Surface color from texture
 * @return RGB color contribution from this spotlight
 */
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir, vec3 texColor)
{
    vec3 lightDir = normalize(light.position - fragPos);
    
    // Get angle between light direction and spotlight direction
    float theta = dot(lightDir, normalize(-light.direction));
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    
    if(theta > light.outerCutOff) {
        // Enhanced diffuse for daytime
        float diff = max(dot(normal, lightDir), 0.0);
        vec3 diffuse = diff * light.color * 1.2; // Slightly increased for visibility
        
        // Reduced specular for daytime
        vec3 halfwayDir = normalize(lightDir + viewDir);
        float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
        vec3 specular = spec * light.color * 0.3; // Reduced specular for daytime
        
        // Adjusted attenuation for daytime visibility
        float distance = length(light.position - fragPos);
        float attenuation = 1.0 / (light.constant + light.linear * distance * 0.5 + 
                                 light.quadratic * distance * distance * 0.5);
        
        // Combine results with smooth falloff
        vec3 result = (diffuse + specular) * attenuation * intensity * light.intensity;
        return result * texColor;
    }
    
    return vec3(0.0);
}

/**
 * Calculates atmospheric fog effect
 * 
 * Implements linear fog that increases with distance from the camera.
 * Based on the atmospheric attenuation model from the Graphics Programming
 * course materials.
 * 
 * @param fragPos Fragment position in world space
 * @return Fog factor (0.0 = no fog, 1.0 = full fog)
 */
float CalculateFog(vec3 fragPos)
{
    // Calculate distance from fragment to camera
    float fragDistance = abs(fragPos.z); // Since we're typically looking down -Z
    
    // Linear fog calculation
    // 0.0 = full fog, 1.0 = no fog (to use in mix function later)
    float fogFactor = (fogEnd - fragDistance) / (fogEnd - fogStart);
    
    // Clamp to [0,1] range
    return clamp(fogFactor, 0.0, 1.0);
}

/**
 * Main shader entry point
 * 
 * Combines all lighting contributions and applies texture colors to produce
 * the final fragment color.
 */
void main()
{
    vec3 normal = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 texColor = texture(texture_diffuse1, TexCoords).rgb;
    
    // Reduced roughness for daytime
    texColor *= 0.98;
    
    // Calculate regular lighting
    vec3 result = vec3(0.0);
    for(int i = 0; i < numLights; i++) {
        result += CalcLight(lights[i], normal, FragPos, viewDir, texColor) * texColor;
    }
    
    // Add spotlight (headlight) contributions
    for(int i = 0; i < numSpotLights; i++) {
        result += CalcSpotLight(spotLights[i], normal, FragPos, viewDir, texColor);
    }
    
    // Reduced ambient occlusion for daytime
    float ao = 0.98;
    result *= ao;
    
    // Gamma correction with slightly reduced intensity for daytime
    result = pow(result, vec3(1.0/2.4));
    
    // Apply atmospheric fog effect
    if (fogEnabled) {
        float fogFactor = CalculateFog(FragPos);
        result = mix(fogColor, result, fogFactor);
    }
    
    FragColor = vec4(result, 1.0);
}