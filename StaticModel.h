/**
 * StaticModel.h
 *
 * My implementation for loading and rendering static 3D models in the bumper car game.
 * I created this class to handle environment objects like the Ferris wheel and trees
 * that need different handling from the dynamic bumper cars. The class loads OBJ files,
 * manages material properties, and provides easy transformation controls.
 */
#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <string>
#include <map>
#include "Shader.h"

 /**
  * Vertex structure for static models
  *
  * Stores position, normal, and texture coordinates for each vertex.
  * I needed this specialized format to properly load and render OBJ models.
  */
struct StaticModelVertex {
    glm::vec3 Position;   // Vertex position in model space
    glm::vec3 Normal;     // Vertex normal for lighting calculations
    glm::vec2 TexCoords;  // Texture coordinates for mapping
};

/**
 * Material properties for static models
 *
 * Stores the various properties defined in MTL files, allowing
 * me to render models with their intended visual appearance.
 */
struct StaticModelMaterial {
    std::string name;          // Material name from MTL file
    glm::vec3 diffuseColor;    // Main/base color
    glm::vec3 specularColor;   // Highlight/reflection color
    float shininess;           // Surface smoothness
    float opacity;             // Transparency (1.0 = opaque)
    std::string diffuseMap;    // Path to diffuse texture file
};

/**
 * A mesh component of a static model
 *
 * Models often consist of multiple parts with different materials.
 * This structure lets me organize those parts and render them efficiently.
 */
struct StaticModelMesh {
    std::vector<StaticModelVertex> vertices;  // Vertex data
    std::vector<unsigned int> indices;        // Index data for drawing
    int materialIndex;                        // Which material to use
};

/**
 * Manages loading, transforming, and rendering static 3D models
 *
 * I created this class to handle environment objects in my bumper car game.
 * It loads OBJ/MTL files, maintains transformation state, and efficiently
 * renders models with proper materials.
 */
class StaticModel {
public:
    /**
     * Creates a static model from an OBJ file
     *
     * Initializes a model instance but doesn't load any data yet.
     * Initialize() must be called to actually load the model.
     *
     * modelPath Path to the OBJ file to load
     */
    StaticModel(const std::string& modelPath);

    /**
     * Cleans up OpenGL resources
     *
     * Ensures all GPU buffers are properly deleted to prevent memory leaks.
     */
    ~StaticModel();

    /**
     * Loads model data and prepares for rendering
     *
     * This is where I actually load the model geometry and materials,
     * and set up the GPU buffers. I separated this from the constructor
     * to have better control over the loading process.
     */
    void Initialize();

    /**
     * Renders the model with the specified shader
     *
     * Applies the current transformation, sets material properties,
     * and draws all meshes in the model.
     *
     * shader Shader to use for rendering
     */
    void Render(Shader& shader);

    // Transformation methods
    /**
     * Sets the model's position in world space
     *
     * I use this to place models at specific locations in the game world.
     *
     * pos Position vector in world space
     */
    void SetPosition(const glm::vec3& pos);

    /**
     * Sets the model's rotation
     *
     * Rotation is specified as an angle around an axis.
     *
     * angle Rotation angle in degrees
     * axis Normalized axis to rotate around
     */
    void SetRotation(float angle, const glm::vec3& axis);

    /**
     * Sets the model's uniform scale
     *
     * I use this to resize models to fit properly in the scene.
     *
     * scale Uniform scale factor (1.0 = original size)
     */
    void SetScale(float scale);

    /**
     * Changes a material's color at runtime
     *
     * Allows me to customize model appearance without reloading.
     * Useful for things like changing colors for different variants.
     *
     * materialName Name of the material to modify
     * color New diffuse color
     */
    void SetMaterialColor(const std::string& materialName, const glm::vec3& color);

    /**
     * Gets the model path
     *
     * Returns the path to the OBJ file used by this model.
     *
     * return Path to OBJ file
     */
    std::string GetModelPath() const { return modelPath; }

private:
    // Model data
    std::vector<StaticModelMesh> meshes;        // Mesh components
    std::vector<StaticModelMaterial> materials; // Material definitions
    std::string modelPath;                      // Path to OBJ file
    std::map<std::string, unsigned int> loadedTextures; // Cache for loaded textures

    // OpenGL buffers
    std::vector<GLuint> VAOs;                   // Vertex Array Objects
    std::vector<GLuint> VBOs;                   // Vertex Buffer Objects
    std::vector<GLuint> EBOs;                   // Element Buffer Objects

    // Transformation state
    glm::vec3 position;                         // World position
    float rotationAngle;                        // Rotation angle in degrees
    glm::vec3 rotationAxis;                     // Rotation axis (normalized)
    float modelScale;                           // Uniform scale factor

    /**
     * Loads geometry data from OBJ file
     *
     * Parses vertices, normals, UVs, and faces from the OBJ file,
     * organizing them into mesh segments. Also triggers material loading.
     *
     * return True if loading succeeded, false otherwise
     */
    bool LoadOBJModel();

    /**
     * Loads material definitions from MTL file
     *
     * Parses material properties like colors, shininess, etc.,
     * to be used during rendering.
     *
     * mtlPath Path to the MTL file
     * return True if loading succeeded, false otherwise
     */
    bool LoadMaterialLibrary(const std::string& mtlPath);

    /**
     * Loads a texture from file
     *
     * Creates and configures an OpenGL texture from an image file.
     *
     * path Path to the texture file
     * return OpenGL texture ID
     */
    unsigned int LoadTexture(const std::string& path);

    /**
     * Creates and configures OpenGL buffers for rendering
     *
     * Sets up VAOs, VBOs, and EBOs for each mesh segment, configuring
     * vertex attributes for positions, normals, and texture coordinates.
     */
    void SetupBuffers();

    /**
     * Deletes all OpenGL buffer objects
     *
     * Ensures proper cleanup of GPU resources.
     */
    void CleanupBuffers();
};