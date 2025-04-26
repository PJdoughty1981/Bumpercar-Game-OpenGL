/**
 * Arena.h
 *
 * This class defines the environment where all the bumper car action happens.
 * I've designed it to create a visually appealing and physically correct arena
 * with a central floor, surrounding walls, and a grass area outside.
 *
 * The arena handles:
 * - Generating geometry for floor, walls, and grass
 * - Loading and managing textures for each surface
 * - Setting up OpenGL buffers and rendering
 * - Collision detection for arena boundaries
 *
 * I've gone with a square arena design with colored walls that provide both
 * visual boundaries and physical constraints for the bumper cars. The textured
 * floor gives good visual feedback for movement and adds to the arcade feeling
 * I want for the game.
 */
#pragma once

 //-----------------------------------------------------------------------------
 // Includes
 //-----------------------------------------------------------------------------
#include <glad/glad.h>
#include <GLFW/include/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Shader.h"
#include <vector>

//-----------------------------------------------------------------------------
// Structures
//-----------------------------------------------------------------------------
/**
 * Vertex structure for arena geometry
 *
 * Stores the position, texture coordinates, and normal
 * for each vertex in the arena geometry.
 */
struct Vertex {
    glm::vec3 Position;   // Vertex position in model space
    glm::vec2 TexCoords;  // Texture coordinates
    glm::vec3 Normal;     // Surface normal for lighting
};

//-----------------------------------------------------------------------------
// Arena Class Declaration
//-----------------------------------------------------------------------------
class Arena {
public:
    //-----------------------------------------------------------------------------
    // Public Methods
    //-----------------------------------------------------------------------------
    /**
     * Constructor initializes default values
     *
     * Sets up initial arena dimensions and initializes OpenGL resource IDs.
     */
    Arena();

    /**
     * Destructor cleans up OpenGL resources
     *
     * Ensures all buffers and textures are properly deleted.
     */
    ~Arena();

    /**
     * Initializes the arena
     *
     * Generates geometry, sets up buffers, and loads textures.
     * This needs to be called after creating an Arena but before using it.
     */
    void Initialize();

    /**
     * Renders the arena
     *
     * Draws the floor, walls, and grass areas using the specified shader.
     *
     * shader Shader to use for rendering
     */
    void Render(Shader& shader);

    /**
     * Checks if an object is colliding with arena boundaries
     *
     * Tests if a spherical object at the given position intersects
     * with any of the arena walls.
     *
     * position Center position of the object
     * radius Radius of the object
     * return True if collision detected, false otherwise
     */
    bool CheckCollision(const glm::vec3& position, float radius);

    /**
     * Loads a texture from file
     *
     * Creates and configures an OpenGL texture from an image file.
     * Made public as it's used for multiple texture types.
     *
     * path Path to the image file
     * return OpenGL texture ID
     */
    unsigned int LoadTexture(const char* path);

private:
    //-----------------------------------------------------------------------------
    // Member Variables
    //-----------------------------------------------------------------------------
    // Arena dimensions
    float arenaWidth;      // Width of the main arena floor
    float arenaLength;     // Length of the main arena floor
    float wallHeight;      // Height of the surrounding walls
    float endRadius;       // Radius of any rounded corners
    float grassWidth;      // Width of the surrounding grass area
    float grassLength;     // Length of the surrounding grass area

    // Geometry data
    std::vector<Vertex> floorVertices;     // Floor mesh vertices
    std::vector<unsigned int> floorIndices; // Floor mesh indices
    std::vector<Vertex> wallVertices;      // Wall mesh vertices
    std::vector<unsigned int> wallIndices;  // Wall mesh indices
    std::vector<Vertex> grassVertices;     // Grass mesh vertices
    std::vector<unsigned int> grassIndices; // Grass mesh indices

    // OpenGL objects - Arena
    GLuint floorVAO, floorVBO, floorEBO;   // Floor mesh buffers
    GLuint wallVAO, wallVBO, wallEBO;      // Wall mesh buffers
    GLuint floorTexture;                   // Floor color/albedo texture
    GLuint floorNormalMap;                 // Floor normal map
    GLuint floorMetalnessMap;              // Floor metalness map

    // OpenGL objects - Grass
    GLuint grassVAO, grassVBO, grassEBO;   // Grass mesh buffers
    GLuint grassColorTexture;              // Grass color/albedo texture
    GLuint grassNormalMap;                 // Grass normal map
    GLuint grassRoughnessMap;              // Grass roughness map
    GLuint grassAOMap;                     // Grass ambient occlusion map

    // Collision data
    std::vector<glm::vec3> wallNormals;    // Wall surface normals
    std::vector<glm::vec3> wallPositions;  // Wall center positions

    unsigned int LoadTextureWithParams(const char* path, GLint wrapMode); // Load arena texture

    //-----------------------------------------------------------------------------
    // Private Helper Methods
    //-----------------------------------------------------------------------------
    /**
     * Generates floor geometry
     *
     * Creates vertices and indices for the arena floor.
     */
    void GenerateFloorGeometry();

    /**
     * Generates wall geometry
     *
     * Creates vertices and indices for the arena walls.
     */
    void GenerateWallGeometry();

    /**
     * Generates grass geometry
     *
     * Creates vertices and indices for the surrounding grass area.
     */
    void GenerateGrassGeometry();

    /**
     * Sets up OpenGL buffers for arena geometry
     *
     * Creates and configures VAOs, VBOs, and EBOs for floor and walls.
     */
    void SetupBuffers();

    /**
     * Sets up OpenGL buffers for grass geometry
     *
     * Creates and configures VAO, VBO, and EBO for the grass area.
     */
    void SetupGrassBuffers();

    /**
     * Loads textures for the arena floor and walls
     *
     * Loads color, normal, and metalness textures for the arena surfaces.
     */
    void LoadTextures();

    /**
     * Loads textures for the grass area
     *
     * Loads color, normal, roughness, and ambient occlusion textures for grass.
     */
    void LoadGrassTextures();

    /**
     * Sets standard texture parameters
     *
     * Configures filtering, wrapping, and other texture parameters
     * for a currently bound texture.
     */
    void SetTextureParameters();
};