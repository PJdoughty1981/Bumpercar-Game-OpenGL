/**
 * Arena.cpp
 *
 * Implements the arena environment for my bumper car game, including floor, walls,
 * and surrounding grass area with textured rendering and collision detection.
 *
 * This was one of the first components I created for the game because it establishes
 * the physical boundaries and visual foundation for everything else. I spent considerable
 * time getting the texture mapping right to ensure the floor and walls look good
 * from all camera angles.
 * 
 * Last modified 12/03/2025
 */

#include "Arena.h"
#include <stb_image.h>
#include <iostream>
#include <cmath>

 //-----------------------------------------------------------------------------
 // Constants
 //-----------------------------------------------------------------------------
const float WALL_HEIGHT = 0.4f;      // Height of arena walls
const float WALL_THICKNESS = 1.2f;   // Thickness of arena walls

//-----------------------------------------------------------------------------
// Constructor & Destructor
//-----------------------------------------------------------------------------
/**
 * Constructor initializes arena parameters
 *
 * Sets up initial arena dimensions and initializes OpenGL resource IDs to zero.
 */
Arena::Arena() :
    arenaWidth(30.0f),
    arenaLength(30.0f),
    grassWidth(50.0f),
    grassLength(60.0f),
    wallHeight(WALL_HEIGHT),
    floorVAO(0),
    floorVBO(0),
    floorEBO(0),
    wallVAO(0),
    wallVBO(0),
    wallEBO(0),
    floorTexture(0),
    floorNormalMap(0),
    floorMetalnessMap(0)
{
}

/**
 * Destructor cleans up OpenGL resources
 *
 * Ensures all buffers and textures are properly deleted.
 */
Arena::~Arena() {
    // Clean up OpenGL resources - very important to avoid memory leaks
    glDeleteVertexArrays(1, &floorVAO);
    glDeleteBuffers(1, &floorVBO);
    glDeleteBuffers(1, &floorEBO);
    glDeleteVertexArrays(1, &wallVAO);
    glDeleteBuffers(1, &wallVBO);
    glDeleteBuffers(1, &wallEBO);
    glDeleteTextures(1, &floorTexture);
    glDeleteTextures(1, &floorNormalMap);
    glDeleteTextures(1, &floorMetalnessMap);
    glDeleteVertexArrays(1, &grassVAO);
    glDeleteBuffers(1, &grassVBO);
    glDeleteBuffers(1, &grassEBO);
    glDeleteTextures(1, &grassColorTexture);
    glDeleteTextures(1, &grassNormalMap);
    glDeleteTextures(1, &grassRoughnessMap);
    glDeleteTextures(1, &grassAOMap);
}

//-----------------------------------------------------------------------------
// Initialization
//-----------------------------------------------------------------------------
/**
 * Initializes the arena components
 *
 * Sets up geometry, buffers, and textures for all arena components.
 * This is called once during game startup.
 */
void Arena::Initialize() {
    GenerateFloorGeometry();
    GenerateWallGeometry();
    GenerateGrassGeometry();
    SetupBuffers();
    SetupGrassBuffers();
    LoadTextures();
    LoadGrassTextures();
}

//-----------------------------------------------------------------------------
// Geometry Generation
//-----------------------------------------------------------------------------
/**
 * Generate vertices and indices for the grass area
 *
 * Creates a large quad for the grass area surrounding the arena.
 * The grass texture will be stretched to fill the entire area without tiling to blend into the 
 * skybox floor better.
 */
void Arena::GenerateGrassGeometry() {
    float halfWidth = grassWidth / 2.0f;
    float halfLength = grassLength / 2.0f;

    // Non-scaled texture coordinates (0-1 range for the entire area)
    grassVertices = {
        // Position                                   // TexCoords        // Normal
        {glm::vec3(-halfWidth, -0.01f, -halfLength), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(halfWidth, -0.01f, -halfLength),  glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(halfWidth, -0.01f, halfLength),   glm::vec2(1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(-halfWidth, -0.01f, halfLength),  glm::vec2(0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)}
    };

    // Grass indices for drawing as triangles
    grassIndices = { 0, 1, 2, 2, 3, 0 };
}

/**
 * Generates vertices and indices for the arena floor
 *
 * Creates a quad for the main arena floor with scaled texture coordinates.
 */
void Arena::GenerateFloorGeometry() {
    float halfWidth = arenaWidth / 2.0f;
    float halfLength = arenaLength / 2.0f;
    const float TEXTURE_SCALE = 40.0f;

    // Floor vertices with scaled texture coordinates
    floorVertices = {
        // Position                               // TexCoords                            // Normal
        {glm::vec3(-halfWidth, 0.0f, -halfLength), glm::vec2(0.0f, 0.0f) * TEXTURE_SCALE, glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(halfWidth, 0.0f, -halfLength), glm::vec2(1.0f, 0.0f) * TEXTURE_SCALE, glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(halfWidth, 0.0f, halfLength), glm::vec2(1.0f, 1.0f) * TEXTURE_SCALE, glm::vec3(0.0f, 1.0f, 0.0f)},
        {glm::vec3(-halfWidth, 0.0f, halfLength), glm::vec2(0.0f, 1.0f) * TEXTURE_SCALE, glm::vec3(0.0f, 1.0f, 0.0f)}
    };

    // Floor indices for drawing as triangles
    floorIndices = { 0, 1, 2, 2, 3, 0 };
}

/**
 * Generates vertices and indices for the arena walls
 *
 * Creates the wall geometry with proper normals for lighting.
 * The walls form a square boundary around the arena floor.
 */
void Arena::GenerateWallGeometry() {
    float halfWidth = arenaWidth / 2.0f;
    float halfLength = arenaLength / 2.0f;
    const float wallThickness = WALL_THICKNESS;

    // Define wall vertices for a square arena
    std::vector<glm::vec3> wallCorners = {
        {-halfWidth, 0.0f, -halfLength},  // 0 front-left bottom
        {halfWidth, 0.0f, -halfLength},   // 1 front-right bottom
        {halfWidth, 0.0f, halfLength},    // 2 back-right bottom
        {-halfWidth, 0.0f, halfLength}    // 3 back-left bottom
    };

    // Wall generation lambda function to create uniform walls
    auto addWallSegment = [&](const glm::vec3& start, const glm::vec3& end, const glm::vec3& normal) {
        int baseIndex = wallVertices.size();

        // Bottom vertices
        wallVertices.push_back({ start, {0.0f, 0.0f}, normal });
        wallVertices.push_back({ end, {1.0f, 0.0f}, normal });

        // Top vertices
        wallVertices.push_back({ start + glm::vec3(0.0f, wallHeight, 0.0f), {0.0f, 1.0f}, normal });
        wallVertices.push_back({ end + glm::vec3(0.0f, wallHeight, 0.0f), {1.0f, 1.0f}, normal });

        // Generate wall segment indices
        std::vector<unsigned int> segmentIndices = {
        static_cast<unsigned int>(baseIndex),
        static_cast<unsigned int>(baseIndex + 1),
        static_cast<unsigned int>(baseIndex + 2),  // Bottom face
        static_cast<unsigned int>(baseIndex + 1),
        static_cast<unsigned int>(baseIndex + 3),
        static_cast<unsigned int>(baseIndex + 2),
        static_cast<unsigned int>(baseIndex),
        static_cast<unsigned int>(baseIndex + 2),
        static_cast<unsigned int>(baseIndex + 1),  // Top face
        static_cast<unsigned int>(baseIndex + 2),
        static_cast<unsigned int>(baseIndex + 3),
        static_cast<unsigned int>(baseIndex + 1)
        };

        wallIndices.insert(wallIndices.end(), segmentIndices.begin(), segmentIndices.end());
        };

    // Generate walls with proper normals for each side
    addWallSegment(wallCorners[0], wallCorners[1], glm::vec3(0.0f, 0.0f, -1.0f)); // Front wall
    addWallSegment(wallCorners[1], wallCorners[2], glm::vec3(1.0f, 0.0f, 0.0f));  // Right wall
    addWallSegment(wallCorners[2], wallCorners[3], glm::vec3(0.0f, 0.0f, 1.0f));  // Back wall
    addWallSegment(wallCorners[3], wallCorners[0], glm::vec3(-1.0f, 0.0f, 0.0f)); // Left wall
}

//-----------------------------------------------------------------------------
// Buffer Setup
//-----------------------------------------------------------------------------
/**
 * Sets up OpenGL buffers for grass geometry
 *
 * Creates and configures VAO, VBO, and EBO for the grass area.
 */
void Arena::SetupGrassBuffers() {
    glGenVertexArrays(1, &grassVAO);
    glGenBuffers(1, &grassVBO);
    glGenBuffers(1, &grassEBO);

    glBindVertexArray(grassVAO);
    glBindBuffer(GL_ARRAY_BUFFER, grassVBO);
    glBufferData(GL_ARRAY_BUFFER, grassVertices.size() * sizeof(Vertex), grassVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grassEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, grassIndices.size() * sizeof(unsigned int), grassIndices.data(), GL_STATIC_DRAW);

    // Setup vertex attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
}

/**
 * Sets up OpenGL buffers for floor and wall geometry
 *
 * Creates and configures VAOs, VBOs, and EBOs for the arena floor and walls.
 */
void Arena::SetupBuffers() {
    // Floor Buffers
    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glGenBuffers(1, &floorEBO);

    glBindVertexArray(floorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, floorVertices.size() * sizeof(Vertex), floorVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, floorIndices.size() * sizeof(unsigned int), floorIndices.data(), GL_STATIC_DRAW);

    // Vertex attributes for floor
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

    // Wall Buffers
    glGenVertexArrays(1, &wallVAO);
    glGenBuffers(1, &wallVBO);
    glGenBuffers(1, &wallEBO);

    glBindVertexArray(wallVAO);
    glBindBuffer(GL_ARRAY_BUFFER, wallVBO);
    glBufferData(GL_ARRAY_BUFFER, wallVertices.size() * sizeof(Vertex), wallVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wallEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, wallIndices.size() * sizeof(unsigned int), wallIndices.data(), GL_STATIC_DRAW);

    // Vertex attributes for walls
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
}
//-----------------------------------------------------------------------------
// Texture Loading
//-----------------------------------------------------------------------------
/**
 * Loads and configures a texture from file
 *
 * Creates an OpenGL texture from an image file with proper configuration.
 * Handles different image formats and sets appropriate texture parameters.
 *
 * path Path to the image file
 * return OpenGL texture ID
 */
unsigned int Arena::LoadTexture(const char* path) {
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format = (nrComponents == 1) ? GL_RED :
            (nrComponents == 3) ? GL_RGB : GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        // Set texture parameters for better quality
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Apply anisotropic filtering if available
        float maxAniso = 0.0f;
        if (glfwExtensionSupported("GL_EXT_texture_filter_anisotropic")) {
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);
        }

        stbi_image_free(data);
    }
    else {
        std::cerr << "Texture failed to load at path: " << path << std::endl;
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

/**
 * Loads grass textures from files
 *
 * Loads color, normal, roughness, and ambient occlusion maps for the grass.
 * Uses clamp-to-edge to prevent texture tiling.
 */
void Arena::LoadGrassTextures() {
    stbi_set_flip_vertically_on_load(true);

    // Load grass textures
    const char* colorPath = "Resources/textures/environment/grass/Grass007_2K-JPG_Color.jpg";
    const char* normalPath = "Resources/textures/environment/grass/Grass007_2K-JPG_NormalGL.jpg";
    const char* roughnessPath = "Resources/textures/environment/grass/Grass007_2K-JPG_Roughness.jpg";
    const char* aoPath = "Resources/textures/environment/grass/Grass007_2K-JPG_AmbientOcclusion.jpg";

    // Load each texture with custom parameters
    grassColorTexture = LoadTextureWithParams(colorPath, GL_CLAMP_TO_EDGE);
    grassNormalMap = LoadTextureWithParams(normalPath, GL_CLAMP_TO_EDGE);
    grassRoughnessMap = LoadTextureWithParams(roughnessPath, GL_CLAMP_TO_EDGE);
    grassAOMap = LoadTextureWithParams(aoPath, GL_CLAMP_TO_EDGE);
}

/**
 * Loads and configures a texture from file with specific wrapping parameters
 *
 * Creates an OpenGL texture from an image file with custom configuration.
 * Handles different image formats and sets appropriate texture parameters.
 *
 * path Path to the image file
 * wrapMode Texture wrapping mode (GL_REPEAT or GL_CLAMP_TO_EDGE)
 * return OpenGL texture ID
 */
unsigned int Arena::LoadTextureWithParams(const char* path, GLint wrapMode) {
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format = (nrComponents == 1) ? GL_RED :
            (nrComponents == 3) ? GL_RGB : GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        // Set texture parameters with custom wrap mode
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Apply anisotropic filtering if available
        float maxAniso = 0.0f;
        if (glfwExtensionSupported("GL_EXT_texture_filter_anisotropic")) {
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);
        }

        stbi_image_free(data);
        std::cout << "Loaded texture: " << path << " (" << width << "x" << height << ")" << std::endl;
    }
    else {
        std::cerr << "Texture failed to load at path: " << path << std::endl;
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

/**
 * Loads arena floor and wall textures
 *
 * Loads color, normal, and metalness maps for the arena surfaces.
 */
void Arena::LoadTextures() {
    stbi_set_flip_vertically_on_load(true);

    // Relative paths to textures
    const char* metalnessPath = "Resources/textures/arena/Rubber_Roughness.jpg";
    const char* colorPath = "Resources/textures/arena/Rubber_COLOR.jpg";
    const char* normalPath = "Resources/textures/arena/Rubber_NormalGL.jpg";

    // Variables for texture loading
    int width, height, nrChannels;
    unsigned char* data;

    // Metalness Map
    glGenTextures(1, &floorMetalnessMap);
    glBindTexture(GL_TEXTURE_2D, floorMetalnessMap);
    SetTextureParameters();
    data = stbi_load(metalnessPath, &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format = (nrChannels == 1) ? GL_RED :
            (nrChannels == 3) ? GL_RGB : GL_RGBA;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load metalness map: " << metalnessPath << std::endl;
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
    }

    // Color/Albedo Texture with some built in debugging
    glGenTextures(1, &floorTexture);
    glBindTexture(GL_TEXTURE_2D, floorTexture);
    SetTextureParameters();
    data = stbi_load(colorPath, &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format = (nrChannels == 1) ? GL_RED :
            (nrChannels == 3) ? GL_RGB : GL_RGBA;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load color map: " << colorPath << std::endl;
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
    }

    // Normal Map
    glGenTextures(1, &floorNormalMap);
    glBindTexture(GL_TEXTURE_2D, floorNormalMap);
    SetTextureParameters();
    data = stbi_load(normalPath, &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format = (nrChannels == 1) ? GL_RED :
            (nrChannels == 3) ? GL_RGB : GL_RGBA;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load normal map: " << normalPath << std::endl;
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
    }
}

/**
 * Sets standard texture parameters
 *
 * Configures commonly used texture parameters for better visual quality.
 */
void Arena::SetTextureParameters() {
    // Standard texture parameters for good appearance
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Apply anisotropic filtering if available
    float maxAniso = 0.0f;
    if (glfwExtensionSupported("GL_EXT_texture_filter_anisotropic")) {
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);
    }
}

//-----------------------------------------------------------------------------
// Rendering Methods
//-----------------------------------------------------------------------------
/**
 * Renders the complete arena
 *
 * Draws the grass, floor, and walls using the provided shader.
 * The rendering order ensures proper depth sorting.
 *
 * shader Shader to use for rendering
 */
void Arena::Render(Shader& shader) {
    shader.use();
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);

    // Render grass area first (underneath everything)
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, grassColorTexture);
    shader.setInt("texture_diffuse1", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, grassNormalMap);
    shader.setInt("texture_normal1", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, grassRoughnessMap);
    shader.setInt("texture_metalness1", 2);

    glBindVertexArray(grassVAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(grassIndices.size()), GL_UNSIGNED_INT, 0);

    // Then render floor
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, floorTexture);
    shader.setInt("texture_diffuse1", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, floorNormalMap);
    shader.setInt("texture_normal1", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, floorMetalnessMap);
    shader.setInt("texture_metalness1", 2);

    glBindVertexArray(floorVAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(floorIndices.size()), GL_UNSIGNED_INT, 0);

    // Finally render walls
    shader.setVec3("material.diffuse", 1.0f, 0.5f, 0.0f);  // Orange color
    shader.setVec3("material.specular", 0.7f, 0.7f, 0.7f);
    shader.setFloat("material.shininess", 32.0f);

    glBindVertexArray(wallVAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(wallIndices.size()), GL_UNSIGNED_INT, 0);
}

//-----------------------------------------------------------------------------
// Collision Detection
//-----------------------------------------------------------------------------
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
bool Arena::CheckCollision(const glm::vec3& position, float radius) {
    float halfWidth = arenaWidth / 2.0f;
    float halfLength = arenaLength / 2.0f;

    // Check if the position is outside the arena boundaries
    if (position.x + radius > halfWidth ||
        position.x - radius < -halfWidth ||
        position.z + radius > halfLength ||
        position.z - radius < -halfLength) {
        return true;  // Collision detected
    }

    return false;  // No collision
}