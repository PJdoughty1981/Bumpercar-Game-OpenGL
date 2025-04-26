/**
 * StaticModel.cpp
 *
 * Implementation of my StaticModel class for loading and rendering environment
 * objects like the Ferris wheel and trees in the bumper car game. I designed
 * this to efficiently handle static (non-moving) objects separately from the
 * more complex physics-based bumper cars.
 */
#include "StaticModel.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <stb_image.h>

 /**
  * Constructor initializes model properties
  *
  * Stores the model path and sets default transform values.
  * The actual model loading happens in Initialize() to give
  * more control over the loading process.
  *
  * modelPath Path to the OBJ file to load
  */
StaticModel::StaticModel(const std::string& modelPath) :
    modelPath(modelPath),
    position(0.0f),
    rotationAngle(0.0f),
    rotationAxis(0.0f, 1.0f, 0.0f),
    modelScale(1.0f)
{
}

/**
 * Destructor cleans up OpenGL resources
 *
 * Makes sure all GPU buffers are properly deleted to prevent memory leaks.
 */
StaticModel::~StaticModel() {
    CleanupBuffers();
    // Clean up textures
    for (auto& texture : loadedTextures) {
        glDeleteTextures(1, &texture.second);
    }
}

/**
 * Loads the model and sets up OpenGL resources
 *
 * This is where I actually load the model geometry and materials,
 * and set up the GPU buffers for rendering.
 */
void StaticModel::Initialize() {
    // Load model and materials
    if (!LoadOBJModel()) {
        std::cerr << "Failed to load static model: " << modelPath << std::endl;
        return;
    }

    // Setup OpenGL buffers
    SetupBuffers();
}

 /**
  * Loads a texture from file path
  *
  * Creates an OpenGL texture from an image file with proper configuration.
  *
  * path Path to the texture file
  * return OpenGL texture ID
  */
unsigned int StaticModel::LoadTexture(const std::string& path) {
    // Check if texture is already loaded
    if (loadedTextures.find(path) != loadedTextures.end()) {
        return loadedTextures[path];
    }

    unsigned int textureID;
    glGenTextures(1, &textureID);

    // Try to find the texture in multiple locations
    std::vector<std::string> possiblePaths = {
        path,
        "Resources/models/sign/" + path,
        modelPath.substr(0, modelPath.find_last_of("/\\") + 1) + path
    };

    int width = 0, height = 0, nrChannels = 0;
    unsigned char* data = nullptr;
    std::string foundPath;

    for (const auto& tryPath : possiblePaths) {
        data = stbi_load(tryPath.c_str(), &width, &height, &nrChannels, 0);
        if (data) {
            foundPath = tryPath;
            std::cout << "Successfully loaded texture from: " << tryPath << std::endl;
            break;
        }
    }

    if (data) {
        GLenum format;
        if (nrChannels == 1)
            format = GL_RED;
        else if (nrChannels == 3)
            format = GL_RGB;
        else if (nrChannels == 4)
            format = GL_RGBA;
        else {
            std::cerr << "Unsupported number of channels: " << nrChannels << " in " << foundPath << std::endl;
            format = GL_RGB;
        }

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        // Set texture parameters for better quality
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        std::cout << "Loaded texture: " << foundPath << " with ID " << textureID
            << " (size: " << width << "x" << height << ", channels: " << nrChannels << ")" << std::endl;

        // Store the texture ID for future reference
        loadedTextures[path] = textureID;
    }
    else {
        std::cerr << "Failed to load texture: " << path << std::endl;
        std::cerr << "Tried paths:" << std::endl;
        for (const auto& tryPath : possiblePaths) {
            std::cerr << "  - " << tryPath << std::endl;
        }
        std::cerr << "Error: " << stbi_failure_reason() << std::endl;
        return 0;
    }

    return textureID;
}

/**
 * Loads geometry and material data from OBJ file
 *
 * This is where I parse the OBJ file format to extract vertices,
 * normals, texture coordinates, and face definitions. I also
 * extract material references and load the corresponding MTL file.
 *
 * return True if loading succeeded, false otherwise
 */
bool StaticModel::LoadOBJModel() {
    std::ifstream file(modelPath);
    if (!file.is_open()) {
        std::cerr << "Could not open OBJ file: " << modelPath << std::endl;
        return false;
    }

    // Find MTL file path (same base path as OBJ)
    std::string mtlPath = modelPath.substr(0, modelPath.find_last_of('.')) + ".mtl";

    // Load material library
    if (!LoadMaterialLibrary(mtlPath)) {
        std::cerr << "Failed to load material library: " << mtlPath << std::endl;
        return false;
    }

    // Temporary storage for parsed data
    std::vector<glm::vec3> tempVertices;
    std::vector<glm::vec3> tempNormals;
    std::vector<glm::vec2> tempTexCoords;
    std::map<std::string, unsigned int> uniqueVertices;

    // Current mesh being constructed
    StaticModelMesh currentMesh;
    currentMesh.materialIndex = -1;

    // Parse the OBJ file line by line
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            // Vertex position
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            tempVertices.push_back(vertex);
        }
        else if (type == "vn") {
            // Vertex normal
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            tempNormals.push_back(glm::normalize(normal));
        }
        else if (type == "vt") {
            // Texture coordinate
            glm::vec2 texCoord;
            iss >> texCoord.x >> texCoord.y;
            tempTexCoords.push_back(texCoord);
        }
        else if (type == "usemtl") {
            // Material change - start a new mesh segment
            if (!currentMesh.vertices.empty()) {
                meshes.push_back(currentMesh);
                currentMesh = StaticModelMesh();
            }

            // Find the material index
            std::string materialName;
            iss >> materialName;
            for (size_t i = 0; i < materials.size(); ++i) {
                if (materials[i].name == materialName) {
                    currentMesh.materialIndex = i;
                    break;
                }
            }
        }
        else if (type == "f") {
            // Face definition
            std::string v1, v2, v3, v4;
            iss >> v1 >> v2 >> v3;

            // Check if this is a quad face (4 vertices)
            bool isQuad = false;
            if (iss >> v4) {
                isQuad = true;
            }

            // Process the first triangle
            std::vector<std::string> vertexData = { v1, v2, v3 };
            for (const auto& vertex : vertexData) {
                // If we haven't seen this exact vertex before, create a new one
                if (uniqueVertices.count(vertex) == 0) {
                    StaticModelVertex newVertex;
                    std::vector<std::string> indices;
                    std::stringstream ss(vertex);
                    std::string index;

                    // Parse vertex/texcoord/normal indices
                    while (std::getline(ss, index, '/')) {
                        indices.push_back(index);
                    }

                    // Position
                    if (!indices.empty() && !indices[0].empty()) {
                        int posIndex = std::stoi(indices[0]) - 1;
                        if (posIndex >= 0 && posIndex < tempVertices.size()) {
                            newVertex.Position = tempVertices[posIndex];
                        }
                    }

                    // Texture coordinates
                    if (indices.size() > 1 && !indices[1].empty()) {
                        int texIndex = std::stoi(indices[1]) - 1;
                        if (texIndex >= 0 && texIndex < tempTexCoords.size()) {
                            newVertex.TexCoords = tempTexCoords[texIndex];
                        }
                    }

                    // Normal
                    if (indices.size() > 2 && !indices[2].empty()) {
                        int normalIndex = std::stoi(indices[2]) - 1;
                        if (normalIndex >= 0 && normalIndex < tempNormals.size()) {
                            newVertex.Normal = tempNormals[normalIndex];
                        }
                    }

                    uniqueVertices[vertex] = currentMesh.vertices.size();
                    currentMesh.vertices.push_back(newVertex);
                }

                // Add index
                currentMesh.indices.push_back(uniqueVertices[vertex]);
            }

            // If it's a quad, add the second triangle (v1, v3, v4)
            if (isQuad) {
                vertexData = { v1, v3, v4 };
                for (const auto& vertex : vertexData) {
                    if (uniqueVertices.count(vertex) == 0) {
                        StaticModelVertex newVertex;
                        std::vector<std::string> indices;
                        std::stringstream ss(vertex);
                        std::string index;

                        while (std::getline(ss, index, '/')) {
                            indices.push_back(index);
                        }

                        if (!indices.empty() && !indices[0].empty()) {
                            int posIndex = std::stoi(indices[0]) - 1;
                            if (posIndex >= 0 && posIndex < tempVertices.size()) {
                                newVertex.Position = tempVertices[posIndex];
                            }
                        }

                        if (indices.size() > 1 && !indices[1].empty()) {
                            int texIndex = std::stoi(indices[1]) - 1;
                            if (texIndex >= 0 && texIndex < tempTexCoords.size()) {
                                newVertex.TexCoords = tempTexCoords[texIndex];
                            }
                        }

                        if (indices.size() > 2 && !indices[2].empty()) {
                            int normalIndex = std::stoi(indices[2]) - 1;
                            if (normalIndex >= 0 && normalIndex < tempNormals.size()) {
                                newVertex.Normal = tempNormals[normalIndex];
                            }
                        }

                        uniqueVertices[vertex] = currentMesh.vertices.size();
                        currentMesh.vertices.push_back(newVertex);
                    }

                    currentMesh.indices.push_back(uniqueVertices[vertex]);
                }
            }
        }
    }

    // Add the last mesh
    if (!currentMesh.vertices.empty()) {
        meshes.push_back(currentMesh);
    }

    std::cout << "Loaded model: " << modelPath << std::endl;
    std::cout << "Number of meshes: " << meshes.size() << std::endl;
    std::cout << "Number of materials: " << materials.size() << std::endl;

    return !meshes.empty();
}

/**
 * Loads material definitions from MTL file
 *
 * Parses material properties like colors, shininess, etc. from
 * the MTL file. These properties will be used during rendering.
 *
 * mtlPath Path to the MTL file
 * return True if loading succeeded, false otherwise
 */
bool StaticModel::LoadMaterialLibrary(const std::string& mtlPath) {
    std::ifstream mtlFile(mtlPath);
    if (!mtlFile.is_open()) {
        std::cerr << "Could not open MTL file: " << mtlPath << std::endl;
        return false;
    }

    StaticModelMaterial currentMaterial;
    std::string line, keyword;

    while (std::getline(mtlFile, line)) {
        std::istringstream iss(line);
        iss >> keyword;

        if (keyword == "newmtl") {
            // Store previous material if exists
            if (!currentMaterial.name.empty()) {
                materials.push_back(currentMaterial);
            }
            // Start new material
            currentMaterial = StaticModelMaterial();
            std::getline(iss >> std::ws, currentMaterial.name);
            std::cout << "Loading material: " << currentMaterial.name << std::endl;
        }
        else if (keyword == "Kd") {
            // Diffuse color
            iss >> currentMaterial.diffuseColor.x
                >> currentMaterial.diffuseColor.y
                >> currentMaterial.diffuseColor.z;
        }
        else if (keyword == "Ks") {
            // Specular color
            iss >> currentMaterial.specularColor.x
                >> currentMaterial.specularColor.y
                >> currentMaterial.specularColor.z;
        }
        else if (keyword == "Ns") {
            // Shininess
            iss >> currentMaterial.shininess;
        }
        else if (keyword == "d" || keyword == "Tr") {
            // Opacity
            iss >> currentMaterial.opacity;
        }
        else if (keyword == "map_Kd") {
            // Diffuse texture map
            std::string texPath;
            std::getline(iss >> std::ws, texPath);
            if (!texPath.empty()) {
                // Generate absolute path based on MTL file location
                std::string directory = mtlPath.substr(0, mtlPath.find_last_of("/\\") + 1);

                // If texture path already has Resource/ at the start, don't prepend directory
                if (texPath.find("Resources/") == 0) {
                    currentMaterial.diffuseMap = texPath;
                }
                else {
                    currentMaterial.diffuseMap = directory + texPath;
                }

                std::cout << "Material texture: " << currentMaterial.diffuseMap << std::endl;

                // Pre-load texture
                LoadTexture(currentMaterial.diffuseMap);
            }
        }
    }

    // Add last material
    if (!currentMaterial.name.empty()) {
        materials.push_back(currentMaterial);
    }

    return !materials.empty();
}

/**
 * Renders the model with its current transformation
 *
 * This is called every frame to draw the model. It applies the current
 * transformation, sets material properties, and draws all mesh segments.
 *
 * shader Shader to use for rendering
 */
void StaticModel::Render(Shader& shader) {
    // Create model matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::rotate(model, glm::radians(rotationAngle), rotationAxis);
    model = glm::scale(model, glm::vec3(modelScale));

    shader.setMat4("model", model);

    // Special handling for sign - check if this is the sign model
    bool isSignModel = (modelPath.find("Sign.obj") != std::string::npos);

    if (isSignModel) {
        // Log detailed debug info
        static bool firstRender = true;
        if (firstRender) {
            std::cout << "===========================================\n";
            std::cout << "SIGN RENDER DEBUG:\n";
            std::cout << "Sign model found: " << modelPath << "\n";
            std::cout << "Sign position: " << position.x << ", " << position.y << ", " << position.z << "\n";
            std::cout << "Sign scale: " << modelScale << "\n";
            std::cout << "Sign rotation: " << rotationAngle << " degrees\n";
            std::cout << "Number of materials: " << materials.size() << "\n";
            std::cout << "Number of meshes: " << meshes.size() << "\n";

            // Detailed material info
            for (size_t i = 0; i < materials.size(); i++) {
                std::cout << "  Material " << i << ": " << materials[i].name << "\n";
                std::cout << "    Diffuse color: " << materials[i].diffuseColor.x << ", "
                    << materials[i].diffuseColor.y << ", " << materials[i].diffuseColor.z << "\n";
                std::cout << "    Diffuse map: " << (materials[i].diffuseMap.empty() ? "none" : materials[i].diffuseMap) << "\n";
            }

            firstRender = false;
        }
    }

    // Render each mesh with its material
    for (size_t i = 0; i < meshes.size(); i++) {
        // Set material properties
        if (meshes[i].materialIndex >= 0 && meshes[i].materialIndex < materials.size()) {
            const auto& material = materials[meshes[i].materialIndex];

            // Set material properties in shader
            shader.setVec3("material.diffuse", material.diffuseColor);
            shader.setVec3("material.specular", material.specularColor);
            shader.setFloat("material.shininess", material.shininess);
            shader.setFloat("material.opacity", material.opacity);

            // Bind texture if available
            bool hasTexture = false;
            if (!material.diffuseMap.empty()) {
                unsigned int textureID = LoadTexture(material.diffuseMap);
                if (textureID > 0) {
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, textureID);
                    shader.setInt("texture_diffuse1", 0);
                    shader.setInt("material_has_texture", 1);
                    hasTexture = true;

                    // Extra log for sign texture debug
                    if (isSignModel && material.name == "SignLogo") {
                        static bool firstTextureBind = true;
                        if (firstTextureBind) {
                            std::cout << "Sign texture bound: ID " << textureID << " for mesh " << i << "\n";
                            std::cout << "===========================================\n";
                            firstTextureBind = false;
                        }
                    }
                }
            }

            if (!hasTexture) {
                shader.setInt("material_has_texture", 0);
            }
        }
        else {
            // Default material if no material is found
            shader.setVec3("material.diffuse", glm::vec3(1.0f, 1.0f, 1.0f));
            shader.setVec3("material.specular", glm::vec3(0.5f, 0.5f, 0.5f));
            shader.setFloat("material.shininess", 32.0f);
            shader.setFloat("material.opacity", 1.0f);
            shader.setInt("material_has_texture", 0);
        }

        // Bind mesh VAO and draw
        glBindVertexArray(VAOs[i]);

        // Enable alpha blending for transparent textures
        bool needsBlending = meshes[i].materialIndex >= 0 &&
            meshes[i].materialIndex < materials.size() &&
            !materials[meshes[i].materialIndex].diffuseMap.empty();

        if (needsBlending) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        // Draw the mesh
        glDrawElements(GL_TRIANGLES, meshes[i].indices.size(), GL_UNSIGNED_INT, 0);

        // Restore blend state
        if (needsBlending) {
            glDisable(GL_BLEND);
        }
    }
}

/**
 * Creates and configures OpenGL bufers for rendering
 *
 * Sets up one VAO, VBO, and EBO for each mesh segment, along with
 * the appropriate vertex attribute pointers.
 */
void StaticModel::SetupBuffers() {
    // Prepare buffers for each mesh
    VAOs.resize(meshes.size());
    VBOs.resize(meshes.size());
    EBOs.resize(meshes.size());

    glGenVertexArrays(meshes.size(), VAOs.data());
    glGenBuffers(meshes.size(), VBOs.data());
    glGenBuffers(meshes.size(), EBOs.data());

    for (size_t i = 0; i < meshes.size(); i++) {
        glBindVertexArray(VAOs[i]);

        // Vertex Buffer
        glBindBuffer(GL_ARRAY_BUFFER, VBOs[i]);
        glBufferData(GL_ARRAY_BUFFER,
            meshes[i].vertices.size() * sizeof(StaticModelVertex),
            meshes[i].vertices.data(),
            GL_STATIC_DRAW);

        // Index Buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[i]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            meshes[i].indices.size() * sizeof(unsigned int),
            meshes[i].indices.data(),
            GL_STATIC_DRAW);

        // Position attribute
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
            sizeof(StaticModelVertex), (void*)offsetof(StaticModelVertex, Position));

        // Normal attribute
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
            sizeof(StaticModelVertex), (void*)offsetof(StaticModelVertex, Normal));

        // Texture coordinate attribute
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
            sizeof(StaticModelVertex), (void*)offsetof(StaticModelVertex, TexCoords));
    }
}

/**
 * Deletes all OpenGL buffer objects
 *
 * Called during destruction to clean up GPU resources.
 */
void StaticModel::CleanupBuffers() {
    if (!VAOs.empty()) {
        glDeleteVertexArrays(VAOs.size(), VAOs.data());
        glDeleteBuffers(VBOs.size(), VBOs.data());
        glDeleteBuffers(EBOs.size(), EBOs.data());
    }
}

/**
 * Sets the model's position in world space
 *
 * pos Position vector in world space
 */
void StaticModel::SetPosition(const glm::vec3& pos) {
    position = pos;
}

/**
 * Sets the model's rotation
 *
 * angle Rotation angle in degrees
 * axis Normalized axis to rotate around
 */
void StaticModel::SetRotation(float angle, const glm::vec3& axis) {
    rotationAngle = angle;
    rotationAxis = axis;
}

/**
 * Sets the model's uniform scale
 *
 * scale Uniform scale factor (1.0 = original size)
 */
void StaticModel::SetScale(float scale) {
    modelScale = scale;
}

/**
 * Changes a material's color at runtime
 *
 * materialName Name of the material to modify
 * color New diffuse color
 */
void StaticModel::SetMaterialColor(const std::string& materialName, const glm::vec3& color) {
    for (auto& material : materials) {
        if (material.name == materialName) {
            material.diffuseColor = color;
            break;
        }
    }
}