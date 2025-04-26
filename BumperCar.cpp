/**
 * BumperCar.cpp
 *
 * Implements the core Player bumper car physics, rendering, and behavior systems including
 * collision response, movement controls, and visual effects.
 *
 * Key Components:
 * - Physics simulation with realistic friction and momentum
 * - Collision detection and response
 * - 3D model loading and rendering
 * - Material and texture management
 * - Impact feedback effects (shake, bounce)
 *
 * Key Functions:
 * - UpdateMovement(): Handles physics and movement updates
 * - HandleCollisionWith(): Manages collision responses
 * - ApplyTurnForces(): Controls turning mechanics
 * - LoadOBJModel(): Loads and processes 3D models
 * - Render(): Handles 3D rendering with materials
 *
 * Physics Properties:
 * - Realistic momentum conservation
 * - Semi-elastic collisions (ELASTICITY = 0.5)
 * - Angular velocity damping
 * - Ground friction simulation
 * - Impact-based visual feedback
 *
 * Author: Peter Doughty
 * Last Modified: 2024-02-08
 */

#define GLM_ENABLE_EXPERIMENTAL
#include "BumperCar.h"
#include <cmath>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stb_image.h>
#include <string>

//-----------------------------------------------------------------------------
// Constructor Implementation
//-----------------------------------------------------------------------------
BumperCar::BumperCar(const std::string& modelPath) :
    position(0.0f, 0.0f, 0.0f),
    rotation(0.0f),
    currentSpeed(0.0f),
    velocity(0.0f),
    angularVelocity(0.0f),
    isColliding(false),
    isTurning(false)
{
    if (!LoadOBJModel(modelPath)) {
        std::cerr << "Failed to load bumper car model from " << modelPath << std::endl;
    }
    position.y = -0.3f;  // Ground level
}

//-----------------------------------------------------------------------------
// Movement & Physics Implementation
//-----------------------------------------------------------------------------
void BumperCar::UpdateMovement(float deltaTime) {
    UpdatePhysics(deltaTime);
    UpdateShake(deltaTime);

    // Calculate rotation point at the rear center of the car
    float rearOffset = TURN_REAR_OFFSET;
    float lateralOffset = TURN_LATERAL_OFFSET;
    glm::vec3 forward = GetForwardVector();

    // Calculate lateral offset perpendicular to forward direction
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));

    // Combine rear and lateral offsets
    glm::vec3 rotationPoint = position +
        forward * rearOffset +     // Backward offset
        right * lateralOffset;     // Lateral offset

    // Calculate new position based on velocity
    glm::vec3 newPosition = position + velocity * deltaTime;

    // Apply rotation around the rear wheel point
    if (std::abs(angularVelocity) > 0.0f) {
        // Create rotation matrix around the rear wheel pivot
        float rotationAngle = angularVelocity * deltaTime;
        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotationAngle), glm::vec3(0.0f, 1.0f, 0.0f));

        // Translate relative to rotation point
        glm::vec3 relativePosition = newPosition - rotationPoint;
        glm::vec3 rotatedPosition = glm::vec3(rotationMatrix * glm::vec4(relativePosition, 1.0f));
        newPosition = rotatedPosition + rotationPoint;
    }

    // Check for collisions
    glm::vec3 wallNormal;
    if (CheckArenaCollision(newPosition, wallNormal)) {
        if (!isColliding) {  // Only handle collision if not already colliding
            HandleWallCollision(wallNormal);
            isColliding = true;
        }
    }
    else {
        isColliding = false;
        position = newPosition;
    }

    // Update rotation based on angular velocity
    rotation += angularVelocity * deltaTime;

    // Keep rotation in [0, 360) range
    while (rotation >= 360.0f) rotation -= 360.0f;
    while (rotation < 0.0f) rotation += 360.0f;

    // Reset turning flag
    isTurning = false;
}

void BumperCar::HandleCollisionWith(const glm::vec3& collisionNormal) {
    // Calculate impact speed and direction
    float impactSpeed = glm::length(velocity);

    // Get direction of impact relative to car's forward direction
    glm::vec3 forwardDir = GetForwardVector();
    float headOnFactor = -glm::dot(forwardDir, collisionNormal);

    // Calculate normal and tangential components of velocity
    float normalVelocityMagnitude = glm::dot(velocity, collisionNormal);
    glm::vec3 normalVelocity = collisionNormal * normalVelocityMagnitude;
    glm::vec3 tangentialVelocity = velocity - normalVelocity;

    // Conservation of momentum with elasticity factor 
    const float ELASTICITY = 0.5f;  // 0.5 = semi-elastic collision
    const float TANGENTIAL_FRICTION = 0.8f; // Preserve most tangential momentum

    // Calculate new velocity components with controlled bounce
    glm::vec3 newNormalVelocity;
    // Only bounce if moving toward the other object
    if (normalVelocityMagnitude < 0) {
        // Reverse normal component with elasticity factor
        newNormalVelocity = -normalVelocity * ELASTICITY;

        // Ensure minimum separation velocity to prevent sticking
        float minSeparationSpeed = 1.5f;
        if (glm::length(newNormalVelocity) < minSeparationSpeed) {
            newNormalVelocity = collisionNormal * minSeparationSpeed;
        }
    }
    else {
    // Already moving away, maintain normal velocity
        newNormalVelocity = normalVelocity;
    }

    // Apply friction to tangential component
    glm::vec3 newTangentialVelocity = tangentialVelocity * TANGENTIAL_FRICTION;

    // Combine components for final velocity
    velocity = newNormalVelocity + newTangentialVelocity;

    // Cap maximum reaction speed to prevent extreme bounces
    float newSpeed = glm::length(velocity);
    float maxReactionSpeed = std::min(BumperCar::MAX_SPEED * 1.2f, impactSpeed * 1.2f);
    if (newSpeed > maxReactionSpeed) {
        velocity = glm::normalize(velocity) * maxReactionSpeed;
    }

    // Apply angular impulse based on impact angle and position
    float impactAngle = std::abs(glm::acos(glm::clamp(headOnFactor, -1.0f, 1.0f)));
    float spinFactor = std::sin(impactAngle) * impactSpeed * 0.4f; // Reduced spin factor

    // Determine rotation direction based on cross product
    float rotationDirection = (glm::cross(velocity, collisionNormal).y > 0.0f) ? 1.0f : -1.0f;

    // Calculate and apply angular impulse with limits
    float angularImpulse = rotationDirection * spinFactor;
    float newAngularVelocity = angularVelocity + angularImpulse;

    // Limit maximum angular velocity to prevent excessive spinning
    angularVelocity = std::min(std::max(newAngularVelocity, -BumperCar::MAX_ANGULAR_SPEED * 0.8f), BumperCar::MAX_ANGULAR_SPEED * 0.8f);

    // Visual feedback - shake effect proportional to impact
    float shakeStrength = std::min(impactSpeed * 0.15f, 1.0f); // Reduced shake intensity
    StartShakeEffect(shakeStrength);
}

void BumperCar::UpdatePhysics(float deltaTime) {
    float speed = glm::length(velocity);

    // Apply friction based on whether turning
    float currentFriction = isTurning ? BumperCar::TURN_FRICTION : BumperCar::FRICTION;

    // Apply friction to linear velocity
    if (speed > 0.0f) {
        glm::vec3 frictionForce = -glm::normalize(velocity) * currentFriction * deltaTime;
        if (glm::length(frictionForce) > speed) {
            velocity = glm::vec3(0.0f);
        }
        else {
            velocity += frictionForce;
        }
    }

    // Apply damping to angular velocity
    if (std::abs(angularVelocity) > 0.0f) {
        float angularFriction = BumperCar::ANGULAR_DAMPING * deltaTime;
        angularVelocity *= (1.0f - angularFriction);

        // If angular velocity is very small, set it to zero
        if (std::abs(angularVelocity) < 0.01f) {
            angularVelocity = 0.0f;
        }
    }
}

void BumperCar::Accelerate(float deltaTime) {
    glm::vec3 forward = GetForwardVector();
    float currentSpeed = glm::length(velocity);

    // Gradual acceleration
    if (currentSpeed < MAX_SPEED) {
        // Soft acceleration curve
        float accelerationFactor = 1.0f - (currentSpeed / MAX_SPEED);
        velocity += forward * (ACCELERATION * accelerationFactor) * deltaTime;
    }

    // Limit maximum speed
    float speed = glm::length(velocity);
    if (speed > MAX_SPEED) {
        velocity = glm::normalize(velocity) * MAX_SPEED;
    }
}

void BumperCar::Brake(float deltaTime) {
    glm::vec3 forward = GetForwardVector();
    float currentSpeed = glm::length(velocity);
    float dotProduct = glm::dot(glm::normalize(velocity), forward);

    // If moving forward and no acceleration, gradually slow down
    if (currentSpeed > 0.1f && dotProduct > 0) {
        velocity *= (1.0f - (FRICTION * 1.5f * deltaTime));

        // If speed is very low, stop completely
        if (currentSpeed < 0.1f) {
            velocity = glm::vec3(0.0f);
        }
    }
    // If stopped or nearly stopped, start reversing
    else if (currentSpeed <= 0.1f) {
        // Reverse acceleration with soft curve
        float reverseAccelerationFactor = 1.0f - (std::abs(currentSpeed) / MAX_SPEED);
        velocity -= forward * (ACCELERATION * reverseAccelerationFactor) * deltaTime;
    }

    // Limit reverse speed
    float speed = glm::length(velocity);
    float maxReverseSpeed = MAX_SPEED * 0.8f;
    if (speed > maxReverseSpeed) {
        velocity = glm::normalize(velocity) * maxReverseSpeed;
    }
}

void BumperCar::TurnLeft(float deltaTime) {
    ApplyTurnForces(deltaTime, 1.0f);
}

void BumperCar::TurnRight(float deltaTime) {
    ApplyTurnForces(deltaTime, -1.0f);
}

void BumperCar::ApplyTurnForces(float deltaTime, float direction) {
    float speed = glm::length(velocity);

    // Pivot turning: rotation happens around back wheels
    if (speed > MIN_SPEED_FOR_TURN) {
        isTurning = true;

        // More aggressive turning at lower speeds, less at higher speeds
        float turnSpeedFactor = 1.0f - (speed / MAX_SPEED);
        angularVelocity = direction * TURN_SPEED * turnSpeedFactor;

        // Reduce speed more significantly when turning
        velocity *= (1.0f - (deltaTime * TURN_FRICTION * 0.5f));
    }
    else if (speed <= MIN_SPEED_FOR_TURN) {
        // Tight rotation when nearly stationary
        angularVelocity = direction * TURN_SPEED * 2.0f;
    }
}

glm::vec3 BumperCar::GetForwardVector() const {
    float radians = glm::radians(rotation);
    return glm::normalize(glm::vec3(-sin(radians), 0.0f, -cos(radians)));
}

void BumperCar::HandleWallCollision(const glm::vec3& wallNormal) {
    float impactSpeed = glm::length(velocity);

    // Get the impact angle between velocity and wall normal
    glm::vec3 normalizedVelocity = glm::normalize(velocity);
    float impactAngle = glm::degrees(std::acos(glm::dot(normalizedVelocity, wallNormal)));

    // Calculate reflection vector with increased bounce for rubber physics
    glm::vec3 reflected = glm::reflect(velocity, wallNormal);

    // Increase bounce effect based on speed and angle
    float bounceMultiplier = 1.0f;
    if (impactAngle > 45.0f) {
        bounceMultiplier = 1.2f; // More bounce for sharper angles
    }

    // Apply bouncy rubber physics
    velocity = reflected * BumperCar::BOUNCE_FACTOR * bounceMultiplier;

    // Apply rotational force based on impact angle
    if (impactAngle > 30.0f) {
        float rotationDirection = (glm::cross(normalizedVelocity, wallNormal).y > 0.0f) ? 1.0f : -1.0f;
        float angularImpulse = (impactAngle / 90.0f) * impactSpeed * 0.5f;
        angularVelocity += rotationDirection * angularImpulse * BumperCar::MAX_ANGULAR_SPEED;
    }

    // Trigger shake effect if impact is strong enough, Ive based that on the speed of the car
    if (impactSpeed > 2.0f) {
        StartShakeEffect(impactSpeed);
    }
}

bool BumperCar::CheckArenaCollision(const glm::vec3& newPosition, glm::vec3& wallNormal) {
    // Create rotation matrix for car's orientation
    float carRotation = glm::radians(this->rotation);
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));

    // Calculate offset in car's local space
    glm::vec3 localOffset(COLLISION_OFFSET_X, COLLISION_OFFSET_Y, COLLISION_OFFSET_Z);

    // Transform offset to world space
    glm::vec4 worldOffset = rotationMatrix * glm::vec4(localOffset, 1.0f);

    // Calculate collision box center
    glm::vec3 boxCenter = newPosition + glm::vec3(worldOffset);

    // Box half-dimensions
    float halfLength = COLLISION_BOX_LENGTH * 0.5f;
    float halfWidth = COLLISION_BOX_WIDTH * 0.5f;

    // Calculate the corners in local space
    std::vector<glm::vec2> corners = {
        glm::vec2(-halfWidth, -halfLength),  // Back left
        glm::vec2(halfWidth, -halfLength),   // Back right
        glm::vec2(halfWidth, halfLength),    // Front right
        glm::vec2(-halfWidth, halfLength)    // Front left
    };

    // Rotate corners to match car orientation
    for (auto& corner : corners) {
        float x = corner.x;
        float z = corner.y;  // Using y in 2D for z in 3D
        corner.x = x * cos(carRotation) - z * sin(carRotation);
        corner.y = x * sin(carRotation) + z * cos(carRotation);
        // Add box center position
        corner += glm::vec2(boxCenter.x, boxCenter.z);
    }

    // Arena boundaries
    const float ARENA_HALF_WIDTH = 15.0f - WALL_THICKNESS;
    const float ARENA_HALF_LENGTH = 15.0f - WALL_THICKNESS;

    // Check each corner against arena boundaries
    for (const auto& corner : corners) {
        if (corner.x > ARENA_HALF_WIDTH) {
            wallNormal = glm::vec3(-1.0f, 0.0f, 0.0f);
            return true;
        }
        if (corner.x < -ARENA_HALF_WIDTH) {
            wallNormal = glm::vec3(1.0f, 0.0f, 0.0f);
            return true;
        }
        if (corner.y > ARENA_HALF_LENGTH) {
            wallNormal = glm::vec3(0.0f, 0.0f, -1.0f);
            return true;
        }
        if (corner.y < -ARENA_HALF_LENGTH) {
            wallNormal = glm::vec3(0.0f, 0.0f, 1.0f);
            return true;
        }
    }

    return false;
}

void BumperCar::ApplyGravity(float deltaTime) {
    const float GROUND_LEVEL = -0.3f;
    position.y = GROUND_LEVEL;  // Keep car at ground level
}
//-----------------------------------------------------------------------------
// Car impact shake effect
//-----------------------------------------------------------------------------

void BumperCar::StartShakeEffect(float impactSpeed) {
    isShaking = true;
    shakeTimer = 0.0f;
    shakeDuration = 1.0f; // 1 second shake
    shakeIntensity = glm::min(impactSpeed * 0.05f, 0.15f); // Limit max shake
    shakeOffset = glm::vec3(0.0f);
}

void BumperCar::UpdateShake(float deltaTime) {
    if (!isShaking) return;

    shakeTimer += deltaTime;
    if (shakeTimer >= shakeDuration) {
        isShaking = false;
        shakeOffset = glm::vec3(0.0f);
        return;
    }

    // Calculate shake falloff
    float progress = shakeTimer / shakeDuration;
    float currentIntensity = shakeIntensity * (1.0f - progress);

    // Generate perlin noise or random shake
    shakeOffset = glm::vec3(
        (rand() / float(RAND_MAX) * 2.0f - 1.0f) * currentIntensity,
        (rand() / float(RAND_MAX) * 2.0f - 1.0f) * currentIntensity * 0.5f,
        (rand() / float(RAND_MAX) * 2.0f - 1.0f) * currentIntensity
    );
}

//-----------------------------------------------------------------------------
// Initialization & Resource Management
//-----------------------------------------------------------------------------
void BumperCar::Initialize() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(BumperCarVertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(BumperCarVertex), (void*)0);

    // Normal attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(BumperCarVertex), (void*)offsetof(BumperCarVertex, Normal));

    glBindVertexArray(0);
}

//-----------------------------------------------------------------------------
// Rendering Implementation
//-----------------------------------------------------------------------------

bool BumperCar::LoadMaterialLibrary(const std::string& mtlPath) {
    std::ifstream mtlFile(mtlPath);
    if (!mtlFile.is_open()) {
        std::cerr << "Could not open MTL file: " << mtlPath << std::endl;
        return false;
    }

    MaterialInfo currentMaterial;
    std::string line, keyword;

    while (std::getline(mtlFile, line)) {
        std::istringstream iss(line);
        iss >> keyword;

        if (keyword == "newmtl") {
            if (!currentMaterial.name.empty()) {
                materials.push_back(currentMaterial);
            }
            currentMaterial = MaterialInfo();
            std::getline(iss >> std::ws, currentMaterial.name);
            std::cout << "Loading material: " << currentMaterial.name << std::endl;
        }
        else if (keyword == "Kd") {
            iss >> currentMaterial.diffuseColor.x
                >> currentMaterial.diffuseColor.y
                >> currentMaterial.diffuseColor.z;
            std::cout << "Diffuse color loaded: "
                << currentMaterial.diffuseColor.x << ", "
                << currentMaterial.diffuseColor.y << ", "
                << currentMaterial.diffuseColor.z << std::endl;
        }
        else if (keyword == "Ks") {
            iss >> currentMaterial.specularColor.x
                >> currentMaterial.specularColor.y
                >> currentMaterial.specularColor.z;
        }
        else if (keyword == "Ns") {
            iss >> currentMaterial.shininess;
        }
        else if (keyword == "d" || keyword == "Tr") {
            iss >> currentMaterial.opacity;
        }
    }

    if (!currentMaterial.name.empty()) {
        materials.push_back(currentMaterial);
    }

    mtlFile.close();
    return !materials.empty();
}

//-----------------------------------------------------------------------------
// Model Loading Implementation
//-----------------------------------------------------------------------------
bool BumperCar::LoadOBJModel(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << path << std::endl;
        return false;
    }

    std::vector<glm::vec3> tempVertices;
    std::vector<glm::vec3> tempNormals;
    std::map<std::string, unsigned int> uniqueVertices;

    std::string mtlPath = path.substr(0, path.find_last_of('.')) + ".mtl";
    if (!LoadMaterialLibrary(mtlPath)) {
        std::cerr << "Failed to load material library: " << mtlPath << std::endl;
        return false;
    }

    MeshSegment currentSegment;
    currentSegment.materialIndex = 0;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            // Scale and flip the model
            vertex *= 0.2f;
            vertex.z = -vertex.z;  // Flip Z-axis
            vertex.x = -vertex.x;  // Flip X-axis for proper orientation
            tempVertices.push_back(vertex);
        }
        else if (type == "vn") {
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            // Flip normal vectors to match vertex transformations
            normal.z = -normal.z;
            normal.x = -normal.x;
            tempNormals.push_back(glm::normalize(normal));
        }
        else if (type == "usemtl") {
            if (!currentSegment.indices.empty()) {
                meshSegments.push_back(currentSegment);
                currentSegment.indices.clear();
            }

            std::string materialName;
            iss >> materialName;

            for (size_t i = 0; i < materials.size(); i++) {
                if (materials[i].name == materialName) {
                    currentSegment.materialIndex = i;
                    break;
                }
            }
        }
        else if (type == "f") {
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;

            // Reverse face winding order to match flipped geometry
            std::vector<std::string> vertexData = { v1, v3, v2 };

            for (const auto& vertex : vertexData) {
                if (uniqueVertices.count(vertex) == 0) {
                    BumperCarVertex newVertex;
                    std::vector<std::string> indices;
                    std::stringstream ss(vertex);
                    std::string index;

                    while (std::getline(ss, index, '/')) {
                        indices.push_back(index);
                    }

                    if (!indices.empty() && !indices[0].empty()) {
                        int posIndex = std::stoi(indices[0]) - 1;
                        newVertex.Position = tempVertices[posIndex];
                    }

                    if (indices.size() > 2 && !indices[2].empty()) {
                        int normalIndex = std::stoi(indices[2]) - 1;
                        newVertex.Normal = tempNormals[normalIndex];
                    }

                    uniqueVertices[vertex] = vertices.size();
                    vertices.push_back(newVertex);
                }
                currentSegment.indices.push_back(uniqueVertices[vertex]);
            }
        }
    }

    if (!currentSegment.indices.empty()) {
        meshSegments.push_back(currentSegment);
    }

    indices.clear();
    for (const auto& segment : meshSegments) {
        indices.insert(indices.end(), segment.indices.begin(), segment.indices.end());
    }

    std::cout << "Model Loading Results:" << std::endl;
    std::cout << "Vertices loaded: " << vertices.size() << std::endl;
    std::cout << "Indices loaded: " << indices.size() << std::endl;
    std::cout << "Materials loaded: " << materials.size() << std::endl;
    std::cout << "Mesh segments: " << meshSegments.size() << std::endl;

    return !vertices.empty() && !indices.empty() && !materials.empty();
}

void BumperCar::Render(Shader& shader) {
    shader.use();

    glm::mat4 model = glm::mat4(1.0f);
    glm::vec3 finalPosition = position + shakeOffset;
    model = glm::translate(model, finalPosition);
    model = glm::rotate(model, glm::radians(rotation), glm::vec3(0.0f, 1.0f, 0.0f));
    shader.setMat4("model", model);

    glBindVertexArray(VAO);

    GLuint offset = 0;
    for (const auto& segment : meshSegments) {
        if (segment.materialIndex >= 0 && segment.materialIndex < materials.size()) {
            const auto& material = materials[segment.materialIndex];

            shader.setVec3("material.diffuse", material.diffuseColor);
            shader.setVec3("material.specular", material.specularColor);
            shader.setFloat("material.shininess", material.shininess);
            shader.setFloat("material.opacity", 1.0f);

            glDrawElements(GL_TRIANGLES,
                static_cast<GLsizei>(segment.indices.size()),
                GL_UNSIGNED_INT,
                reinterpret_cast<const void*>(offset));

            offset += segment.indices.size() * sizeof(GLuint);
        }
    }

    glBindVertexArray(0);
}

//-----------------------------------------------------------------------------
// Texture Loading Implementation
//-----------------------------------------------------------------------------
unsigned int BumperCar::LoadTexture(const char* path) {
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

        // Update texture parameters for better mapping
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Enable anisotropic filtering if available
        float maxAniso = 0.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);

        stbi_image_free(data);
    }
    else {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}