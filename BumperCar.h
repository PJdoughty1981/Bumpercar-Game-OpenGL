/**
 * BumperCar.h
 *
 * This is the heart of my bumper car game - a comprehensive class that handles
 * all the physics, rendering, and behavior of the bumper cars. I designed it with
 * realistic physics in mind, implementing momentum-based movement, collision responses,
 * and visual effects like car shake on impact.
 *
 * Key features I've implemented:
 * - Physically-based movement with acceleration, friction, and momentum
 * - Realistic turning mechanics with reduced speed during turns
 * - Collision detection with arena walls and other cars
 * - Impact response with appropriate bounce and spin
 * - Vehicle shake effect for visual feedback on collisions
 * - 3D model loading and rendering with materials
 *
 * The physics values are fine-tuned for fun gameplay rather than strict realism.
 * I spent a lot of time balancing variables like friction, bounce factor, and
 * maximum speeds to make the cars feel fun to drive but still somewhat realistic.
 */
#pragma once

 //-----------------------------------------------------------------------------
 // Standard Library Includes
 //-----------------------------------------------------------------------------
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>

//-----------------------------------------------------------------------------
// External Library Includes
//-----------------------------------------------------------------------------
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include "Shader.h"

//-----------------------------------------------------------------------------
// Structures
//-----------------------------------------------------------------------------
/**
 * Vertex structure for bumper car models
 *
 * Contains position and normal data for rendering with lighting.
 */
struct BumperCarVertex {
    glm::vec3 Position;  // Vertex position in model space
    glm::vec3 Normal;    // Surface normal for lighting calculations
};

/**
 * Material properties for car rendering
 *
 * Stores visual properties defined in MTL files that determine
 * how the car appears when rendered.
 */
struct MaterialInfo {
    std::string name;          // Material name from MTL file
    glm::vec3 diffuseColor;    // Main surface color
    glm::vec3 specularColor;   // Highlight color
    float shininess;           // Surface smoothness
    float opacity;             // Transparency 
};

/**
 * Segment of a mesh with a specific material
 *
 * Cars often consist of multiple parts with different materials.
 * This structure lets me organize those parts and render them correctly.
 */
struct MeshSegment {
    std::vector<unsigned int> indices;  // Vertex indices for this segment
    int materialIndex;                  // Which material to use for rendering
};

/**
 * Color scheme for car customization
 *
 * Allows creating different color variants for cars.
 */
struct CarColorScheme {
    glm::vec3 primaryColor;    // Main body color
    glm::vec3 secondaryColor;  // Secondary color for details
    glm::vec3 accentColor;     // Accent color for smaller details
};

//-----------------------------------------------------------------------------
// BumperCar Class Declaration & Editable movement / collision variables 
//-----------------------------------------------------------------------------
class BumperCar {
public:
    // Car movement physics constants - I've tuned these for fun gameplay
    static constexpr float MAX_SPEED = 6.0f;         // Maximum forward speed
    static constexpr float ACCELERATION = 3.0f;      // How quickly car gains speed
    static constexpr float TURN_SPEED = 30.0f;       // How quickly car can turn
    static constexpr float FRICTION = 0.9f;          // Ground friction when driving straight
    static constexpr float TURN_FRICTION = 1.2f;     // Increased friction when turning
    static constexpr float BOUNCE_FACTOR = 0.2f;     // How bouncy the car is on collision
    static constexpr float ANGULAR_DAMPING = 0.9f;   // How quickly spinning slows down
    static constexpr float MAX_ANGULAR_SPEED = 30.0f; // Maximum rotation speed
    static constexpr float MIN_SPEED_FOR_TURN = 0.4f; // Minimum speed needed for normal turning

    // Left headlight position - offsets from car center
    static constexpr float LEFT_HEADLIGHT_OFFSET_X = -0.65f;
    static constexpr float LEFT_HEADLIGHT_OFFSET_Y = 0.65f;
    static constexpr float LEFT_HEADLIGHT_OFFSET_Z = -1.6f;

    // Right headlight position - offsets from car center
    static constexpr float RIGHT_HEADLIGHT_OFFSET_X = -0.25f;
    static constexpr float RIGHT_HEADLIGHT_OFFSET_Y = 0.65f;
    static constexpr float RIGHT_HEADLIGHT_OFFSET_Z = -1.6f;

    // Rotation point configuration
    static constexpr float TURN_REAR_OFFSET = -0.48f;     // Back from center point
    static constexpr float TURN_LATERAL_OFFSET = -0.5f;   // Side offset for rotation point

    // Car antenna effect marker
    static constexpr float ANTENNA_OFFSET_X = -0.442f;    // Left/right position of antenna
    static constexpr float ANTENNA_OFFSET_Y = 3.545f;     // Height of antenna
    static constexpr float ANTENNA_OFFSET_Z = 0.45f;      // Front/back position of antenna

    // Car Collision box configuration
    static constexpr float COLLISION_BOX_LENGTH = 2.4f;   // Length of collision box
    static constexpr float COLLISION_BOX_WIDTH = 1.2f;    // Width of collision box
    static constexpr float COLLISION_BOX_HEIGHT = 1.0f;   // Height of collision box
    static constexpr float COLLISION_OFFSET_X = -0.45f;   // Left/right offset of collision box
    static constexpr float COLLISION_OFFSET_Y = 0.9f;     // Height offset of collision box
    static constexpr float COLLISION_OFFSET_Z = -0.4f;    // Front/back offset of collision box
    static constexpr float WALL_THICKNESS = 1.2f;         // Thickness of arena walls for collision

    /**
     * Creates a bumper car with the specified 3D model
     *
     * Loads the 3D model from the OBJ file but doesn't initialize OpenGL
     * resources yet. Call Initialize() to complete setup.
     *
     * modelPath Path to the OBJ file for this car
     */
    BumperCar(const std::string& modelPath);

    /**
     * Initializes OpenGL resources for rendering
     *
     * Sets up vertex buffers, index buffers, and prepares the car
     * for rendering. Call this after creating the car and before rendering.
     */
    void Initialize();

    /**
     * Renders the car with the specified shader
     *
     * Draws the car model with all its materials, applying the current
     * transformation (position, rotation) and any shake effect.
     *
     * shader Shader to use for rendering
     */
    void Render(Shader& shader);

    /**
     * Applies gravity to keep the car on the ground
     *
     * A simple function to ensure the car stays at ground level.
     *
     * deltaTime Time since last update
     */
    void ApplyGravity(float deltaTime);

    /**
     * Updates car physics and movement
     *
     * Main update function that handles:
     * - Applying forces based on input
     * - Updating position and rotation
     * - Applying friction and damping
     * - Checking for collisions
     * - Updating visual effects like shake
     *
     * deltaTime Time since last update
     */
    void UpdateMovement(float deltaTime);

    /**
     * Accelerates the car forward
     *
     * Increases speed in the direction the car is facing,
     * up to the maximum speed limit.
     *
     * deltaTime Time since last update
     */
    void Accelerate(float deltaTime);

    /**
     * Applies brakes or reverses the car
     *
     * If moving forward, slows the car down. If stopped
     * or nearly stopped, starts reversing.
     *
     * deltaTime Time since last update
     */
    void Brake(float deltaTime);

    /**
     * Turns the car to the left
     *
     * Applies left steering input, strength varies based on speed.
     *
     * deltaTime Time since last update
     */
    void TurnLeft(float deltaTime);

    /**
     * Turns the car to the right
     *
     * Applies right steering input, strength varies based on speed.
     *
     * deltaTime Time since last update
     */
    void TurnRight(float deltaTime);

    /**
     * Gets the car's current position
     *
     * return Current position in world space
     */
    glm::vec3 GetPosition() const { return position; }

    /**
     * Gets the car's current rotation angle
     *
     * return Rotation angle in degrees
     */
    float GetRotation() const { return rotation; }

    /**
     * Gets the car's current velocity vector
     *
     * return Current velocity in world space
     */
    glm::vec3 GetVelocity() const { return velocity; }

    /**
     * Gets the car's current angular velocity
     *
     * return Angular velocity in degrees per second
     */
    float GetAngularVelocity() const { return angularVelocity; }

    /**
     * Gets the car's forward direction vector
     *
     * return Normalized forward vector in world space
     */
    glm::vec3 GetForwardVector() const;

    /**
     * Gets the car's current speed
     *
     * return Speed in units per second
     */
    float GetSpeed() const { return currentSpeed; }

    /**
     * Sets the car's position
     *
     * pos New position in world space
     */
    void SetPosition(const glm::vec3& pos) { position = pos; }

    /**
     * Sets the car's rotation
     *
     * rot New rotation angle in degrees
     */
    void SetRotation(float rot) { rotation = rot; }

    /**
     * Sets the car's velocity
     *
     * vel New velocity vector
     */
    void SetVelocity(const glm::vec3& vel) { velocity = vel; }

    /**
     * Sets the car's angular velocity
     *
     * angVel New angular velocity in degrees per second
     */
    void SetAngularVelocity(float angVel) { angularVelocity = angVel; }

    /**
     * Triggers a shake effect when the car is hit
     *
     * Creates a visual feedback effect where the car shakes after
     * an impact. Intensity scales with impact speed.
     *
     * impactSpeed Speed of impact
     */
    void StartShakeEffect(float impactSpeed);

    /**
     * Updates the shake effect animation
     *
     * Gradually reduces shake over time until it stops.
     *
     * deltaTime Time since last update
     */
    void UpdateShake(float deltaTime);

    /**
     * Gets the current shake offset
     *
     * return Offset to apply to position for shake effect
     */
    glm::vec3 GetShakeOffset() const { return shakeOffset; }

    /**
     * Sets the car's color scheme
     *
     * scheme Color scheme to apply
     */
    void SetColorScheme(const CarColorScheme& scheme) { colorScheme = scheme; }

    /**
     * Handles collision with another object
     *
     * Applies physics response when colliding with another car or wall,
     * including bouncing, angular impulse, and visual shake effect.
     *
     * collisionNormal Normalized direction vector of the collision
     */
    void HandleCollisionWith(const glm::vec3& collisionNormal);

private:
    // 3D model data
    std::vector<BumperCarVertex> vertices;      // Vertex data
    std::vector<unsigned int> indices;          // Index data
    std::vector<MaterialInfo> materials;        // Materials
    std::vector<MeshSegment> meshSegments;      // Mesh segments
    std::map<std::string, int> materialMap;     // Material lookup
    std::string currentMaterialName;            // Current material
    CarColorScheme colorScheme;                 // Car colors

    // OpenGL rendering objects
    unsigned int VAO, VBO, EBO;                 // Vertex arrays and buffers

    // Physics state
    glm::vec3 position;                         // World position
    float rotation;                             // Rotation angle (degrees)
    float currentSpeed;                         // Current speed (units/sec)
    glm::vec3 velocity;                         // Velocity vector
    float angularVelocity;                      // Rotation speed (deg/sec)
    bool isColliding;                           // Currently in collision?
    bool isTurning;                             // Currently turning?

    // Visual effect state
    bool isShaking;                             // Currently shaking?
    float shakeTimer;                           // Current shake time
    float shakeDuration;                        // How long shake lasts
    float shakeIntensity;                       // How strong the shake is
    glm::vec3 shakeOffset;                      // Current shake offset

    /**
     * Loads the 3D model from an OBJ file
     *
     * Parses vertices, normals, and faces from the OBJ file,
     * and loads associated materials from the MTL file.
     *
     * path Path to the OBJ file
     * return True if loading succeeded
     */
    bool LoadOBJModel(const std::string& path);

    /**
     * Loads a texture from file
     *
     * Creates and configures an OpenGL texture from an image file.
     *
     * path Path to the image file
     * return OpenGL texture ID
     */
    unsigned int LoadTexture(const char* path);

    /**
     * Handles collision with arena walls
     *
     * Special case of collision handling for the static walls.
     *
     * wallNormal Normal vector of the wall surface
     */
    void HandleWallCollision(const glm::vec3& wallNormal);

    /**
     * Updates physics properties like friction and damping
     *
     * Applies physical forces that slow the car down over time.
     *
     * deltaTime Time since last update
     */
    void UpdatePhysics(float deltaTime);

    /**
     * Checks if the car is colliding with arena walls
     *
     * Determines if the car's collision box intersects with any walls.
     *
     * newPosition Potential new position to check
     * wallNormal Output parameter for wall normal if collision occurs
     * return True if collision detected
     */
    bool CheckArenaCollision(const glm::vec3& newPosition, glm::vec3& wallNormal);

    /**
     * Applies turning forces to the car
     *
     * Internal helper that handles the physics of turning,
     * including adjusting speed and applying angular velocity.
     *
     * deltaTime Time since last update
     * direction Direction to turn (1.0 = left, -1.0 = right)
     */
    void ApplyTurnForces(float deltaTime, float direction);

    /**
     * Loads material definitions from MTL file
     *
     * Parses material properties like colors and shininess.
     *
     * mtlPath Path to the MTL file
     * return True if loading succeeded
     */
    bool LoadMaterialLibrary(const std::string& mtlPath);
};