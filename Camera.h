/**
 * Camera.h
 *
 * My versatile camera system with multiple viewing modes for the bumper car game.
 * I designed this to provide different perspectives that make the game more immersive
 * and fun to play. The camera can smoothly switch between free-flying, first-person,
 * third-person, and ground-level walking modes.
 *
 * Key features:
 * - FREE mode: Unrestricted flying camera for overview and debugging
 * - FIRST_PERSON mode: Driver's perspective from inside the car
 * - THIRD_PERSON mode: Follow camera that stays behind the car
 * - GROUND mode: Human-height "walking" camera with vertical constraints
 * - Collision detection to prevent clipping through walls and cars
 * - Smooth movement and rotation controls
 * - Mouse-based look control with configurable sensitivity
 */
#pragma once

 //-----------------------------------------------------------------------------
 // Includes
 //-----------------------------------------------------------------------------
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

// Forward declaration
class BumperCar;

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
// Camera constraints and movement parameters
const float MIN_HEIGHT = 0.5f;        // Minimum height from ground
const float MAX_ZOOM = 120.0f;        // Maximum zoom distance
const float MIN_ZOOM = 1.0f;          // Minimum zoom distance
const float YAW = -90.0f;             // Default camera rotation
const float PITCH = 0.0f;             // Default camera pitch
const float SPEED = 2.5f;             // Movement speed
const float SENSITIVITY = 0.1f;       // Mouse sensitivity
const float DEFAULT_ZOOM = 45.0f;     // Default zoom level

// Collision adjustment parameters
const float WALL_COLLISION_DISTANCE = 0.5f;    // Distance to keep from walls
const float WALL_THICKNESS = 1.2f;             // Wall thickness for collision

// Ground Camera human eye level constant
const float GROUND_CAMERA_HEIGHT = 1.7f;       // Human eye level for ground mode

// First person Camera initial position - offsets from car center
const float FIRST_PERSON_X_OFFSET = -0.45f;    // Left/right offset
const float FIRST_PERSON_Y_OFFSET = 1.49f;     // Vertical offset (eye height)
const float FIRST_PERSON_Z_OFFSET = 0.5f;      // Forward/back offset

// 3rd Person Camera configuration
const float THIRD_PERSON_DISTANCE = 6.0f;      // Distance behind target
const float THIRD_PERSON_X_OFFSET = 1.0f;      // Left/right offset
const float THIRD_PERSON_Y_OFFSET = 3.0f;      // Vertical offset (height)
const float THIRD_PERSON_Z_OFFSET = 0.0f;      // Forward/back offset
const float THIRD_PERSON_LOOK_AT_HEIGHT = 2.0f; // Height of look-at point

//-----------------------------------------------------------------------------
// Enums
//-----------------------------------------------------------------------------
/**
 * Camera movement directions
 *
 * Used to specify which direction to move the camera
 * in response to keyboard input.
 */
enum Camera_Movement {
    FORWARD,  // Move forward along view direction
    BACKWARD, // Move backward along view direction
    LEFT,     // Strafe left
    RIGHT,    // Strafe right
    UP,       // Move up along world Y axis
    DOWN      // Move down along world Y axis
};

//-----------------------------------------------------------------------------
// Camera Class Declaration
//-----------------------------------------------------------------------------
class Camera {
public:
    /**
     * Camera operation modes
     *
     * These determine how the camera behaves and what constraints apply.
     */
    enum class Mode {
        FREE,           // Free-flying camera with collision
        FIRST_PERSON,   // Fixed to driver position
        THIRD_PERSON,   // Fixed behind car
        GROUND          // Walking height with vertical constraint
    };

private:
    //-----------------------------------------------------------------------------
    // Member Variables
    //-----------------------------------------------------------------------------
    // Camera vectors
    glm::vec3 Position;       // Camera position in world space
    glm::vec3 Front;          // Direction camera is facing
    glm::vec3 Up;             // Camera up vector
    glm::vec3 Right;          // Camera right vector
    glm::vec3 WorldUp;        // World up direction (typically 0,1,0)
    glm::vec3 TargetPosition; // Position of followed object (for 3rd/1st person)

    // Camera orientation
    float Yaw;                // Horizontal rotation angle
    float Pitch;              // Vertical rotation angle

    // Camera parameters
    float MovementSpeed;      // How fast camera moves
    float MouseSensitivity;   // How sensitive mouse controls are
    float Zoom;               // Field of view / zoom level
    Mode currentMode;         // Current camera mode
    float collisionRadius;    // Radius for collision detection

    // Collision state
    bool isColliding;                // Currently colliding with something?
    std::vector<BumperCar*> allCars; // References to cars for collision checks

public:
    //-----------------------------------------------------------------------------
    // Constructor & Public Methods
    //-----------------------------------------------------------------------------
    /**
     * Creates a camera with specified initial position
     *
     * Initializes the camera with default values for orientation,
     * movement speed, sensitivity, and zoom level.
     *
     * position Initial position in world space
     */
    Camera(glm::vec3 position = glm::vec3(0.0f, 5.0f, 10.0f));

    /**
     * Gets the view matrix for rendering
     *
     * Calculates the view transformation matrix based on
     * current position and orientation.
     *
     * return View matrix for use in rendering
     */
    glm::mat4 GetViewMatrix() const;

    /**
     * Processes keyboard input for camera movement
     *
     * Moves the camera based on WASD or arrow keys, taking
     * into account collision detection and current mode.
     *
     * direction Direction to move in
     * deltaTime Time since last update
     */
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);

    /**
     * Processes mouse movement for camera rotation
     *
     * Rotates the camera based on mouse movement, taking into
     * account sensitivity settings and pitch constraints.
     *
     * xoffset Horizontal mouse movement
     * yoffset Vertical mouse movement
     * constrainPitch Whether to limit vertical rotation
     */
    void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);

    /**
     * Processes mouse scroll wheel for zoom
     *
     * Adjusts the field of view (zoom) based on scroll wheel input.
     *
     * yoffset Scroll amount
     */
    void ProcessMouseScroll(float yoffset);

    /**
     * Changes the camera's operation mode
     *
     * Switches between different camera behaviors (free, first-person,
     * third-person, ground) and sets up appropriate initial settings.
     *
     * newMode Mode to switch to
     */
    void UpdateMode(Mode newMode);

    /**
     * Updates camera for third-person view
     *
     * Positions the camera behind the target car at the specified
     * distance and angle.
     *
     * targetPos Position of the car to follow
     * targetRotation Rotation of the car to follow
     */
    void UpdateThirdPerson(const glm::vec3& targetPos, float targetRotation);

    /**
     * Updates camera for first-person view
     *
     * Positions the camera at the driver's perspective inside
     * the target car.
     *
     * targetPos Position of the car to view from
     * targetRotation Rotation of the car to view from
     */
    void UpdateFirstPerson(const glm::vec3& targetPos, float targetRotation);

    /**
     * Checks if proposed position would cause collision
     *
     * Determines if moving to the specified position would
     * cause the camera to collide with walls or cars.
     *
     * newPosition Position to check
     * return True if collision would occur
     */
    bool CheckCollision(const glm::vec3& newPosition) const;

    /**
     * Sets cars to check for collision
     *
     * Updates the list of bumper cars that the camera should
     * avoid colliding with.
     *
     * cars Vector of pointers to bumper cars
     */
    void SetAllCars(const std::vector<BumperCar*>& cars) { allCars = cars; }

    //-----------------------------------------------------------------------------
    // Getters & Setters
    //-----------------------------------------------------------------------------
    /**
     * Gets current zoom/FOV level
     *
     * return Current zoom value
     */
    float GetZoom() const { return Zoom; }

    /**
     * Gets camera position
     *
     * return Current position in world space
     */
    glm::vec3 GetPosition() const { return Position; }

    /**
     * Gets current camera mode
     *
     * return Current operation mode
     */
    Mode GetCurrentMode() const { return currentMode; }

    /**
     * Sets camera position
     *
     * pos New position in world space
     */
    void SetPosition(const glm::vec3& pos) { Position = pos; }

    /**
     * Sets camera rotation
     *
     * yaw Horizontal rotation angle
     * pitch Vertical rotation angle
     */
    void SetRotation(float yaw, float pitch) {
        Yaw = yaw;
        Pitch = pitch;
        updateCameraVectors();
    }

private:
    //-----------------------------------------------------------------------------
    // Private Helper Methods
    //-----------------------------------------------------------------------------
    /**
     * Updates derived camera vectors
     *
     * Recalculates Front, Right, and Up vectors based on
     * current Yaw and Pitch angles.
     */
    void updateCameraVectors();

    /**
     * Ensures camera stays at ground height
     *
     * Used in GROUND mode to maintain consistent eye level.
     */
    void maintainGroundHeight();

    /**
     * Handles collision with environment boundaries
     *
     * Prevents camera from moving outside the arena or below
     * minimum height.
     *
     * newPosition Position to check and adjust
     */
    void handleEnvironmentCollision(glm::vec3& newPosition);

    /**
     * Checks for collision with cars
     *
     * Determines if the camera would intersect with any bumper cars.
     *
     * position Position to check
     * return True if collision would occur
     */
    bool checkCarCollision(const glm::vec3& position) const;

    /**
     * Checks for collision with arena walls
     *
     * Determines if the camera would intersect with arena boundaries.
     *
     * position Position to check
     * return True if collision would occur
     */
    bool checkWallCollision(const glm::vec3& position) const;
};