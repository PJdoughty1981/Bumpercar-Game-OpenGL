/**
 * Camera.cpp
 *
 * Implementation of my versatile camera system for the bumper car game.
 * This file contains the complete logic for all four camera modes and the
 * smooth transitions between them. I've spent a lot of time tuning the
 * parameters to make each mode feel right for its purpose.
 *
 * The camera system was one of the more challenging parts of the project,
 * especially getting the first-person and third-person modes to follow
 * the car naturally while maintaining smooth movement. The collision detection
 * for the camera was also tricky to get right, but it really enhances
 * the experience when the camera can't clip through walls.
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include "Camera.h"
#include "BumperCar.h"
#include <algorithm>

 //-----------------------------------------------------------------------------
 // Constructor Implementation
 //-----------------------------------------------------------------------------
 /**
  * Initializes camera with default settings
  *
  * Sets up camera position, orientation, and movement parameters
  * with reasonable defaults. The camera starts in FREE mode.
  *
  * position Initial position in world space
  */
Camera::Camera(glm::vec3 position) :
    Position(position),
    Front(glm::vec3(0.0f, 0.0f, -1.0f)),
    WorldUp(glm::vec3(0.0f, 1.0f, 0.0f)),
    TargetPosition(glm::vec3(0.0f)),
    Yaw(YAW),
    Pitch(PITCH),
    MovementSpeed(SPEED),
    MouseSensitivity(SENSITIVITY),
    Zoom(DEFAULT_ZOOM),
    currentMode(Mode::FREE),
    isColliding(false),
    collisionRadius(1.3f)  // Size of collision sphere around camera
{
    updateCameraVectors();
}

//-----------------------------------------------------------------------------
// Public Methods Implementation
//-----------------------------------------------------------------------------
/**
 * Calculates the view matrix for rendering
 *
 * Creates the view transformation matrix that positions and orients
 * the camera for rendering. This is used in the rendering pipeline
 * to transform world-space coordinates to view space.
 *
 * return The view matrix
 */
glm::mat4 Camera::GetViewMatrix() const {
    return glm::lookAt(Position, Position + Front, Up);
}

/**
 * Processes keyboard input for camera movement
 *
 * Handles WASD-style movement controls, including:
 * - Moving forward/backward along view direction
 * - Strafing left/right
 * - Moving up/down along world Y axis (in FREE mode only)
 *
 * Also handles collision detection to prevent moving through walls
 * and other objects.
 *
 * direction Direction of movement
 * deltaTime Time since last update
 */
void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime) {
    // Don't process keyboard input for camera in first or third person modes
    // as these are driven by the car's position
    if (currentMode == Mode::FIRST_PERSON || currentMode == Mode::THIRD_PERSON) {
        return;
    }

    float velocity = MovementSpeed * deltaTime;
    glm::vec3 newPosition = Position;

    // Calculate new position based on input direction
    switch (direction) {
    case FORWARD:
        newPosition += Front * velocity;
        break;
    case BACKWARD:
        newPosition -= Front * velocity;
        break;
    case LEFT:
        newPosition -= Right * velocity;
        break;
    case RIGHT:
        newPosition += Right * velocity;
        break;
    case UP:
        // Only allow vertical movement in FREE mode
        if (currentMode == Mode::FREE) {
            newPosition += Up * velocity;
        }
        break;
    case DOWN:
        if (currentMode == Mode::FREE) {
            newPosition -= Up * velocity;
        }
        break;
    }

    // Handle collision and height constraints
    handleEnvironmentCollision(newPosition);

    // Specific handling for ground camera mode
    if (currentMode == Mode::GROUND) {
        // Reset Y to ground height before collision check
        newPosition.y = GROUND_CAMERA_HEIGHT;

        // Check collision while maintaining ground height
        if (!CheckCollision(newPosition)) {
            Position = newPosition;
        }
    }
    // Free camera mode keeps existing behavior
    else if (currentMode == Mode::FREE) {
        if (!CheckCollision(newPosition)) {
            Position = newPosition;
        }
    }
}

/**
 * Processes mouse movement for camera rotation
 *
 * Updates Yaw and Pitch based on mouse movement, then recalculates
 * camera orientation vectors. Also handles constraining the pitch
 * to avoid gimbal lock.
 *
 * xoffset Horizontal mouse movement
 * yoffset Vertical mouse movement
 * constrainPitch Whether to limit vertical rotation
 */
void Camera::ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
    // Apply sensitivity
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    // Update Euler angles
    Yaw += xoffset;
    Pitch += yoffset;

    // Prevent looking too far up or down
    if (constrainPitch) {
        Pitch = glm::clamp(Pitch, -89.0f, 89.0f);
    }

    // Update camera vectors based on new angles
    updateCameraVectors();
}

/**
 * Processes mouse scroll for zoom adjustment
 *
 * Changes the field of view (zoom) based on scroll wheel input.
 * Only works in FREE mode since other modes have fixed perspectives.
 *
 * yoffset Vertical scroll amount
 */
void Camera::ProcessMouseScroll(float yoffset) {
    // Only allow zoom in FREE mode
    if (currentMode == Mode::FREE) {
        Zoom -= yoffset;
        Zoom = glm::clamp(Zoom, MIN_ZOOM, MAX_ZOOM);
    }
}

/**
 * Changes the camera's operation mode
 *
 * Switches between different camera behaviors and sets up
 * appropriate initial settings for each mode.
 *
 * newMode Mode to switch to
 */
void Camera::UpdateMode(Mode newMode) {
    currentMode = newMode;
    switch (newMode) {
    case Mode::FREE:
        // Keep current position but enable full movement
        break;
    case Mode::GROUND:
        // Set to ground level and maintain it
        Position.y = GROUND_CAMERA_HEIGHT;
        Pitch = 0.0f;  // Look horizontally
        updateCameraVectors();
        break;
    case Mode::FIRST_PERSON:
        // Will be updated by UpdateFirstPerson
        Pitch = 0.0f;
        updateCameraVectors();
        break;
    case Mode::THIRD_PERSON:
        // Will be updated by UpdateThirdPerson
        Position = glm::vec3(0.0f, 0.0f, 0.0f);
        Pitch = -15.0f;  // Slight downward angle to see the car better
        Yaw = -90.0f;    // Look forward
        updateCameraVectors();
        break;
    }
}

/**
 * Updates camera position for third-person view
 *
 * Positions the camera behind the target car at a configurable distance
 * and height. The camera smoothly follows the car as it moves and turns.
 *
 * targetPos Position of the car
 * targetRotation Rotation angle of the car in degrees
 */
void Camera::UpdateThirdPerson(const glm::vec3& targetPos, float targetRotation) {
    if (currentMode != Mode::THIRD_PERSON) return;

    TargetPosition = targetPos;

    // Calculate camera position relative to target
    float radians = glm::radians(targetRotation);
    glm::vec3 offset = glm::vec3(
        sin(radians) * THIRD_PERSON_DISTANCE + THIRD_PERSON_X_OFFSET,
        THIRD_PERSON_Y_OFFSET,
        cos(radians) * THIRD_PERSON_DISTANCE + THIRD_PERSON_Z_OFFSET
    );

    Position = targetPos + offset;

    // Point camera at target (with configurable look-at height)
    glm::vec3 targetLookAt = targetPos + glm::vec3(0.0f, THIRD_PERSON_LOOK_AT_HEIGHT, 0.0f);
    Front = glm::normalize(targetLookAt - Position);
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}

/**
 * Updates camera position for first-person view
 *
 * Positions the camera at the driver's position inside the car.
 * The camera moves and rotates with the car, creating an immersive
 * driver perspective.
 *
 * targetPos Position of the car
 * targetRotation Rotation angle of the car in degrees
 */
void Camera::UpdateFirstPerson(const glm::vec3& targetPos, float targetRotation) {
    if (currentMode != Mode::FIRST_PERSON) return;

    TargetPosition = targetPos;

    // Fixed offset in the car's local space with configurable initial position
    glm::vec3 localOffset = glm::vec3(
        FIRST_PERSON_X_OFFSET,    // Left/right offset
        FIRST_PERSON_Y_OFFSET,    // Vertical offset
        FIRST_PERSON_Z_OFFSET     // Forward/back offset
    );

    // Create rotation matrix for the car's current rotation
    float radians = glm::radians(targetRotation);
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), radians, glm::vec3(0.0f, 1.0f, 0.0f));

    // Transform the local offset by the car's rotation
    glm::vec3 rotatedOffset = glm::vec3(rotationMatrix * glm::vec4(localOffset, 1.0f));

    // Set camera position fixed relative to the car
    Position = targetPos + rotatedOffset;

    // Ensure camera always looks forward 
    Front = glm::vec3(
        -sin(radians),   // X component (added negative)
        0.0f,            // Y component (level)
        -cos(radians)    // Z component (added negative)
    );

    // Recalculate Right and Up vectors to maintain proper orientation
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}

/**
 * Checks if camera would colide with a car
 *
 * Tests if the camera's collision sphere intersects with any
 * of the bumper cars' bounding boxes.
 *
 * position Position to check
 * return True if collision detected
 */
bool Camera::checkCarCollision(const glm::vec3& position) const {
    for (const auto& car : allCars) {
        if (!car) continue;

        glm::vec3 carPos = car->GetPosition();
        float quickCheck = glm::length(position - carPos);

        // Rough radius check first for efficiency
        if (quickCheck < 3.0f) {
            float carRotation = glm::radians(car->GetRotation());
            glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));

            // Box dimensions plus padding
            float boxHalfWidth = 1.2f;
            float boxHalfLength = 2.4f;

            // Transform camera position to car's local space
            glm::vec3 toCam = position - carPos;
            glm::vec3 localSpace = glm::vec3(
                glm::dot(toCam, glm::vec3(rotationMatrix[0])),
                0.0f,
                glm::dot(toCam, glm::vec3(rotationMatrix[2]))
            );

            // Check if camera sphere intersects car box
            if (std::abs(localSpace.x) < boxHalfWidth + collisionRadius &&
                std::abs(localSpace.z) < boxHalfLength + collisionRadius) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Checks if camera would collide with arena walls
 *
 * Tests if the camera's collision sphere intersects with any
 * of the arena wall boundaries.
 *
 * position Position to check
 * return True if collision detected
 */
bool Camera::checkWallCollision(const glm::vec3& position) const {
    const float ARENA_HALF_WIDTH = 15.0f - WALL_COLLISION_DISTANCE;
    const float ARENA_HALF_LENGTH = 15.0f - WALL_COLLISION_DISTANCE;

    return (position.x + collisionRadius > ARENA_HALF_WIDTH - WALL_THICKNESS ||
        position.x - collisionRadius < -ARENA_HALF_WIDTH + WALL_THICKNESS ||
        position.z + collisionRadius > ARENA_HALF_LENGTH - WALL_THICKNESS ||
        position.z - collisionRadius < -ARENA_HALF_LENGTH + WALL_THICKNESS);
}

/**
 * Checks for collision with any object
 *
 * Combined check for both wall and car collisions.
 *
 * newPosition Position to check
 * return True if collision detected
 */
bool Camera::CheckCollision(const glm::vec3& newPosition) const {
    // Check both wall and car collisions
    return checkWallCollision(newPosition) || checkCarCollision(newPosition);
}

//-----------------------------------------------------------------------------
// Private Methods Implementation
//-----------------------------------------------------------------------------
/**
 * Recalculates camera orientation vectors
 *
 * Updates Front, Right, and Up vectors based on current Yaw and Pitch.
 * This needs to be called whenever the camera's orientation changes.
 */
void Camera::updateCameraVectors() {
    // Calculate new Front vector from Euler angles
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);

    // Recalculate Right and Up vectors
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}

/**
 * Maintains camera at ground level
 *
 * Helper for GROUND mode to keep the camera at eye level.
 */
void Camera::maintainGroundHeight() {
    Position.y = GROUND_CAMERA_HEIGHT;
}

/**
 * Handles collision with environment boundaries
 *
 * Prevents camera from moving outside arena limits or below
 * minimum height.
 *
 * newPosition Position to check and modify if needed
 */
void Camera::handleEnvironmentCollision(glm::vec3& newPosition) {
    // Arena boundaries (should match Arena class dimensions)
    const float ARENA_HALF_WIDTH = 15.0f - WALL_COLLISION_DISTANCE;
    const float ARENA_HALF_LENGTH = 15.0f - WALL_COLLISION_DISTANCE;

    // Check collision with arena walls
    bool collision = false;
    if (newPosition.x > ARENA_HALF_WIDTH) {
        newPosition.x = ARENA_HALF_WIDTH;
        collision = true;
    }
    if (newPosition.x < -ARENA_HALF_WIDTH) {
        newPosition.x = -ARENA_HALF_WIDTH;
        collision = true;
    }
    if (newPosition.z > ARENA_HALF_LENGTH) {
        newPosition.z = ARENA_HALF_LENGTH;
        collision = true;
    }
    if (newPosition.z < -ARENA_HALF_LENGTH) {
        newPosition.z = -ARENA_HALF_LENGTH;
        collision = true;
    }

    // Check floor collision in FREE mode
    if (currentMode == Mode::FREE && newPosition.y < MIN_HEIGHT) {
        newPosition.y = MIN_HEIGHT;
        collision = true;
    }

    isColliding = collision;
}