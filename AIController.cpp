/**
 * AIController.cpp
 *
 * This implements the AI control system for my computer-controlled bumper cars.
 * The core of my design is a state-based behavior system with four states:
 * cruising, targeting, evading, and recovering from collisions.
 *
 * I spent a lot of time tuning the AI to make it fun to play against. It's
 * aggressive enough to be challenging, but not so perfect that it's frustrating
 * to play against. The cars intentionally make "mistakes" and don't always
 * take the optimal path.
 *
 * Collision detection uses the Separating Axis Theorem (SAT) for accurate
 * oriented bounding box (OBB) collisions as described in the collision detection
 * course materials.
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include "AIController.h"
#include "BumperCar.h"
#include <algorithm>
#include <random>
#include <vector>
#include <limits>

 /**
  * Constructor initializes the AI controller for a specific car
  *
  * Sets up initial parameters including randomized behavior preferences.
  * The controller starts inactive and needs to be explicitly activated.
  *
  * car Pointer to the bumper car to control
  */
AIController::AIController(BumperCar* car) :
    controlledCar(car),
    isActive(false),
    currentState(State::CRUISING),
    stateTimer(0.0f),
    clockwiseDirection(rand() % 2 == 0),
    previousPosition(car->GetPosition()),
    stuckTimer(0.0f),
    decisionTimer(0.0f),
    pathUpdateTimer(0.0f)
{
    // Initialize random generator with true randomness
    std::random_device rd;
    rng = std::mt19937(rd());

    // Calculate initial target point
    targetPoint = CalculateNextTargetPoint();
}

/**
 * Main update method for AI decision making and behavior
 *
 * This is the brain of the AI system, called each frame to update
 * internal state and control the car's movement.
 *
 * deltaTime Time since last update in seconds
 * otherCars List of all other cars in the game
 */
void AIController::Update(float deltaTime, const std::vector<BumperCar*>& otherCars) {
    // Skip update if inactive
    if (!isActive) return;

    // Update timers
    stateTimer += deltaTime;
    decisionTimer += deltaTime;
    pathUpdateTimer += deltaTime;

    // Check if car is stuck
    UpdateStuckStatus(deltaTime);

    // Periodically reconsider AI state
    if (decisionTimer >= STATE_UPDATE_INTERVAL) {
        UpdateState(otherCars);
        decisionTimer = 0.0f;
    }

    // Periodically update target point while cruising
    if (pathUpdateTimer >= PATH_UPDATE_INTERVAL && currentState == State::CRUISING) {
        targetPoint = CalculateNextTargetPoint();
        pathUpdateTimer = 0.0f;
    }

    // Run the appropriate behavior for the current state
    switch (currentState) {
    case State::CRUISING: HandleCruising(deltaTime); break;
    case State::TARGETING: HandleTargeting(deltaTime, otherCars); break;
    case State::EVADING: HandleEvading(deltaTime, otherCars); break;
    case State::RECOVERING: HandleRecovering(deltaTime); break;
    }
}

/**
 * Updates the AI state based on current conditions
 *
 * This is where I decide which state the AI should be in.
 * The priority order is:
 * 1. If stuck → RECOVERING
 * 2. If collision imminent → EVADING
 * 3. If conditions are right → switch between CRUISING/TARGETING
 *
 * otherCars List of all other cars for decision making
 */
void AIController::UpdateState(const std::vector<BumperCar*>& otherCars) {
    // Highest priority: If stuck, enter recovery mode
    if (stuckTimer >= STUCK_THRESHOLD) {
        currentState = State::RECOVERING;
        return;
    }

    // Second priority: Check for imminent collisions
    BumperCar* nearestCar = FindNearestCar(otherCars);
    if (nearestCar) {
        float timeToCollision;
        if (IsOnCollisionCourse(nearestCar, timeToCollision)) {
            currentState = State::EVADING;
            return;
        }
    }

    // Lowest priority: Switch between cruising and targeting
    // Occasionally switch from cruising to targeting if we've been cruising for a while
    if (currentState == State::CRUISING && stateTimer > 5.0f) {
        // 30% chance to start targeting if there's a valid target
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        if (dist(rng) < 0.3f && nearestCar) {
            currentState = State::TARGETING;
            stateTimer = 0.0f;
        }
    }
    // Switch back to cruising after targeting for a while
    else if (currentState == State::TARGETING && stateTimer > 3.0f) {
        currentState = State::CRUISING;
        stateTimer = 0.0f;
    }
}

/**
 * Handles CRUISING state behavior
 *
 * In this state, the car drives around the arena in a circuit,
 * adjusting speed based on turning requirements.
 *
 * deltaTime Time since last update
 */
void AIController::HandleCruising(float deltaTime) {
    // Get current speed
    float speed = glm::length(controlledCar->GetVelocity());

    // Calculate angle to target
    float angleToTarget = CalculateSteeringAngle(targetPoint);

    // Adjust desired speed based on turn angle (slower in sharp turns)
    float desiredSpeed = BumperCar::MAX_SPEED * (1.0f - std::abs(angleToTarget) / 180.0f);
    desiredSpeed = std::max(desiredSpeed, MIN_SPEED);

    // Turn towards target if needed
    if (angleToTarget > 5.0f) {
        controlledCar->TurnLeft(deltaTime);
    }
    else if (angleToTarget < -5.0f) {
        controlledCar->TurnRight(deltaTime);
    }

    // Adjust speed to match desired speed
    if (speed < desiredSpeed) {
        controlledCar->Accelerate(deltaTime);
    }
    else {
        controlledCar->Brake(deltaTime * 0.5f);
    }
}

/**
 * Handles TARGETING state behavior
 *
 * In this state, the car actively pursues another car (usually the player),
 * predicting its future position for more accurate targeting.
 *
 * deltaTime Time since last update
 * otherCars List of all other cars for targeting
 */
void AIController::HandleTargeting(float deltaTime, const std::vector<BumperCar*>& otherCars) {
    // Find the nearest car to target
    BumperCar* target = FindNearestCar(otherCars);
    if (!target) {
        // No valid target, fall back to cruising
        currentState = State::CRUISING;
        return;
    }

    // Predict target's future position based on its velocity
    glm::vec3 predictedPos = target->GetPosition() + target->GetVelocity() * COLLISION_PREDICTION_TIME;

    // Calculate steering angle to predicted position
    float angleToTarget = CalculateSteeringAngle(predictedPos);

    // Turn towards predicted position with more aggressive turning
    if (angleToTarget > 3.0f) {
        controlledCar->TurnLeft(deltaTime * 1.2f);
    }
    else if (angleToTarget < -3.0f) {
        controlledCar->TurnRight(deltaTime * 1.2f);
    }

    // Accelerate faster than normal when targeting
    controlledCar->Accelerate(deltaTime * 1.5f);
}

/**
 * Handles EVADING state behavior
 *
 * In this state, the car tries to avoid an imminent collision by
 * steering away from the other car or obstacle.
 *
 * deltaTime Time since last update
 * otherCars List of all other cars for avoidance
 */
void AIController::HandleEvading(float deltaTime, const std::vector<BumperCar*>& otherCars) {
    BumperCar* nearestCar = FindNearestCar(otherCars);
    if (!nearestCar) {
        // No car to evade, return to cruising
        currentState = State::CRUISING;
        return;
    }

    // Calculate direction away from the nearest car
    glm::vec3 evadeDirection = controlledCar->GetPosition() - nearestCar->GetPosition();

    // Determine the angle to steer to avoid collision
    float angleToSafe = CalculateSteeringAngle(controlledCar->GetPosition() + evadeDirection);

    // Turn sharply away from the threat
    if (angleToSafe > 0.0f) {
        controlledCar->TurnLeft(deltaTime * 2.0f);
    }
    else {
        controlledCar->TurnRight(deltaTime * 2.0f);
    }

    // Accelerate to get away quickly
    controlledCar->Accelerate(deltaTime);

    // Return to cruising once we've reached a safe distance
    if (glm::length(evadeDirection) > SAFE_DISTANCE * 2.0f) {
        currentState = State::CRUISING;
    }
}

/**
 * Handles RECOVERING state behavior
 *
 * In this state, the car tries to escape from being stuck by
 * braking, changing direction, and then accelerating.
 *
 * deltaTime Time since last update
 */
void AIController::HandleRecovering(float deltaTime) {
    // First brake to clear any existing momentum
    controlledCar->Brake(deltaTime);

    // Once nearly stopped, change direction and accelerate
    if (glm::length(controlledCar->GetVelocity()) < 0.1f) {
        // Change direction preference
        clockwiseDirection = !clockwiseDirection;

        // Accelerate in the new direction
        controlledCar->Accelerate(deltaTime);

        // If no longer stuck, return to cruising
        if (stuckTimer < STUCK_THRESHOLD) {
            currentState = State::CRUISING;
        }
    }
}

/**
 * Calculates the next target point for path following
 *
 * This generates points around the arena in a circular pattern
 * based on the car's clockwise/counterclockwise preference.
 *
 * Next target position in world space
 */
glm::vec3 AIController::CalculateNextTargetPoint() {
    const float ARENA_SIZE = 30.0f - ARENA_MARGIN * 2.0f;
    const float HALF_SIZE = ARENA_SIZE * 0.5f;

    // Different angle depending on direction preference
    float angle = clockwiseDirection ? -glm::pi<float>() * 0.5f : glm::pi<float>() * 0.5f;

    // Keep radius slightly smaller than arena size
    float radius = HALF_SIZE * 0.8f;

    // Calculate point on the circle
    return glm::vec3(
        radius * cos(angle),
        0.0f,
        radius * sin(angle)
    );
}

/**
 * Finds the nearest car from the provided list
 *
 * Used to find targets for pursuit or collision avoidance.
 *
 * otherCars List of all cars to check
 * return Pointer to the nearest car, or nullptr if none found
 */
BumperCar* AIController::FindNearestCar(const std::vector<BumperCar*>& otherCars) {
    BumperCar* nearest = nullptr;
    float minDistance = std::numeric_limits<float>::max();

    for (auto* car : otherCars) {
        // Skip self
        if (car == controlledCar) continue;

        // Calculate distance to this car
        float distance = glm::length(car->GetPosition() - controlledCar->GetPosition());

        // Update nearest if this one is closer
        if (distance < minDistance) {
            minDistance = distance;
            nearest = car;
        }
    }

    return nearest;
}

/**
 * Gets the corners of a rotated bounding box
 *
 * Calculates the 2D corners of a car's collision box in the XZ plane.
 * Implementation inspired by lab sheet 7's collision detection examples.
 *
 * center Box center position
 * rotation Box rotation in radians
 * halfWidth Half width of the box
 * halfLength Half length of the box
 * return Vector of 2D corner points in the XZ plane
 */
std::vector<glm::vec2> AIController::GetBoxCorners(const glm::vec3& center, float rotation,
    float halfWidth, float halfLength) {
    std::vector<glm::vec2> corners = {
        glm::vec2(-halfWidth, -halfLength),  // Back left
        glm::vec2(halfWidth, -halfLength),   // Back right
        glm::vec2(halfWidth, halfLength),    // Front right
        glm::vec2(-halfWidth, halfLength)    // Front left
    };

    // Rotate and translate corners
    float cosRot = cos(rotation);
    float sinRot = sin(rotation);

    for (auto& corner : corners) {
        float x = corner.x;
        float z = corner.y;
        corner.x = x * cosRot - z * sinRot + center.x;
        corner.y = x * sinRot + z * cosRot + center.z;
    }

    return corners;
}

/**
 * Checks for a separating axis between two boxes
 *
 * Implementation of the Separating Axis Theorem (SAT) as described in the
 * collision detection lab sheet. If any separating axis exists, the boxes don't collide.
 *
 * corners1 Corners of first box
 * corners2 Corners of second box
 * return True if a separating axis exists (no collision)
 */
bool AIController::HasSeparatingAxis(const std::vector<glm::vec2>& corners1,
    const std::vector<glm::vec2>& corners2) {
    // Get axes to test (perpendicular to each box's edges)
    std::vector<glm::vec2> axes;

    // Generate axes for both boxes (4 in total for 2D boxes)
    for (size_t i = 0; i < corners1.size(); i++) {
        glm::vec2 edge = corners1[(i + 1) % corners1.size()] - corners1[i];
        axes.push_back(glm::vec2(-edge.y, edge.x)); // Perpendicular to edge
    }

    for (size_t i = 0; i < corners2.size(); i++) {
        glm::vec2 edge = corners2[(i + 1) % corners2.size()] - corners2[i];
        axes.push_back(glm::vec2(-edge.y, edge.x)); // Perpendicular to edge
    }

    // Test projection onto each axis
    for (const auto& axis : axes) {
        float min1 = std::numeric_limits<float>::max();
        float max1 = std::numeric_limits<float>::lowest();
        float min2 = std::numeric_limits<float>::max();
        float max2 = std::numeric_limits<float>::lowest();

        // Normalize the axis
        glm::vec2 normalizedAxis = glm::normalize(axis);

        // Project corners onto axis
        for (const auto& corner : corners1) {
            float proj = glm::dot(corner, normalizedAxis);
            min1 = std::min(min1, proj);
            max1 = std::max(max1, proj);
        }

        for (const auto& corner : corners2) {
            float proj = glm::dot(corner, normalizedAxis);
            min2 = std::min(min2, proj);
            max2 = std::max(max2, proj);
        }

        // Check for gap
        if (max1 < min2 || max2 < min1) {
            return true;  // Found a separating axis
        }
    }

    return false;  // No separating axis found, boxes overlap
}

/**
 * Checks for collision between two oriented bounding boxes
 *
 * Implementation of OBB collision detection using the Separating Axis Theorem (SAT),
 * as described in the collision detection course materials.
 *
 * center1 Center of first box
 * rot1 Rotation of first box in radians
 * center2 Center of second box
 * rot2 Rotation of second box in radians
 * return True if boxes are colliding
 */
bool AIController::CheckBoxCollision(const glm::vec3& center1, float rot1,
    const glm::vec3& center2, float rot2) {
    // Define box half-dimensions
    float halfLength = BumperCar::COLLISION_BOX_LENGTH * 0.5f;
    float halfWidth = BumperCar::COLLISION_BOX_WIDTH * 0.5f;

    // Get corners of both boxes in world space
    std::vector<glm::vec2> corners1 = GetBoxCorners(center1, rot1, halfWidth, halfLength);
    std::vector<glm::vec2> corners2 = GetBoxCorners(center2, rot2, halfWidth, halfLength);

    // Use Separating Axis Theorem to check for collision
    return !HasSeparatingAxis(corners1, corners2);
}

/**
 * Checks if the car is on a collision course with another car
 *
 * Uses the Separating Axis Theorem for accurate oriented bounding box (OBB) collision detection.
 * This improves upon the simple distance-based approach by considering the actual shapes and
 * orientations of the cars.
 *
 * otherCar Car to check collision with
 * timeToCollision Output parameter to receive time until collision
 * True if collision is predicted, false otherwise
 */
bool AIController::IsOnCollisionCourse(const BumperCar* otherCar, float& timeToCollision) {
    // Default to a reasonable value in case we detect a collision but can't compute exact time
    timeToCollision = COLLISION_PREDICTION_TIME * 0.5f;

    // Current positions and rotations
    glm::vec3 myPos = controlledCar->GetPosition();
    glm::vec3 otherPos = otherCar->GetPosition();
    float myRot = glm::radians(controlledCar->GetRotation());
    float otherRot = glm::radians(otherCar->GetRotation());

    // Current velocities
    glm::vec3 myVel = controlledCar->GetVelocity();
    glm::vec3 otherVel = otherCar->GetVelocity();

    // If both cars are essentially stationary, no collision is predicted
    if (glm::length(myVel) < 0.1f && glm::length(otherVel) < 0.1f) {
        return false;
    }

    // Test multiple future positions to find approximate collision time
    const int numSteps = 10;
    float stepSize = COLLISION_PREDICTION_TIME / numSteps;

    for (int i = 1; i <= numSteps; i++) {
        float t = stepSize * i;

        // Predict future positions
        glm::vec3 myFuturePos = myPos + myVel * t;
        glm::vec3 otherFuturePos = otherPos + otherVel * t;

        // Assume rotation doesn't change significantly in short prediction time
        if (CheckBoxCollision(myFuturePos, myRot, otherFuturePos, otherRot)) {
            timeToCollision = t;
            return true;
        }
    }

    return false;
}

/**
 * Updates the stuck status detection
 *
 * Checks if the car hasn't moved significantly for a while,
 * which indicates it might be stuck against a wall or another car.
 *
 * deltaTime Time since last update
 */
void AIController::UpdateStuckStatus(float deltaTime) {
    glm::vec3 currentPos = controlledCar->GetPosition();

    // Measure how much the car has moved since last frame
    float movement = glm::length(currentPos - previousPosition);

    // If barely moving and not accelerating, increment stuck timer
    if (movement < 0.01f && glm::length(controlledCar->GetVelocity()) < 0.1f) {
        stuckTimer += deltaTime;
    }
    else {
        // Reset timer if moving normally
        stuckTimer = 0.0f;
    }

    // Update position for next frame's comparison
    previousPosition = currentPos;
}

/**
 * Calculates the steering angle to reach a target
 *
 * Determines what direction and how much the car needs to turn
 * to face the target position.
 *
 * targetPos Position to steer towards
 * return Angle in degrees (positive = left, negative = right)
 */
float AIController::CalculateSteeringAngle(const glm::vec3& targetPos) {
    // Get direction to target
    glm::vec3 toTarget = targetPos - controlledCar->GetPosition();

    // Get car's forward direction
    glm::vec3 forward = controlledCar->GetForwardVector();

    // Flatten vectors to XZ plane for 2D steering
    glm::vec3 flatForward = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
    glm::vec3 flatToTarget = glm::normalize(glm::vec3(toTarget.x, 0.0f, toTarget.z));

    // Calculate angle between vectors
    float dot = glm::dot(flatForward, flatToTarget);
    dot = glm::clamp(dot, -1.0f, 1.0f);  // Avoid precision errors
    float angle = glm::degrees(glm::acos(dot));

    // Determine if the target is to the left or right
    glm::vec3 cross = glm::cross(flatForward, flatToTarget);

    // Return positive angle for left turns, negative for right turns
    return (cross.y < 0.0f) ? -angle : angle;
}

/**
 * Predicts if a collision will occur with the given object
 *
 * Uses the Separating Axis Theorem for accurate collision prediction
 * between oriented bounding boxes.
 *
 * otherPos Other object position
 * otherVel Other object velocity
 * return True if collision is predicted, false otherwise
 */
bool AIController::PredictCollision(const glm::vec3& otherPos, const glm::vec3& otherVel) {
    // Simple prediction by projecting current velocities forward
    glm::vec3 futurePos = controlledCar->GetPosition() + controlledCar->GetVelocity() * COLLISION_PREDICTION_TIME;
    glm::vec3 otherFuturePos = otherPos + otherVel * COLLISION_PREDICTION_TIME;

    // Use the SAT-based box collision detection
    float myRot = glm::radians(controlledCar->GetRotation());
    // Assuming the other object has zero rotation for simplicity
    return CheckBoxCollision(futurePos, myRot, otherFuturePos, 0.0f);
}