/**
 * AIController.h
 *
 * This is my AI system for the computer-controlled bumper cars. I wanted
 * the AI cars to feel lively and unpredictable without being too chaotic,
 * so I implemented a state-based behavior system with several different
 * driving modes and smooth transitions between them.
 *
 * The AI has four states:
 * - CRUISING: Normal driving around the arena
 * - TARGETING: Actively pursuing another car (especially the player)
 * - EVADING: Avoiding imminent collisions
 * - RECOVERING: Escaping from being stuck or tipped over
 *
 * Each state has its own behavior logic, and the transitions are handled
 * by the UpdateState method based on various conditions.
 */

#pragma once

#include "BumperCar.h"
#include <glm/glm.hpp>
#include <random>
#include <vector>

 /**
  * Controller for AI-driven bumper cars
  *
  * Manages the behavior and decision-making for computer-controlled cars.
  * Each AI car has its own instance of this controller.
  */
class AIController {
public:
    /**
     * Creates an AI controller for a specific bumper car
     *
     * Initializes with random behavior parameters and sets up the state machine.
     * The controller starts inactive and needs to be explicitly activated.
     *
     * car Pointer to the bumper car to control
     */
    AIController(BumperCar* car);

    /**
     * Main update method for AI decision making and behavior
     *
     * This is the brain of the AI system. It's called each frame to:
     * 1. Update the internal timers and state
     * 2. Check if the car is stuck
     * 3. Run the appropriate behavior for the current state
     * 4. Control the car's movement and steering
     *
     * deltaTime Time since last update in seconds
     * otherCars List of all other cars in the game for targeting/avoidance
     */
    void Update(float deltaTime, const std::vector<BumperCar*>& otherCars);

    /**
     * Activates the AI controller
     *
     * When active, the controller will update and control the car.
     * This is used to enable the AI after the game starts.
     */
    void Activate() { isActive = true; }

    /**
     * Deactivates the AI controller
     *
     * When inactive, the controller won't update or control the car.
     * This is used to disable the AI before the game starts or for debugging.
     */
    void Deactivate() { isActive = false; }

    /**
     * Checks if the controller is currently active
     *
     * return True if active, false otherwise
     */
    bool IsActive() const { return isActive; }

private:
    /**
     * AI behavior states
     *
     * These states determine what behavior the AI car will exhibit.
     * Each state has its own handling function.
     */
    enum class State {
        CRUISING,   // Normal driving around the arena
        TARGETING,  // Actively pursuing another car
        EVADING,    // Avoiding imminent collisions
        RECOVERING  // Escaping from being stuck
    };

    // Constants for AI behavior tuning 
    static constexpr float ARENA_MARGIN = 2.0f;              // How far to stay from arena edges
    static constexpr float SAFE_DISTANCE = 3.0f;             // Minimum distance to maintain from other cars
    static constexpr float STATE_UPDATE_INTERVAL = 0.5f;     // How often to reconsider state
    static constexpr float PATH_UPDATE_INTERVAL = 0.2f;      // How often to update target point
    static constexpr float STUCK_THRESHOLD = 1.5f;           // Time to consider car stuck
    static constexpr float MIN_SPEED = 0.5f;                 // Minimum speed to maintain
    static constexpr float COLLISION_PREDICTION_TIME = 1.0f; // How far ahead to predict collisions

    // Core components
    BumperCar* controlledCar;      // Car being controlled
    bool isActive;                 // Whether the controller is currently active
    State currentState;            // Current behavior state
    float stateTimer;              // Time spent in current state
    bool clockwiseDirection;       // Direction to circulate arena (randomized)
    glm::vec3 targetPoint;         // Current destination point
    glm::vec3 previousPosition;    // Last position (for stuck detection)
    float stuckTimer;              // Time spent not moving
    std::mt19937 rng;              // Random number generator
    float decisionTimer;           // Timer for state decisions
    float pathUpdateTimer;         // Timer for path updates

    /**
     * Updates the AI state based on current conditions
     *
     * This is where I decide which state the AI should be in:
     * - If stuck, it should enter RECOVERING state
     * - If collision is imminent, it should enter EVADING state
     * - If currently cruising and certain conditions are met, it might start TARGETING
     * - If targeting for too long, it should return to CRUISING
     *
     * otherCars List of all other cars for decision making
     */
    void UpdateState(const std::vector<BumperCar*>& otherCars);

    /**
     * Handles CRUISING state behavior
     *
     * In this state, the car drives around the arena in an intentionally predictable
     * pattern, following a path and maintaining a moderate speed.
     *
     * deltaTime Time since last update
     */
    void HandleCruising(float deltaTime);

    /**
     * Handles TARGETING state behavior
     *
     * In this state, the car actively pursues the nearest other car,
     * usually the player. It uses prediction to anticipate where the
     * target will be and steers accordingly.
     *
     * deltaTime Time since last update
     * otherCars List of all other cars for targeting
     */
    void HandleTargeting(float deltaTime, const std::vector<BumperCar*>& otherCars);

    /**
     * Handles EVADING state behavior
     *
     * In this state, the car tries to avoid an imminent collision by
     * steering away from the other car or obstacle.
     *
     * deltaTime Time since last update
     * otherCars List of all other cars for avoidance
     */
    void HandleEvading(float deltaTime, const std::vector<BumperCar*>& otherCars);

    /**
     * Handles RECOVERING state behavior
     *
     * In this state, the car tries to escape from being stuck by
     * reversing, turning, and then driving forward again.
     *
     * deltaTime Time since last update
     */
    void HandleRecovering(float deltaTime);

    /**
     * Calculates the next target point for path following
     *
     * This generates points around the arena in a circular pattern
     * based on the car's clockwise/counterclockwise preference.
     *
     *return Next target position in world space
     */
    glm::vec3 CalculateNextTargetPoint();

    /**
     * Finds the nearest car from the provided list
     *
     * Used to find targets for pursuit or collision avoidance.
     *
     * otherCars List of all cars to check
     * return Pointer to the nearest car, or nullptr if none found
     */
    BumperCar* FindNearestCar(const std::vector<BumperCar*>& otherCars);

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
    std::vector<glm::vec2> GetBoxCorners(const glm::vec3& center, float rotation,
        float halfWidth, float halfLength);

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
    bool HasSeparatingAxis(const std::vector<glm::vec2>& corners1,
        const std::vector<glm::vec2>& corners2);

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
    bool CheckBoxCollision(const glm::vec3& center1, float rot1,
        const glm::vec3& center2, float rot2);

    /**
     * Checks if the car is on a collision course with another car
     *
     * Uses velocity projection to predict future positions and determine
     * if the cars will collide within a certain time window.
     *
     * otherCar Car to check collision with
     * timeToCollision Output parameter to receive time until collision
     * return True if collision is predicted, false otherwise
     */
    bool IsOnCollisionCourse(const BumperCar* otherCar, float& timeToCollision);

    /**
     * Updates the stuck status detection
     *
     * Checks if the car hasn't moved significantly for a while,
     * which indicates it might be stuck against a wall or another car.
     *
     * deltaTime Time since last update
     */
    void UpdateStuckStatus(float deltaTime);

    /**
     * Calculates the steering angle to reach a target
     *
     * Determines what direction and how much the car needs to turn
     * to face the target position.
     *
     * targetPos Position to steer towards
     * return Angle in degrees (positive = left, negative = right)
     */
    float CalculateSteeringAngle(const glm::vec3& targetPos);

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
    bool PredictCollision(const glm::vec3& otherPos, const glm::vec3& otherVel);
};