/**
 * GameManager.cpp
 *
 * This is the heart of my bumper car game! It manages all the major systems - physics,
 * rendering, AI, input handling, you name it. I designed it as the central "brain"
 * that coordinates everything else in the game.
 *
 * Creating this class was probably the biggest challenge of the project, since it needs
 * to talk to all the other systems. I spent a lot of time making sure everything worked
 * together smoothly - especially the collision detection, which was tricky to get right.
 *
 * Key things I've implemented here:
 * - The main game loop that updates all game elements
 * - Physics simulation with elastic collisions for that fun bumper car feel
 * - Multiple camera modes that follow the action in different ways
 * - Lighting system with realistic shadows
 * - Particle effects for visual feedback
 * - AI car behavior that makes them challenging to play against
 * - Debug visualization tools that helped me fix lots of physics issues
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/string_cast.hpp>
#include "GameManager.h"
#include "AIController.h"
#include "BumperCar.h"
#include <iostream>

 //-----------------------------------------------------------------------------
 // Constructor & Destructor Implementation
 //-----------------------------------------------------------------------------
 /**
  * Sets up the initial state of the game manager with default values.
  * I initialize all the shaders, particles, and game state variables here.
  */
GameManager::GameManager() :
    globalLightingEnabled(true),
    arenaShader("Shaders/arena.vert", "Shaders/arena.frag"),
    bumperCarShader("Shaders/model_loading.vert", "Shaders/model_loading.frag"),
    shadowShader("Shaders/shadow_mapping.vert", "Shaders/shadow_mapping.frag"),
    staticModelShader("Shaders/static_model.vert", "Shaders/static_model.frag"),
    bumperCar("Resources/models/bumper_car.obj"),
    lastX(1920.0f / 2.0f),
    lastY(1080.0f / 2.0f),
    firstMouse(true),
    window(nullptr),
    deltaTime(0.016f),
    depthMap(0),
    depthMapFBO(0),
    lightSpaceMatrix(glm::mat4(1.0f)),
    projection(glm::mat4(1.0f)),
    view(glm::mat4(1.0f)),
    showCollisions(false),
    debugVAO(0),
    debugVBO(0),
    totalTime(0.0f),
    particleEffectsEnabled(true),
    showKeyBindingsMenu(false)
{
    // Initialize key states
    for (int i = 0; i < 348; i++) {
        keysProcessed[i] = false;
    }

    // Create shaders as unique pointers for better memory management
    headlightSourceShader = std::make_unique<Shader>(
        "Shaders/headlight_source.vert",
        "Shaders/headlight_source.frag"
    );
    debugShader = std::make_unique<Shader>(
        "Shaders/debug.vert",
        "Shaders/debug.frag"
    );
    antennaParticles = std::make_unique<ParticleSystem>(100);
    particleShader = std::make_unique<Shader>(
        "Shaders/particle.vert",
        "Shaders/particle.frag"
    );
    textShader = std::make_unique<Shader>(
        "Shaders/text.vert",
        "Shaders/text.frag"
    );
}

/**
 * Cleans up all the OpenGL resources I've allocated.
 * This was important to get right to avoid memory leaks.
 */
GameManager::~GameManager() {
    // Cleanup OpenGL resources
    for (auto& source : headlightSources) {
        glDeleteVertexArrays(1, &source.VAO);
        glDeleteBuffers(1, &source.VBO);
        glDeleteBuffers(1, &source.EBO);
    }

    glDeleteVertexArrays(1, &debugVAO);
    glDeleteBuffers(1, &debugVBO);
    glDeleteFramebuffers(1, &depthMapFBO);
    glDeleteTextures(1, &depthMap);
}

//-----------------------------------------------------------------------------
// Initialization Functions
//-----------------------------------------------------------------------------

/**
 * This is where I set up all the game systems. It's like the "opening ceremony"
 * of the game - creating the scene, positioning everything, and getting ready
 * for the first frame.
 *
 * The order of initialization is important - some systems depend on others
 * being ready first. For example, I initialize OpenGL resources before loading
 * models that need those resources.
 */
void GameManager::Initialize(GLFWwindow* windowHandle) {
    window = windowHandle;
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Initialize core systems
    InitializeShaders();
    InitializeLighting();
    InitializeShadowMapping();
    InitializeDebugVisualization();
    InitializeHeadlightSources();
    InitializeSkybox();
    InitializeStaticModels();

    // Initialize game objects
    arena.Initialize();
    bumperCar.Initialize();

    // Set initial player car position and orientation
    bumperCar.SetPosition(glm::vec3(0.0f, -0.3f, 0.0f));
    bumperCar.SetRotation(0.0f);

    // Initialize matrices
    UpdateProjectionMatrix();
    UpdateViewMatrix();

    // Initialize particle system
    antennaParticles->SetActive(true);

    // Initialize text rendering
    InitializeFreetype();
    SetupTextRendering();

    // Create and initialize AI cars
    try {
        // Define different model paths for AI cars
        std::vector<std::string> aiCarModelPaths = {
            "Resources/models/bumper_car_AI1.obj",
            "Resources/models/bumper_car_AI2.obj",
            "Resources/models/bumper_car_AI3.obj",
            "Resources/models/bumper_car_AI4.obj"
        };
        for (int i = 0; i < NUM_AI_CARS; i++) {
            // Create unique car with specific model
            auto newCar = std::make_unique<BumperCar>(aiCarModelPaths[i]);
            // Validate car creation
            if (!newCar) {
                std::cerr << "Failed to create AI car " << i << std::endl;
                continue;
            }

            // Position cars staggered to prevent overlap
            float angle = (2 * glm::pi<float>() * i) / NUM_AI_CARS;
            float radius = 8.0f + (i % 2) * 3.0f;
            float xOffset = radius * cos(angle);
            float zOffset = radius * sin(angle);

            // Set position and orientation
            newCar->SetPosition(glm::vec3(xOffset, -0.3f, zOffset));
            newCar->SetRotation(glm::degrees(angle + glm::pi<float>())); // Face center

            // Give cars initial velocity in tangential direction to start moving
            float initialSpeed = 2.0f + (rand() % 20) / 10.0f; // Speed between 2.0 and 4.0
            glm::vec3 tangent(-sin(angle + glm::pi<float>() / 2), 0.0f, -cos(angle + glm::pi<float>() / 2));
            newCar->SetVelocity(tangent * initialSpeed);

            // Initialize car
            newCar->Initialize();

            // Create AI controller
            auto controller = std::make_unique<AIController>(newCar.get());

            // Validate controller creation
            if (!controller) {
                std::cerr << "Failed to create AI controller for car " << i << std::endl;
                continue;
            }

            // Store car and controller
            aiCars.push_back(std::move(newCar));
            aiControllers.push_back(std::move(controller));

            // Debug output
            std::cout << "AI Car " << i << " created at "
                << xOffset << ", " << zOffset
                << " facing " << glm::degrees(angle + glm::pi<float>())
                << " with initial speed " << initialSpeed << std::endl;
        }

        // Initial deactivation of AI controllers, but cars will already be moving
        gameStarted = false;
        for (auto& controller : aiControllers) {
            controller->Deactivate();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error initializing AI cars: " << e.what() << std::endl;
    }

    // Enable OpenGL states
    glEnable(GL_DEPTH_TEST);

    // Final initialization debug
    std::cout << "Game Initialization Complete" << std::endl;
    std::cout << "Total AI Cars: " << aiCars.size() << std::endl;
}

/**
 * Sets up all the shader programs I use throughout the game.
 * I've created different shaders for different purposes - arena, cars,
 * shadows, etc. Each has specific features tuned for its purpose.
 */
void GameManager::InitializeShaders() {
    arenaShader.use();
    arenaShader.setInt("texture_diffuse1", 0);
    arenaShader.setInt("texture_normal1", 1);

    bumperCarShader.use();
    bumperCarShader.setInt("texture_diffuse1", 0);
    bumperCarShader.setInt("texture_normal1", 1);
}

/**
 * Sets up shadow mapping for the main light source.
 * Getting shadows right was one of the more challenging visual effects.
 * I spent a lot of time tweaking the resolution, filtering, and other
 * parameters to get shadows that look good without killing performance.
 */
void GameManager::InitializeShadowMapping() {
    // Increase shadow map resolution for better quality
    const unsigned int SHADOW_WIDTH = 2048;
    const unsigned int SHADOW_HEIGHT = 2048;

    glGenFramebuffers(1, &depthMapFBO);

    // Create depth texture
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
        SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    // Set texture parameters for better filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Use CLAMP_TO_BORDER with white border color to avoid dark edges
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    // Use depth comparison mode for better shadow quality
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

    // Attach depth texture to framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);

    // Tell OpenGL not to render color data
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    // Verify framebuffer is complete
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "ERROR: Framebuffer not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
//-----------------------------------------------------------------------------
// Update & Render Implementation
//-----------------------------------------------------------------------------

/**
 * This is the main update loop that runs once per frame. It's the heartbeat
 * of the game - updating physics, AI, particles, and everything else that changes
 * over time.
 *
 * I spent a lot of time optimizing this function since it needs to run at least
 * 60 times per second for smooth gameplay. The trickiest part was balancing
 * realistic physics with performance.
 */
void GameManager::Update(float dt) {
    deltaTime = dt;

    // Game start logic with timer fallback
    static float startupTimer = 0.0f;
    if (!gameStarted) {
        startupTimer += dt;
        float velocity = glm::length(bumperCar.GetVelocity());

        // Start game after player moves OR after 3 seconds
        if (velocity > 0.1f || startupTimer > 3.0f) {
            std::cout << "Game started! " << (velocity > 0.1f ? "Player moved" : "Auto-start after timeout") << std::endl;
            gameStarted = true;
            // Activate AI controllers with debug output
            for (size_t i = 0; i < aiControllers.size(); ++i) {
                aiControllers[i]->Activate();
                std::cout << "AI Controller " << i << " activated" << std::endl;
            }
        }
    }

    // Always update player car movement
    bumperCar.UpdateMovement(deltaTime);

    // Create list of all cars (needed for collision detection)
    std::vector<BumperCar*> allCars = { &bumperCar };
    for (auto& car : aiCars) {
        allCars.push_back(car.get());
    }

    // Safety checks for all cars (prevent disappearing)
    for (auto* car : allCars) {
        // Force car to stay at correct Y position
        glm::vec3 pos = car->GetPosition();
        if (pos.y != -0.3f) {
            pos.y = -0.3f;
            car->SetPosition(pos);
        }

        // Prevent cars from escaping arena (teleporting out)
        if (std::abs(pos.x) > 25.0f || std::abs(pos.z) > 25.0f) {
            // Car has escaped - reset to safe position
            float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * 3.14159f;
            float radius = 8.0f + static_cast<float>(rand() % 4);
            pos.x = cos(angle) * radius;
            pos.z = sin(angle) * radius;
            pos.y = -0.3f;
            car->SetPosition(pos);

            // Reset rotation and velocities
            car->SetRotation(glm::degrees(angle + glm::pi<float>()));
            car->SetVelocity(glm::vec3(0.0f));
            car->SetAngularVelocity(0.0f);
            std::cout << "Car reset after escape - new position: " << pos.x << ", " << pos.z << std::endl;
        }

        // Cap car speeds to prevent extreme movements
        glm::vec3 vel = car->GetVelocity();
        float speed = glm::length(vel);
        if (speed > BumperCar::MAX_SPEED * 1.5f) {
            car->SetVelocity(glm::normalize(vel) * BumperCar::MAX_SPEED * 1.5f);
        }

        // Cap angular velocity
        float angVel = car->GetAngularVelocity();
        if (std::abs(angVel) > BumperCar::MAX_ANGULAR_SPEED) {
            car->SetAngularVelocity(angVel > 0 ?
                BumperCar::MAX_ANGULAR_SPEED : -BumperCar::MAX_ANGULAR_SPEED);
        }
    }

    if (gameStarted) {
        // Update AI cars and their controllers
        for (size_t i = 0; i < aiCars.size(); i++) {
            aiCars[i]->UpdateMovement(deltaTime);
            aiControllers[i]->Update(deltaTime, allCars);

            // Anti-stall check: if car isn't moving, give it a push
            float carSpeed = glm::length(aiCars[i]->GetVelocity());
            static std::vector<float> carStallTimers(aiCars.size(), 0.0f);

            if (carSpeed < 0.5f) {
                carStallTimers[i] += deltaTime;
                if (carStallTimers[i] > 2.0f) {
                    // Car stalled - give it a push in forward direction
                    glm::vec3 forward = aiCars[i]->GetForwardVector();
                    aiCars[i]->SetVelocity(forward * 3.0f);
                    carStallTimers[i] = 0.0f;
                    std::cout << "Unstalled car " << i << std::endl;
                }
            }
            else {
                carStallTimers[i] = 0.0f;
            }
        }

        // Check collisions between all cars with improved handling
        for (size_t i = 0; i < allCars.size(); i++) {
            for (size_t j = i + 1; j < allCars.size(); j++) {
                BumperCar* car1 = allCars[i];
                BumperCar* car2 = allCars[j];

                // Get collision boxes for both cars
                glm::vec3 pos1 = car1->GetPosition();
                glm::vec3 pos2 = car2->GetPosition();
                float rot1 = glm::radians(car1->GetRotation());
                float rot2 = glm::radians(car2->GetRotation());

                // Calculate collision box centers with offset
                glm::mat4 rot1Matrix = glm::rotate(glm::mat4(1.0f), rot1, glm::vec3(0.0f, 1.0f, 0.0f));
                glm::mat4 rot2Matrix = glm::rotate(glm::mat4(1.0f), rot2, glm::vec3(0.0f, 1.0f, 0.0f));

                glm::vec3 offset(BumperCar::COLLISION_OFFSET_X,
                    BumperCar::COLLISION_OFFSET_Y,
                    BumperCar::COLLISION_OFFSET_Z);

                glm::vec3 box1Center = pos1 + glm::vec3(rot1Matrix * glm::vec4(offset, 1.0f));
                glm::vec3 box2Center = pos2 + glm::vec3(rot2Matrix * glm::vec4(offset, 1.0f));

                // Calculate distance between cars
                glm::vec3 separationVector = box2Center - box1Center;
                float distance = glm::length(separationVector);
                float minDistance = (BumperCar::COLLISION_BOX_LENGTH + BumperCar::COLLISION_BOX_WIDTH) * 0.5f;

                // If cars are close enough, check for collision
                if (distance < minDistance) {
                    // Ensure we have a valid separation direction
                    if (distance < 0.001f) {
                        // If cars are directly on top of each other, create a random separation vector
                        float randomAngle = static_cast<float>(rand()) / RAND_MAX * 2.0f * 3.14159f;
                        separationVector = glm::vec3(cos(randomAngle), 0.0f, sin(randomAngle));
                        distance = 0.001f; // Avoid division by zero
                    }

                    // Calculate collision normal
                    glm::vec3 collisionNormal = separationVector / distance;

                    // Calculate penetration depth
                    float penetrationDepth = minDistance - distance;

                    // Immediately separate the cars based on their velocities
                    // Faster moving car should be pushed back more
                    float totalSpeed = glm::length(car1->GetVelocity()) + glm::length(car2->GetVelocity());
                    float separationRatio;

                    if (totalSpeed < 0.1f) {
                        // If both cars are almost stationary, separate them equally
                        separationRatio = 0.5f;
                    }
                    else {
                        // Distribute separation based on speed ratio, with limits
                        float car1Speed = glm::length(car1->GetVelocity());
                        // Manual clamp since std::clamp requires C++17
                        separationRatio = std::min(std::max(car1Speed / totalSpeed, 0.3f), 0.7f);
                    }

                    // Apply separation with a small buffer to prevent immediate re-collision
                    const float SEPARATION_BUFFER = 1.05f;
                    glm::vec3 car1Displacement = -collisionNormal * (penetrationDepth * separationRatio * SEPARATION_BUFFER);
                    glm::vec3 car2Displacement = collisionNormal * (penetrationDepth * (1.0f - separationRatio) * SEPARATION_BUFFER);

                    // Update positions
                    car1->SetPosition(car1->GetPosition() + car1Displacement);
                    car2->SetPosition(car2->GetPosition() + car2Displacement);

                    // Calculate relative velocity along collision normal
                    glm::vec3 relativeVelocity = car2->GetVelocity() - car1->GetVelocity();
                    float relativeNormalVelocity = glm::dot(relativeVelocity, collisionNormal);

                    // Only process collision if cars are moving toward each other
                    if (relativeNormalVelocity < 0) {
                        // Apply collision response to both cars
                        car1->HandleCollisionWith(-collisionNormal);
                        car2->HandleCollisionWith(collisionNormal);
                    }
                }
            }
        }
    }

    // Update game systems
    UpdateLighting();
    bumperCar.ApplyGravity(deltaTime);
    HandleInput(window);
    UpdateCameraPosition();
    UpdateProjectionMatrix();
    UpdateViewMatrix();

    // Update particle effects
    if (particleEffectsEnabled) {
        // Calculate antenna position in world space
        glm::vec3 carPos = bumperCar.GetPosition();
        float carRotation = glm::radians(bumperCar.GetRotation());
        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::vec3 localAntennaOffset(
            BumperCar::ANTENNA_OFFSET_X,
            BumperCar::ANTENNA_OFFSET_Y,
            BumperCar::ANTENNA_OFFSET_Z
        );
        glm::vec4 antennaWorldOffset = rotationMatrix * glm::vec4(localAntennaOffset, 1.0f);
        glm::vec3 antennaPosition = carPos + glm::vec3(antennaWorldOffset);

        // Update particle system
        antennaParticles->Update(dt, antennaPosition);

        // Only emit particles when accelerating
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            static float emitTimer = 0.0f;
            emitTimer += dt;
            if (emitTimer >= 0.03f) {
                antennaParticles->Emit(antennaPosition, 6);
                emitTimer = 0.0f;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------
//  Collision update logic
//------------------------------------------------------------------------------------------------- 

/**
 * This is my collision detection system for car-to-car collisions.
 * Getting this right was one of the most challenging parts of the physics system.
 *
 * I use oriented bounding boxes (OBBs) to represent the car collision volumes,
 * then check for overlaps between them using the Separating Axis Theorem.
 * It's more complex than simple sphere collision, but I needed the extra accuracy
 * for the bumper car shapes.
 */
void GameManager::CheckCarCollisions() {
    // Create a list of all cars for checking collisions
    std::vector<BumperCar*> allCars;
    allCars.push_back(&bumperCar);  // Add player car
    for (auto& car : aiCars) {
        allCars.push_back(car.get());
    }

    // Check collisions between all pairs of cars
    for (size_t i = 0; i < allCars.size(); i++) {
        for (size_t j = i + 1; j < allCars.size(); j++) {
            BumperCar* car1 = allCars[i];
            BumperCar* car2 = allCars[j];

            // Get collision boxes for both cars
            glm::vec3 pos1 = car1->GetPosition();
            glm::vec3 pos2 = car2->GetPosition();
            float rot1 = glm::radians(car1->GetRotation());
            float rot2 = glm::radians(car2->GetRotation());

            // Create rotation matrices
            glm::mat4 rot1Matrix = glm::rotate(glm::mat4(1.0f), rot1, glm::vec3(0.0f, 1.0f, 0.0f));
            glm::mat4 rot2Matrix = glm::rotate(glm::mat4(1.0f), rot2, glm::vec3(0.0f, 1.0f, 0.0f));

            // Calculate collision box centers with offset
            glm::vec3 offset(BumperCar::COLLISION_OFFSET_X,
                BumperCar::COLLISION_OFFSET_Y,
                BumperCar::COLLISION_OFFSET_Z);

            glm::vec3 box1Center = pos1 + glm::vec3(rot1Matrix * glm::vec4(offset, 1.0f));
            glm::vec3 box2Center = pos2 + glm::vec3(rot2Matrix * glm::vec4(offset, 1.0f));

            // Calculate collision
            if (CheckBoxCollision(box1Center, rot1, box2Center, rot2)) {
                // Calculate collision normal
                glm::vec3 collisionNormal = glm::normalize(box2Center - box1Center);

                // Calculate relative velocity
                glm::vec3 relativeVel = car2->GetVelocity() - car1->GetVelocity();

                // Only process collision if cars are moving towards each other
                if (glm::dot(relativeVel, collisionNormal) < 0) {
                    // Apply collision response to both cars
                    car1->HandleCollisionWith(-collisionNormal);
                    car2->HandleCollisionWith(collisionNormal);
                }
            }
        }
    }
}

/**
 * Tests if two oriented bounding boxes are colliding.
 * This uses some vector math to check if the boxes overlap in 3D space.
 *
 * I actually had to re-implement this a few times to get it working properly.
 * Box-box collision is tricky because the boxes can rotate in any direction.
 */
bool GameManager::CheckBoxCollision(const glm::vec3& center1, float rot1,
    const glm::vec3& center2, float rot2) {
    // Define box half-dimensions
    float halfLength = BumperCar::COLLISION_BOX_LENGTH * 0.5f;
    float halfWidth = BumperCar::COLLISION_BOX_WIDTH * 0.5f;

    // Get corners of both boxes in world space
    std::vector<glm::vec2> corners1 = GetBoxCorners(center1, rot1, halfWidth, halfLength);
    std::vector<glm::vec2> corners2 = GetBoxCorners(center2, rot2, halfWidth, halfLength);

    // Use Separating Axis Theorem (SAT) to check for collision
    return !HasSeparatingAxis(corners1, corners2);
}

/**
 * Calculates the corner points of a rotated box in 2D space.
 * I'm only computing the 2D corners (ignoring height) since our bumper cars
 * can't really stack on top of each other anyway. This simplification makes
 * the collision detection faster.
 */
std::vector<glm::vec2> GameManager::GetBoxCorners(const glm::vec3& center, float rotation,
    float halfWidth, float halfLength) {
    std::vector<glm::vec2> corners = {
        glm::vec2(-halfWidth, -halfLength),
        glm::vec2(halfWidth, -halfLength),
        glm::vec2(halfWidth, halfLength),
        glm::vec2(-halfWidth, halfLength)
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
 * Implements the Separating Axis Theorem for collision detection.
 * This was the most mathematically complex part of the physics system.
 *
 * The idea is to project both boxes onto each potential separating axis
 * and check if there's a gap. If we find ANY axis with a gap, then the
 * boxes don't collide. Otherwise, they must overlap.
 */
bool GameManager::HasSeparatingAxis(const std::vector<glm::vec2>& corners1,
    const std::vector<glm::vec2>& corners2) {
    // Get axes to test (perpendicular to each box's edges)
    std::vector<glm::vec2> axes;
    for (size_t i = 0; i < corners1.size(); i++) {
        glm::vec2 edge = corners1[(i + 1) % corners1.size()] - corners1[i];
        axes.push_back(glm::vec2(-edge.y, edge.x));
    }
    for (size_t i = 0; i < corners2.size(); i++) {
        glm::vec2 edge = corners2[(i + 1) % corners2.size()] - corners2[i];
        axes.push_back(glm::vec2(-edge.y, edge.x));
    }

    // Test projection onto each axis
    for (const auto& axis : axes) {
        float min1 = std::numeric_limits<float>::max();
        float max1 = std::numeric_limits<float>::lowest();
        float min2 = std::numeric_limits<float>::max();
        float max2 = std::numeric_limits<float>::lowest();

        // Project corners onto axis
        for (const auto& corner : corners1) {
            float proj = glm::dot(corner, glm::normalize(axis));
            min1 = std::min(min1, proj);
            max1 = std::max(max1, proj);
        }
        for (const auto& corner : corners2) {
            float proj = glm::dot(corner, glm::normalize(axis));
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

//-------------------------------------------------------------------------------------------------
//  Rendering
//------------------------------------------------------------------------------------------------- 

/**
 * This is the main rendering function that draws everything on screen.
 * I've structured it as a multi-pass renderer:
 * 1. Shadow pass - generates shadow maps
 * 2. Main render pass - renders the scene with lighting and shadows
 * 3. Effects pass - renders particles, debug visualization, etc.
 *
 * Getting all the rendering passes to work together was challenging,
 * especially with the shadow mapping and transparent effects.
 */
void GameManager::Render() {
    // Get current window size
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    // 1. Shadow Pass
    RenderShadowPass();

    // 2. Main Render Pass
    glViewport(0, 0, width, height);
    glClearColor(0.529f, 0.808f, 0.922f, 1.0f);  // Set sky blue color incase skybox is missing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render scene objects
    RenderScene();

    // Render skybox
    RenderSkybox();

    // Render debug and effects
    if (showCollisions) {
        RenderDebugCollisions();
    }
    if (globalLightingEnabled) {
        RenderHeadlightSources();
    }
    // Render particles 
    if (particleEffectsEnabled) {
        antennaParticles->Render(*particleShader, projection, view);
    }
    // Render menu if active
    if (showKeyBindingsMenu) {
        RenderKeyBindingsMenu();
    }
}

/**
 * Performs the shadow pass for rendering.
 * I generate a depth map from the light's perspective, which will be
 * used later to determine which parts of the scene are in shadow.
 *
 * The shadow quality depends a lot on the light projection parameters.
 * I spent quite a bit of time fine-tuning these values.
 */
void GameManager::RenderShadowPass() {
    // Setup shadow matrices with improved parameters
    float near_plane = 1.0f, far_plane = 75.0f;
    glm::mat4 lightProjection = glm::ortho(-40.0f, 40.0f, -40.0f, 40.0f, near_plane, far_plane);
    glm::mat4 lightView = glm::lookAt(
        lights[0].position,
        glm::vec3(0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f)
    );
    lightSpaceMatrix = lightProjection * lightView;

    // Render depth map
    shadowShader.use();
    shadowShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);
    RenderSceneToDepthMap();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/**
 * Renders the main scene with proper lighting and materials.
 * This is where all the visible objects get drawn - arena, cars, static models.
 *
 * I set up all the necessary shader parameters here, including the shadow map
 * that was generated in the shadow pass.
 */
void GameManager::RenderScene() {
    // Setup lighting and matrices
    SetupMatrices();

    // Bind shadow map for all shaders
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, depthMap);

    // Render arena
    arenaShader.use();
    arenaShader.setInt("shadowMap", 2);
    arenaShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
    arena.Render(arenaShader);

    // Render static models
    staticModelShader.use();
    SetupStaticModelMatrices();

    for (auto& staticModel : staticModels) {
        staticModel->Render(staticModelShader);
    }

    // Render player bumper car
    bumperCarShader.use();
    SetupBumperCarMatrices();
    bumperCar.Render(bumperCarShader);

    // Render AI bumper cars
    for (auto& car : aiCars) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), car->GetPosition());
        model = glm::rotate(model, glm::radians(car->GetRotation()), glm::vec3(0.0f, 1.0f, 0.0f));
        bumperCarShader.setMat4("model", model);
        car->Render(bumperCarShader);
    }
}

/**
 * Renders the scene to the depth map for shadow mapping.
 * This is similar to the normal scene rendering, but only outputs depth values
 * without any color or lighting calculations.
 */
void GameManager::RenderSceneToDepthMap() {
    // Render arena to depth map
    glm::mat4 model = glm::mat4(1.0f);
    shadowShader.setMat4("model", model);
    arena.Render(shadowShader);

    // Render player bumper car to depth map
    model = glm::translate(glm::mat4(1.0f), bumperCar.GetPosition());
    model = glm::rotate(model, glm::radians(bumperCar.GetRotation()), glm::vec3(0.0f, 1.0f, 0.0f));
    shadowShader.setMat4("model", model);
    bumperCar.Render(shadowShader);

    // Render AI cars to depth map
    for (const auto& car : aiCars) {
        model = glm::translate(glm::mat4(1.0f), car->GetPosition());
        model = glm::rotate(model, glm::radians(car->GetRotation()), glm::vec3(0.0f, 1.0f, 0.0f));
        shadowShader.setMat4("model", model);
        car->Render(shadowShader);
    }

    // Render static models to depth map
    for (const auto& staticModel : staticModels) {
        staticModel->Render(shadowShader);
    }
}
//-------------------------------------------------------------------------------------------------
//  Free Type Text setup and Render
//------------------------------------------------------------------------------------------------- 

/**
 * Sets up the FreeType library for text rendering.
 * Adding text was something I did later in the project to create a proper UI.
 * It was a bit challenging to integrate FreeType with OpenGL, but it gives
 * much nicer-looking text than manually creating texture atlases would.
 */
void GameManager::InitializeFreetype() {
    std::cout << "Initializing FreeType..." << std::endl;

    // Initialize FreeType library
    FT_Library library;
    if (FT_Init_FreeType(&library)) {
        std::cerr << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        return;
    }
    std::cout << "FreeType library initialized successfully" << std::endl;

    // Try multiple possible paths for the font
    std::vector<std::string> fontPaths = {
        "Resources/font/arial.ttf",
        "./Resources/font/arial.ttf",
        "../Resources/font/arial.ttf",
        "arial.ttf"
    };

    FT_Face face = nullptr;
    std::string loadedPath;
    bool fontLoaded = false;

    for (const auto& path : fontPaths) {
        std::cout << "Trying to load font from: " << path << std::endl;
        if (FT_New_Face(library, path.c_str(), 0, &face) == 0) {
            std::cout << "Successfully loaded font from: " << path << std::endl;
            loadedPath = path;
            fontLoaded = true;
            break;
        }
    }

    if (!fontLoaded) {
        std::cerr << "ERROR::FREETYPE: Failed to load font from any path" << std::endl;
        FT_Done_FreeType(library);
        return;
    }

    // Set font size - use a slightly smaller size for better performance
    if (FT_Set_Pixel_Sizes(face, 0, 32)) {
        std::cerr << "ERROR::FREETYPE: Failed to set font size" << std::endl;
        FT_Done_Face(face);
        FT_Done_FreeType(library);
        return;
    }
    std::cout << "Font size set successfully" << std::endl;

    // Disable byte-alignment restriction
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Load first 128 ASCII characters
    int loadedChars = 0;
    for (unsigned char c = 32; c < 128; c++) {  // Start from space (32) to avoid control characters
        // Load character glyph 
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            std::cerr << "ERROR::FREETYPE: Failed to load Glyph for character " << c << " (" << static_cast<char>(c) << ")" << std::endl;
            continue;
        }

        // Generate texture
        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);

        // Check if the glyph has a valid bitmap
        if (face->glyph->bitmap.width == 0 || face->glyph->bitmap.rows == 0) {
            if (c != 32) {  // Space character often has 0 size
                std::cout << "Warning: Character " << c << " (" << static_cast<char>(c) << ") has empty bitmap" << std::endl;
            }
        }

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            face->glyph->bitmap.buffer
        );

        // Set texture options
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Store character for later use
        Character character = {
            texture,
            glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
            glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
            static_cast<unsigned int>(face->glyph->advance.x)
        };
        Characters.insert(std::pair<char, Character>(c, character));
        loadedChars++;
    }

    std::cout << "Loaded " << loadedChars << " characters successfully" << std::endl;

    // Clean up
    FT_Done_Face(face);
    FT_Done_FreeType(library);

    if (Characters.empty()) {
        std::cerr << "ERROR: No characters were successfully loaded!" << std::endl;
    }
    else {
        std::cout << "FreeType initialization complete!" << std::endl;
    }
}

/**
 * Sets up the OpenGL buffers for text rendering.
 * Each character is rendered as a textured quad, so I need
 * VAOs and VBOs to store and draw those quads.
 */
void GameManager::SetupTextRendering() {
    std::cout << "Setting up text rendering..." << std::endl;

    // Create and compile text shaders
    try {
        textShader = std::make_unique<Shader>("Shaders/text.vert", "Shaders/text.frag");
        std::cout << "Text shaders compiled successfully" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to compile text shaders: " << e.what() << std::endl;
        return;
    }

    // Configure VAO/VBO for texture quads
    glGenVertexArrays(1, &textVAO);
    glGenBuffers(1, &textVBO);

    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO);

    // Allocate a buffer large enough for all character quads I might render
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);

    // Set up vertex attributes for position (xy) and texture coordinates (zw)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);

    // Unbind to prevent accidental modifications
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error during text rendering setup: " << err << std::endl;
    }
    else {
        std::cout << "Text rendering setup complete without errors" << std::endl;
    }
}

/**
 * Renders a string of text at the specified position.
 * This creates a textured quad for each character and positions them
 * according to the font metrics.
 *
 * It was surprisingly tricky to get the text positioning and kerning right,
 * especially since the FreeType coordinate system is different from OpenGL's.
 */
void GameManager::RenderText(const std::string& text, float x, float y, float scale, glm::vec3 color) {
    // Early exit if no characters loaded
    if (Characters.empty()) {
        std::cerr << "ERROR: No characters loaded for text rendering" << std::endl;
        return;
    }

    // Reset any errors that might have occurred before
    glGetError(); // Clear previous errors

    // Activate shader
    textShader->use();
    textShader->setVec3("textColor", color);

    // Make sure VAO and VBO are valid
    if (textVAO == 0 || textVBO == 0) {
        std::cerr << "ERROR: Text VAO or VBO not initialized" << std::endl;
        return;
    }

    // Bind VAO first, then buffer
    glBindVertexArray(textVAO);

    // Ensure I am using texture unit 0
    glActiveTexture(GL_TEXTURE0);
    textShader->setInt("text", 0);

    // Iterate through all characters
    std::string::const_iterator c;
    for (c = text.begin(); c != text.end(); c++) {
        // Skip characters not in our map
        if (Characters.find(*c) == Characters.end()) {
            continue;
        }

        Character ch = Characters[*c];

        // Make sure the texture is valid
        if (ch.TextureID == 0) {
            std::cerr << "Invalid texture for character: " << *c << std::endl;
            continue;
        }

        // Calculate rendering positions
        float xpos = x + ch.Bearing.x * scale;
        float ypos = y - (ch.Size.y - ch.Bearing.y) * scale;
        float w = ch.Size.x * scale;
        float h = ch.Size.y * scale;

        // Skip rendering if the glyph has no size
        if (w <= 0 || h <= 0) {
            // Still advance cursor
            x += (ch.Advance >> 6) * scale;
            continue;
        }

        // Update VBO for each character
        float vertices[6][4] = {
            { xpos,     ypos + h,   0.0f, 0.0f },
            { xpos,     ypos,       0.0f, 1.0f },
            { xpos + w, ypos,       1.0f, 1.0f },
            { xpos,     ypos + h,   0.0f, 0.0f },
            { xpos + w, ypos,       1.0f, 1.0f },
            { xpos + w, ypos + h,   1.0f, 0.0f }
        };

        // First bind the texture
        glBindTexture(GL_TEXTURE_2D, ch.TextureID);

        // Then update buffer data
        glBindBuffer(GL_ARRAY_BUFFER, textVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

        // Draw the glyph
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // Advance cursor position
        x += (ch.Advance >> 6) * scale;
    }

    // Reset state
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

/**
 * Renders the key bindings menu.
 * This displays all the controls for the game in a nicely formatted menu.
 *
 * I added this menu fairly late in development after realizing players
 * had trouble remembering all the keys, especially for camera controls.
 */
void GameManager::RenderKeyBindingsMenu() {
    std::cout << "Rendering key bindings menu..." << std::endl;

    if (Characters.empty()) {
        std::cerr << "WARNING: No characters loaded in FreeType initialization" << std::endl;
        return;
    }

    // Save current OpenGL state
    GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    GLboolean blendEnabled = glIsEnabled(GL_BLEND);
    GLint blendSrc, blendDst;
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDst);

    // Setup appropriate state for text rendering
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Prepare orthographic projection
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(width),
        0.0f, static_cast<float>(height));

    std::cout << "Window size: " << width << "x" << height << std::endl;

    // Use text shader
    if (!textShader) {
        std::cerr << "ERROR: Text shader is not initialized" << std::endl;
        return;
    }

    textShader->use();
    textShader->setMat4("projection", projection);

    // Render semi-transparent black background
    GLuint backgroundVAO, backgroundVBO;
    glGenVertexArrays(1, &backgroundVAO);
    glGenBuffers(1, &backgroundVBO);

    float bgVertices[] = {
        // pos                  // color (semi-transparent black)
        50.0f, 50.0f, 0.0f,     0.0f, 0.0f, 0.0f, 0.8f,
        width - 50.0f, 50.0f, 0.0f,     0.0f, 0.0f, 0.0f, 0.8f,
        width - 50.0f, height - 50.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.8f,
        50.0f, height - 50.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.8f
    };

    glBindVertexArray(backgroundVAO);
    glBindBuffer(GL_ARRAY_BUFFER, backgroundVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(bgVertices), bgVertices, GL_STATIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
    // Color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));

    // Use a simple shader for the background
    if (debugShader) {
        debugShader->use();
        debugShader->setMat4("projection", projection);
        debugShader->setMat4("view", glm::mat4(1.0f)); // Identity view matrix for 2D
        debugShader->setMat4("model", glm::mat4(1.0f)); // Identity model matrix
    }

    // Draw background
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    // Clean up background resources
    glDeleteVertexArrays(1, &backgroundVAO);
    glDeleteBuffers(1, &backgroundVBO);

    // Switch back to text shader
    textShader->use();
    textShader->setMat4("projection", projection);

    // Render text
    float titleScale = 0.8f;  // Slightly smaller for better fit
    float itemScale = 0.4f;   // Smaller for menu items
    float startY = height - 100.0f;
    float lineHeight = 40.0f; // Reduced line height for more compact menu

    // Title
    try {
        std::cout << "Rendering title text..." << std::endl;
        RenderText("UEA BUMPER CARS", 100.0f, startY, titleScale, glm::vec3(1.0f, 1.0f, 1.0f));
    }
    catch (const std::exception& e) {
        std::cerr << "Error rendering title: " << e.what() << std::endl;
    }

    // Section headings and controls
    std::vector<std::pair<std::string, std::string>> controls = {
        {"CAMERA CONTROLS", ""},
        {"1", "Free Camera"},
        {"2", "First-Person Camera"},
        {"3", "Third-Person Camera"},
        {"4", "Ground Camera"},
        {"W/S", "Move Camera Forward/Backward"},
        {"A/D", "Move Camera Left/Right"},
        {"Z/X", "Move Camera Up/Down"},
        {"", ""},
        {"PLAYER CONTROLS", ""},
        {"UP/DOWN Arrow", "Move Forward/Backward"},
        {"LEFT/RIGHT Arrow", "Turn Left/Right"},
        {"", ""},
        {"GAME CONTROLS", ""},
        {"C", "Toggle Collision Boxes"},
        {"P", "Toggle Particle Effects"},
        {"TAB", "Toggle Menu"},
        {"ESC", "Exit Game"}
    };

    for (size_t i = 0; i < controls.size(); ++i) {
        float yPos = startY - (i + 2) * lineHeight;

        // For section headings (entries with empty second string)
        if (controls[i].second.empty() && !controls[i].first.empty()) {
            // Render section headings with different color and slightly larger
            RenderText(controls[i].first, 100.0f, yPos, itemScale * 1.2f, glm::vec3(1.0f, 0.8f, 0.2f));
        }
        else if (!controls[i].first.empty()) {
            // Regular control item with key and description
            std::string key = controls[i].first;
            std::string description = controls[i].second;

            // Render key with white color
            RenderText(key, 100.0f, yPos, itemScale, glm::vec3(1.0f, 1.0f, 1.0f));

            // Render description with light gray
            if (!description.empty()) {
                RenderText(": " + description, 100.0f + 150.0f, yPos, itemScale, glm::vec3(0.8f, 0.8f, 0.8f));
            }
        }
        // Empty lines are just skipped
    }

    // Restore previous OpenGL state
    if (depthTestEnabled) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);

    if (blendEnabled) glEnable(GL_BLEND);
    else glDisable(GL_BLEND);

    glBlendFunc(blendSrc, blendDst);

    std::cout << "Key bindings menu rendering complete" << std::endl;
}

//-----------------------------------------------------------------------------
// Input Handling Implementation
//-----------------------------------------------------------------------------

/**
 * Main input handler that processes all user input.
 * This is where I route input to the appropriate systems - car controls,
 * camera controls, menu toggles, etc.
 *
 * Input handling is more complex than you might think! I had to deal with
 * things like key repeat, toggle states, and prioritizing certain inputs.
 */
void GameManager::HandleInput(GLFWwindow* window) {
    HandleCarInput(deltaTime);
    HandleCameraInput(deltaTime);
    HandleEscapeKey(window);
    HandleCameraModeInput();

    // Show/Hide key bindings menu
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS && !keysProcessed[GLFW_KEY_TAB]) {
        showKeyBindingsMenu = !showKeyBindingsMenu;
        keysProcessed[GLFW_KEY_TAB] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_RELEASE) {
        keysProcessed[GLFW_KEY_TAB] = false;
    }

    // Show/Hide debug visualization
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && !keysProcessed[GLFW_KEY_C]) {
        showCollisions = !showCollisions;
        keysProcessed[GLFW_KEY_C] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_RELEASE) {
        keysProcessed[GLFW_KEY_C] = false;
    }
    // Toggle particle effects with 'P' key
    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS && !keysProcessed[GLFW_KEY_P]) {
        ToggleParticleEffects();
        keysProcessed[GLFW_KEY_P] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_RELEASE) {
        keysProcessed[GLFW_KEY_P] = false;
    }
}

/**
 * Processes inputs for camera mode switching.
 * I added multiple camera modes to give players different ways to experience
 * the game. Each mode has its own feel - from the immersive first-person
 * to the strategic overview of the free camera.
 */
void GameManager::HandleCameraModeInput() {
    // Camera mode switching
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS && !keysProcessed[GLFW_KEY_1]) {
        camera.UpdateMode(Camera::Mode::FREE);
        keysProcessed[GLFW_KEY_1] = true;
    }
    else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS && !keysProcessed[GLFW_KEY_2]) {
        camera.UpdateMode(Camera::Mode::FIRST_PERSON);
        keysProcessed[GLFW_KEY_2] = true;
    }
    else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS && !keysProcessed[GLFW_KEY_3]) {
        camera.UpdateMode(Camera::Mode::THIRD_PERSON);
        keysProcessed[GLFW_KEY_3] = true;
    }
    else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS && !keysProcessed[GLFW_KEY_4]) {
        camera.UpdateMode(Camera::Mode::GROUND);
        keysProcessed[GLFW_KEY_4] = true;
    }

    // Reset key states when released
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_RELEASE) keysProcessed[GLFW_KEY_1] = false;
    if (glfwGetKey(window, GLFW_KEY_2) == GLFW_RELEASE) keysProcessed[GLFW_KEY_2] = false;
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_RELEASE) keysProcessed[GLFW_KEY_3] = false;
    if (glfwGetKey(window, GLFW_KEY_4) == GLFW_RELEASE) keysProcessed[GLFW_KEY_4] = false;
}

/**
 * Processes bumper car control inputs.
 * This maps the arrow keys to car movement commands.
 *
 * I experimented with different control schemes, but found that
 * the simple arrow key controls were the most intuitive for new players.
 */
void GameManager::HandleCarInput(float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        bumperCar.Accelerate(deltaTime);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        bumperCar.Brake(deltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        bumperCar.TurnLeft(deltaTime);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        bumperCar.TurnRight(deltaTime);

    bumperCar.UpdateMovement(deltaTime);
}

/**
 * Processes camera movement inputs.
 * This lets the player move the camera in free and ground modes.
 *
 * I decided to go with WASD controls for camera movement since
 * they're familiar to most gamers. Z and X for up/down movement
 * seemed natural too.
 */
void GameManager::HandleCameraInput(float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
        camera.ProcessKeyboard(Camera_Movement::DOWN, deltaTime);
}

/**
 * Checks for the escape key to exit the game.
 * I made this its own function because it's a special case -
 * it's the only input that can terminate the game.
 */
void GameManager::HandleEscapeKey(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

/**
 * Callback for mouse movement to control camera rotation.
 * This gets called by GLFW whenever the mouse moves, and I use
 * it to update the camera's orientation.
 *
 * Getting the mouse input to feel smooth and responsive took
 * some tuning of sensitivity parameters.
 */
void GameManager::MouseCallback(GLFWwindow* window, double xposIn, double yposIn) {
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

/**
 * Callback for mouse scroll wheel to control camera zoom.
 * This lets the player zoom in and out in free camera mode.
 *
 * It's a small feature, but it makes the free camera much more
 * useful for getting different views of the action.
 */
void GameManager::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

/**
 * Toggles particle effects on/off.
 * I added this as an option because the particles can be a bit performance-heavy
 * on lower-end systems. It's also a nice way to see the game with and without
 * the visual flourishes.
 */
void GameManager::ToggleParticleEffects() {
    particleEffectsEnabled = !particleEffectsEnabled;
    antennaParticles->SetActive(particleEffectsEnabled);
}

/**
 * Toggles the key bindings menu.
 * This is just a convenience function to encapsulate the menu toggle logic.
 */
void GameManager::ToggleKeyBindingsMenu() {
    showKeyBindingsMenu = !showKeyBindingsMenu;
}

//-----------------------------------------------------------------------------
// Camera Control Implementation
//-----------------------------------------------------------------------------

/**
 * Updates the camera position based on the current mode.
 * This is where I handle the different camera behaviors - following the car
 * in third-person mode, positioning at the driver's seat in first-person mode, etc.
 *
 * The camera is a critical part of the game feel, so I spent a lot of time
 * getting these different modes to work smoothly.
 */
void GameManager::UpdateCameraPosition() {
    // Create list of all cars
    std::vector<BumperCar*> allCars = { &bumperCar };
    for (auto& car : aiCars) {
        allCars.push_back(car.get());
    }

    // Update camera based on mode
    switch (camera.GetCurrentMode()) {
    case Camera::Mode::THIRD_PERSON:
        camera.UpdateThirdPerson(bumperCar.GetPosition(), bumperCar.GetRotation());
        break;
    case Camera::Mode::FIRST_PERSON:
        camera.UpdateFirstPerson(bumperCar.GetPosition(), bumperCar.GetRotation());
        break;
    case Camera::Mode::FREE:
    case Camera::Mode::GROUND:
        camera.SetAllCars(allCars);  // Pass cars for collision detection
        break;
    }
}

/**
 * Updates the camera mode.
 * This is a simple wrapper around the camera's mode update function,
 * used when switching between camera modes.
 */
void GameManager::SwitchCamera(Camera::Mode mode) {
    camera.UpdateMode(mode);
}
//-----------------------------------------------------------------------------
// Skybox Implementation
//-----------------------------------------------------------------------------

/**
 * Loads a cubemap texture from 6 image files - one for each face of the cube.
 * Getting a nice skybox makes a huge difference in the visual appeal of the game.
 * I chose a bright daytime sky to match the cheerful feel of a bumper car ride.
 *
 * Creating seamless cubemaps was tricky - I had to make sure the edges of each
 * texture lined up perfectly with adjacent faces.
 */
unsigned int GameManager::LoadCubemap(const std::vector<std::string>& faces)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            GLenum format = GL_RGB;
            if (nrChannels == 1)
                format = GL_RED;
            else if (nrChannels == 3)
                format = GL_RGB;
            else if (nrChannels == 4)
                format = GL_RGBA;

            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap tex failed to load at path: " << faces[i] << std::endl;
            stbi_image_free(data);
        }
    }

    // Enable seamless cubemap sampling to eliminate edge seams
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    // Setup filtering and mipmapping for smoother appearance
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Add anisotropic filtering if supported by the hardware
    float maxAniso = 0.0f;
    if (glfwExtensionSupported("GL_EXT_texture_filter_anisotropic")) {
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
        glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);
    }

    // Generate mipmaps for better appearance at different distances
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    return textureID;
}

/**
 * Sets up the skybox geometry, shader, and textures.
 * The skybox is just a cube with textures rendered on the inside,
 * but there are some special tricks needed to make it work properly.
 *
 * For example, I had to modify the depth buffer behavior to ensure
 * the skybox always appears behind everything else.
 */
void GameManager::InitializeSkybox()
{
    // Create skybox shader
    skyboxShader = std::make_unique<Shader>("Shaders/skybox.vert", "Shaders/skybox.frag");

    // Skybox vertices
    float skyboxVertices[] = {
        // positions          
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        -1.0f,  1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f
    };

    // Setup skybox VAO and VBO
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    // Load cubemap
    std::vector<std::string> faces = {
    "Resources/textures/environment/sh_rt.png",  // Right (+X)
    "Resources/textures/environment/sh_lf.png",  // Left (-X)
    "Resources/textures/environment/sh_up.png",  // Top (+Y)
    "Resources/textures/environment/sh_dn.png",  // Bottom (-Y)
    "Resources/textures/environment/sh_bk.png",  // Back (+Z)
    "Resources/textures/environment/sh_ft.png"   // Front (-Z)
    };
    cubemapTexture = LoadCubemap(faces);
}

/**
 * Renders the skybox.
 * I render the skybox last (except for UI elements) and use a special
 * depth function to ensure it appears behind everything else in the scene.
 *
 * One trick here is removing the translation part of the view matrix,
 * so the skybox always stays centered around the camera.
 */
void GameManager::RenderSkybox()
{
    // Render skybox last and with depth function set to less than or equal
    glDepthFunc(GL_LEQUAL);

    skyboxShader->use();

    // Remove translation from view matrix by converting to 3x3 then back to 4x4
    // This ensures the skybox moves with camera rotation but not translation
    glm::mat4 view = glm::mat4(glm::mat3(camera.GetViewMatrix()));

    skyboxShader->setMat4("view", view);
    skyboxShader->setMat4("projection", projection);

    // Render skybox
    glBindVertexArray(skyboxVAO);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);

    // Set depth function back to default
    glDepthFunc(GL_LESS);
}

//-----------------------------------------------------------------------------
// Add and position my static Models
//-----------------------------------------------------------------------------

/**
 * Creates and places static models in the scene.
 * I added things like a Ferris wheel and trees to make the environment
 * more interesting than just an empty arena. These visual elements
 * help create the fun fairground atmosphere I was going for.
 *
 * Positioning these models took some trial and error to find good
 * locations that frame the arena without getting in the way of gameplay.
 */
void GameManager::InitializeStaticModels() {
    // Create Ferris Wheel
    
    auto ferrisWheel = std::make_unique<StaticModel>("Resources/models/ferriswheel/3d-model.obj");
    ferrisWheel->Initialize();
    ferrisWheel->SetScale(0.2f);
    ferrisWheel->SetPosition(glm::vec3(16.0f, -0.85f, 21.0f));
    ferrisWheel->SetRotation(150.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    staticModels.push_back(std::move(ferrisWheel));

    std::cout << "Ferris Wheel model initialized and added to static models." << std::endl;
    
    // Tree Group 1
    auto Tree1 = std::make_unique<StaticModel>("Resources/models/tree/PineTree.obj");
    Tree1->Initialize();
    Tree1->SetScale(0.21f);
    Tree1->SetPosition(glm::vec3(-18.0f, -1.0f, -15.0f));
    Tree1->SetRotation(100.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    staticModels.push_back(std::move(Tree1));

    std::cout << "Tree Group 1 model initialized and added to static models." << std::endl;
    
    // Tree Group 2
    auto Tree2 = std::make_unique<StaticModel>("Resources/models/tree/PineTree.obj");
    Tree2->Initialize();
    Tree2->SetScale(0.2f);
    Tree2->SetPosition(glm::vec3(5.0f, -1.0f, -28.0f));
    Tree2->SetRotation(265.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    staticModels.push_back(std::move(Tree2));

    std::cout << "Tree Group 2 model initialized and added to static models." << std::endl;
    
    // Tree Group 3
    auto Tree3 = std::make_unique<StaticModel>("Resources/models/tree/PineTree.obj");
    Tree3->Initialize();
    Tree3->SetScale(0.22f);
    Tree3->SetPosition(glm::vec3(23.0f, -1.0f, -25.0f));
    Tree3->SetRotation(225.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    staticModels.push_back(std::move(Tree3));

    std::cout << "Tree Group 3 model initialized and added to static models." << std::endl;
    
    // Tree Group 4
    auto Tree4 = std::make_unique<StaticModel>("Resources/models/tree/PineTree.obj");
    Tree4->Initialize();
    Tree4->SetScale(0.21f);
    Tree4->SetPosition(glm::vec3(-13.0f, -1.0f, 16.0f));
    Tree4->SetRotation(260.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    staticModels.push_back(std::move(Tree4));

    std::cout << "Tree Group 4 model initialized and added to static models." << std::endl;
    
    // sign model
    auto signModel = std::make_unique<StaticModel>("Resources/models/sign/Sign.obj");
    signModel->Initialize();
    signModel->SetScale(0.9f);  // Adjust scale as needed
    signModel->SetPosition(glm::vec3(1.0f, 0.0f, 18.0f));  // Position in grass area
    signModel->SetRotation(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));  // Angle it toward the player
    staticModels.push_back(std::move(signModel));

    std::cout << "UEA Sign model initialized and added to static models." << std::endl;

    // table model
    auto tableModel = std::make_unique<StaticModel>("Resources/models/table/table.obj");
    tableModel->Initialize();
    
    tableModel->SetScale(88.0f);  // Start with a large scale to fit the arena on top
    tableModel->SetPosition(glm::vec3(-154.0f, -181.0f, -50.0f));  // Position below the arena
    tableModel->SetRotation(0.0f, glm::vec3(0.0f, 1.0f, 0.0f));  // No rotation initially
    staticModels.push_back(std::move(tableModel));

    
    std::cout << "Table model initialized and added to static models." << std::endl;

    // controller model
    auto controllerModel = std::make_unique<StaticModel>("Resources/models/controller/controller.obj");
    controllerModel->Initialize();
    controllerModel->SetScale(0.2f);  // Adjust scale as needed
    controllerModel->SetPosition(glm::vec3(24.0f, -1.0f, 5.0f));  // Position on table
    controllerModel->SetRotation(165.0f, glm::vec3(0.0f, 1.0f, 0.0f));  // Angle it toward the player
    staticModels.push_back(std::move(controllerModel));

    std::cout << "Controller model initialized and added to static models." << std::endl;
}


//-----------------------------------------------------------------------------
// Matrix Updates
//-----------------------------------------------------------------------------

/**
 * Updates the projection matrix based on window size and camera zoom.
 * This needs to be called whenever the window size changes or the
 * camera zoom level changes. Getting the projection right is crucial
 * for proper 3D rendering I noticed bits missing when my calulations were off.
 */
void GameManager::UpdateProjectionMatrix() {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    projection = glm::perspective(
        glm::radians(camera.GetZoom()),
        (float)width / (float)height,
        0.1f,   //near clipping
        200.0f  //far clipping
    );
}

/**
 * Updates the view matrix based on camera position and orientation.
 * The view matrix transforms world-space coordinates to view space,
 * essentially positioning everything relative to the camera's viewpoint.
 */
void GameManager::UpdateViewMatrix() {
    view = camera.GetViewMatrix();
}

//-----------------------------------------------------------------------------
// Lighting Implementation
//-----------------------------------------------------------------------------

/**
 * Sets up all the lights in the scene.
 * I created a combination of different light types to make the scene look interesting:
 * - A main sun-like directional light
 * - Colored corner lights that give a funfair atmosphere
 * - Car headlights that illuminate the path ahead
 */
void GameManager::InitializeLighting() {
    // Main sunlight
    lights.push_back({
        glm::vec3(-20.0f, 30.0f, -25.0f),   // Keep high sun position
        glm::vec3(1.0f, 0.98f, 0.95f),      // Natural sunlight color
        2.5f,                               // Intensity
        false,                              // Not flashing
        0.0f,                               // No flash speed
        0.0f                                // No flash timer
        });

    // Initialize corner lights 4 different colors to show fair effect
    const std::vector<LightConfig> cornerLights = {
        {glm::vec3(-12.0f, 2.5f, -12.0f), glm::vec3(1.0f, 0.3f, 0.3f)},  // Red
        {glm::vec3(12.0f, 2.5f, -12.0f), glm::vec3(0.3f, 0.3f, 1.0f)},   // Blue
        {glm::vec3(12.0f, 2.5f, 12.0f), glm::vec3(1.0f, 0.9f, 0.3f)},    // Yellow
        {glm::vec3(-12.0f, 2.5f, 12.0f), glm::vec3(0.3f, 1.0f, 0.3f)}    // Green
    };

    for (const auto& config : cornerLights) {
        lights.push_back({
            config.position,
            config.color,
            0.4f,    // Reduced from 0.6 to 0.4
            true,    // Still flashing
            1.5f,    // Flash speed
            0.0f     // Initial timer
            });
    }

    // Initialize headlights with reduced intensity
    headlights.resize(2);
    for (auto& headlight : headlights) {
        headlight.color = glm::vec3(1.0f, 0.97f, 0.9f);  // Bright white
        headlight.intensity = 1.0f;                      // Reduced from 1.5 to 1.0
        headlight.cutOff = glm::cos(glm::radians(12.5f));
        headlight.outerCutOff = glm::cos(glm::radians(17.5f));
        headlight.constant = 1.0f;
        headlight.linear = 0.09f;
        headlight.quadratic = 0.032f;
    }
}

/**
 * Updates light parameters each frame.
 * Some lights have dynamic effects like flashing or movement,
 * so they need to be updated every frame.
 *
 * I like the way the flashing lights add a bit of energy and movement
 * to the scene, making it feel more like an active fairground.
 */
void GameManager::UpdateLighting() {
    if (!globalLightingEnabled) return;

    totalTime += deltaTime;

    // Update flashing lights
    for (size_t i = 1; i < lights.size(); i++) {
        if (lights[i].isFlashing) {
            lights[i].flashTimer += deltaTime * lights[i].flashSpeed;
            lights[i].intensity = 0.3f + 0.3f * (sin(lights[i].flashTimer) + 1.0f);
        }
    }

    // Update headlight positions
    UpdateHeadlightPositions();
}

/**
 * Updates the car headlight positions and directions.
 * Since the headlights are attached to the car, they need to
 * move and rotate with it. This calculates their transform
 * based on the car's current position and orientation.
 */
void GameManager::UpdateHeadlightPositions() {
    glm::vec3 carPos = bumperCar.GetPosition();
    float carRotation = glm::radians(bumperCar.GetRotation());
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));

    const std::vector<glm::vec3> headlightOffsets = {
        glm::vec3(BumperCar::LEFT_HEADLIGHT_OFFSET_X,
                  BumperCar::LEFT_HEADLIGHT_OFFSET_Y,
                  BumperCar::LEFT_HEADLIGHT_OFFSET_Z),
        glm::vec3(BumperCar::RIGHT_HEADLIGHT_OFFSET_X,
                  BumperCar::RIGHT_HEADLIGHT_OFFSET_Y,
                  BumperCar::RIGHT_HEADLIGHT_OFFSET_Z)
    };

    glm::vec3 forward = glm::vec3(-sin(carRotation), 0.0f, -cos(carRotation));

    for (size_t i = 0; i < 2; i++) {
        glm::vec4 worldOffset = rotationMatrix * glm::vec4(headlightOffsets[i], 1.0f);
        headlights[i].position = carPos + glm::vec3(worldOffset);
        headlights[i].direction = forward;
    }
}

//-----------------------------------------------------------------------------
// Debug Visualization Implementation
//-----------------------------------------------------------------------------

/**
 * Sets up the debug visualization system.
 * This was invaluable during development for seeing things like collision
 * boxes, turning centers, and other invisible game elements.
 *
 * I liked it so much I kept it in the final game as a togglable feature
 * to show the markers my proccess!
 */
void GameManager::InitializeDebugVisualization() {
    debugShader = std::make_unique<Shader>("Shaders/debug.vert", "Shaders/debug.frag");

    glGenVertexArrays(1, &debugVAO);
    glGenBuffers(1, &debugVBO);

    glBindVertexArray(debugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugVBO);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)0);

    // Color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void*)offsetof(DebugVertex, color));

    glBindVertexArray(0);
}

/**
 * Renders the debug visualizations for collisions and other game elements.
 * This draws lines to represent collision boxes, rotation centers, and
 * other invisible elements that are helpful to see during development.
 */
void GameManager::RenderDebugCollisions() {
    if (!showCollisions) return;

    UpdateDebugVertices();

    debugShader->use();
    debugShader->setMat4("projection", projection);
    debugShader->setMat4("view", view);
    debugShader->setMat4("model", glm::mat4(1.0f));

    glDisable(GL_DEPTH_TEST);
    glBindVertexArray(debugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
    glBufferData(GL_ARRAY_BUFFER, debugVertices.size() * sizeof(DebugVertex),
        debugVertices.data(), GL_DYNAMIC_DRAW);

    glDrawArrays(GL_LINES, 0, debugVertices.size());

    glEnable(GL_DEPTH_TEST);
}

/**
 * Updates the debug visualization vertices for the current frame.
 * This is where I gather all the data about what to visualize -
 * car collision boxes, arena walls, turning centers, etc.
 *
 * I use different colors for different types of elements to make
 * it easier to distinguish them visually.
 */
void GameManager::UpdateDebugVertices() {
    debugVertices.clear();

    // Arena wall collisions
    float arenaHalfWidth = 15.0f;
    float arenaHalfLength = 15.0f;
    float wallHeight = 0.5f;

    // Front wall
    AddDebugLine(glm::vec3(-arenaHalfWidth, 0, -arenaHalfLength), glm::vec3(arenaHalfWidth, 0, -arenaHalfLength));
    AddDebugLine(glm::vec3(-arenaHalfWidth, wallHeight, -arenaHalfLength), glm::vec3(arenaHalfWidth, wallHeight, -arenaHalfLength));
    AddDebugLine(glm::vec3(-arenaHalfWidth, 0, -arenaHalfLength), glm::vec3(-arenaHalfWidth, wallHeight, -arenaHalfLength));
    AddDebugLine(glm::vec3(arenaHalfWidth, 0, -arenaHalfLength), glm::vec3(arenaHalfWidth, wallHeight, -arenaHalfLength));

    // Back wall
    AddDebugLine(glm::vec3(-arenaHalfWidth, 0, arenaHalfLength), glm::vec3(arenaHalfWidth, 0, arenaHalfLength));
    AddDebugLine(glm::vec3(-arenaHalfWidth, wallHeight, arenaHalfLength), glm::vec3(arenaHalfWidth, wallHeight, arenaHalfLength));
    AddDebugLine(glm::vec3(-arenaHalfWidth, 0, arenaHalfLength), glm::vec3(-arenaHalfWidth, wallHeight, arenaHalfLength));
    AddDebugLine(glm::vec3(arenaHalfWidth, 0, arenaHalfLength), glm::vec3(arenaHalfWidth, wallHeight, arenaHalfLength));

    // Left wall
    AddDebugLine(glm::vec3(-arenaHalfWidth, 0, -arenaHalfLength), glm::vec3(-arenaHalfWidth, 0, arenaHalfLength));
    AddDebugLine(glm::vec3(-arenaHalfWidth, wallHeight, -arenaHalfLength), glm::vec3(-arenaHalfWidth, wallHeight, arenaHalfLength));

    // Right wall
    AddDebugLine(glm::vec3(arenaHalfWidth, 0, -arenaHalfLength), glm::vec3(arenaHalfWidth, 0, arenaHalfLength));
    AddDebugLine(glm::vec3(arenaHalfWidth, wallHeight, -arenaHalfLength), glm::vec3(arenaHalfWidth, wallHeight, arenaHalfLength));

    // Bumper car collision box
    glm::vec3 carPos = bumperCar.GetPosition();
    float carRotation = glm::radians(bumperCar.GetRotation());

    // Create rotation matrix for car's orientation
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));

    // Calculate offset in car's local space
    glm::vec3 localOffset(
        BumperCar::COLLISION_OFFSET_X,
        BumperCar::COLLISION_OFFSET_Y,
        BumperCar::COLLISION_OFFSET_Z
    );

    // Transform offset to world space
    glm::vec4 worldOffset = rotationMatrix * glm::vec4(localOffset, 1.0f);

    // Calculate collision box center
    glm::vec3 boxCenter = carPos + glm::vec3(worldOffset);

    // Calculate the eight corners of the box in local space
    float boxHalfLength = BumperCar::COLLISION_BOX_LENGTH * 0.5f;
    float boxHalfWidth = BumperCar::COLLISION_BOX_WIDTH * 0.5f;
    float boxHalfHeight = BumperCar::COLLISION_BOX_HEIGHT * 0.5f;

    std::vector<glm::vec3> corners = {
        // Bottom corners
        glm::vec3(-boxHalfWidth, -boxHalfHeight, -boxHalfLength),  // Back left
        glm::vec3(boxHalfWidth, -boxHalfHeight, -boxHalfLength),   // Back right
        glm::vec3(boxHalfWidth, -boxHalfHeight, boxHalfLength),    // Front right
        glm::vec3(-boxHalfWidth, -boxHalfHeight, boxHalfLength),   // Front left
        // Top corners
        glm::vec3(-boxHalfWidth, boxHalfHeight, -boxHalfLength),   // Back left
        glm::vec3(boxHalfWidth, boxHalfHeight, -boxHalfLength),    // Back right
        glm::vec3(boxHalfWidth, boxHalfHeight, boxHalfLength),     // Front right
        glm::vec3(-boxHalfWidth, boxHalfHeight, boxHalfLength)     // Front left
    };

    // Transform and draw the box
    for (auto& corner : corners) {
        // Rotate corner
        glm::vec4 rotatedCorner = rotationMatrix * glm::vec4(corner, 1.0f);
        // Translate to final position
        corner = boxCenter + glm::vec3(rotatedCorner);
    }

    // Draw bottom rectangle
    AddDebugLine(corners[0], corners[1]);
    AddDebugLine(corners[1], corners[2]);
    AddDebugLine(corners[2], corners[3]);
    AddDebugLine(corners[3], corners[0]);

    // Draw top rectangle
    AddDebugLine(corners[4], corners[5]);
    AddDebugLine(corners[5], corners[6]);
    AddDebugLine(corners[6], corners[7]);
    AddDebugLine(corners[7], corners[4]);

    // Draw vertical edges
    AddDebugLine(corners[0], corners[4]);
    AddDebugLine(corners[1], corners[5]);
    AddDebugLine(corners[2], corners[6]);
    AddDebugLine(corners[3], corners[7]);

    // Draw center reference point
    AddDebugLine(
        boxCenter - glm::vec3(0.1f, 0, 0),
        boxCenter + glm::vec3(0.1f, 0, 0)
    );
    AddDebugLine(
        boxCenter - glm::vec3(0, 0, 0.1f),
        boxCenter + glm::vec3(0, 0, 0.1f)
    );

    // Calculate and draw turning center point
    float rearOffset = BumperCar::TURN_REAR_OFFSET;
    float lateralOffset = BumperCar::TURN_LATERAL_OFFSET;
    glm::vec3 forward = glm::vec3(-sin(carRotation), 0.0f, -cos(carRotation));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 turningPoint = carPos + forward * rearOffset + right * lateralOffset;

    // Draw yellow marker for turning point
    float markerSize = 0.15f;
    glm::vec4 yellowColor(1.0f, 1.0f, 0.0f, 1.0f);
    AddDebugLine(
        turningPoint - glm::vec3(markerSize, 0, 0),
        turningPoint + glm::vec3(markerSize, 0, 0),
        yellowColor
    );
    AddDebugLine(
        turningPoint - glm::vec3(0, 0, markerSize),
        turningPoint + glm::vec3(0, 0, markerSize),
        yellowColor
    );

    // Calculate and draw antenna point
    glm::vec3 localAntennaOffset(
        BumperCar::ANTENNA_OFFSET_X,
        BumperCar::ANTENNA_OFFSET_Y,
        BumperCar::ANTENNA_OFFSET_Z
    );
    glm::vec4 antennaWorldOffset = rotationMatrix * glm::vec4(localAntennaOffset, 1.0f);
    glm::vec3 antennaPoint = carPos + glm::vec3(antennaWorldOffset);

    // Draw green cross for antenna
    float antennaCrossSize = 0.1f;
    glm::vec4 greenColor(0.0f, 1.0f, 0.0f, 1.0f);

    // X-axis line
    AddDebugLine(
        antennaPoint - glm::vec3(antennaCrossSize, 0, 0),
        antennaPoint + glm::vec3(antennaCrossSize, 0, 0),
        greenColor
    );
    // Y-axis line
    AddDebugLine(
        antennaPoint - glm::vec3(0, antennaCrossSize, 0),
        antennaPoint + glm::vec3(0, antennaCrossSize, 0),
        greenColor
    );
    // Z-axis line
    AddDebugLine(
        antennaPoint - glm::vec3(0, 0, antennaCrossSize),
        antennaPoint + glm::vec3(0, 0, antennaCrossSize),
        greenColor
    );

    // Calculate and draw headlight positions
    glm::vec3 leftHeadlightLocal(
        BumperCar::LEFT_HEADLIGHT_OFFSET_X,
        BumperCar::LEFT_HEADLIGHT_OFFSET_Y,
        BumperCar::LEFT_HEADLIGHT_OFFSET_Z
    );
    glm::vec3 rightHeadlightLocal(
        BumperCar::RIGHT_HEADLIGHT_OFFSET_X,
        BumperCar::RIGHT_HEADLIGHT_OFFSET_Y,
        BumperCar::RIGHT_HEADLIGHT_OFFSET_Z
    );

    glm::vec4 leftHeadlightWorld = rotationMatrix * glm::vec4(leftHeadlightLocal, 1.0f);
    glm::vec4 rightHeadlightWorld = rotationMatrix * glm::vec4(rightHeadlightLocal, 1.0f);

    glm::vec3 leftHeadlightPos = carPos + glm::vec3(leftHeadlightWorld);
    glm::vec3 rightHeadlightPos = carPos + glm::vec3(rightHeadlightWorld);

    // Draw blue crosses for headlights
    float headlightCrossSize = 0.1f;
    glm::vec4 blueColor(0.0f, 0.3f, 1.0f, 1.0f);

    // Left headlight
    AddDebugLine(
        leftHeadlightPos - glm::vec3(headlightCrossSize, 0, 0),
        leftHeadlightPos + glm::vec3(headlightCrossSize, 0, 0),
        blueColor
    );
    AddDebugLine(
        leftHeadlightPos - glm::vec3(0, headlightCrossSize, 0),
        leftHeadlightPos + glm::vec3(0, headlightCrossSize, 0),
        blueColor
    );
    AddDebugLine(
        leftHeadlightPos - glm::vec3(0, 0, headlightCrossSize),
        leftHeadlightPos + glm::vec3(0, 0, headlightCrossSize),
        blueColor
    );

    // Right headlight
    AddDebugLine(
        rightHeadlightPos - glm::vec3(headlightCrossSize, 0, 0),
        rightHeadlightPos + glm::vec3(headlightCrossSize, 0, 0),
        blueColor
    );
    AddDebugLine(
        rightHeadlightPos - glm::vec3(0, headlightCrossSize, 0),
        rightHeadlightPos + glm::vec3(0, headlightCrossSize, 0),
        blueColor
    );
    AddDebugLine(
        rightHeadlightPos - glm::vec3(0, 0, headlightCrossSize),
        rightHeadlightPos + glm::vec3(0, 0, headlightCrossSize),
        blueColor
    );
    // Draw collision boxes for all AI cars
    for (const auto& aiCar : aiCars) {
        glm::vec3 carPos = aiCar->GetPosition();
        float carRotation = glm::radians(aiCar->GetRotation());

        // Create rotation matrix for car's orientation
        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), carRotation, glm::vec3(0.0f, 1.0f, 0.0f));

        // Calculate offset in car's local space
        glm::vec3 localOffset(
            BumperCar::COLLISION_OFFSET_X,
            BumperCar::COLLISION_OFFSET_Y,
            BumperCar::COLLISION_OFFSET_Z
        );

        // Transform offset to world space
        glm::vec4 worldOffset = rotationMatrix * glm::vec4(localOffset, 1.0f);

        // Calculate collision box center
        glm::vec3 boxCenter = carPos + glm::vec3(worldOffset);

        // Calculate the eight corners of the box in local space
        float boxHalfLength = BumperCar::COLLISION_BOX_LENGTH * 0.5f;
        float boxHalfWidth = BumperCar::COLLISION_BOX_WIDTH * 0.5f;
        float boxHalfHeight = BumperCar::COLLISION_BOX_HEIGHT * 0.5f;

        std::vector<glm::vec3> corners = {
            // Bottom corners
            glm::vec3(-boxHalfWidth, -boxHalfHeight, -boxHalfLength),  // Back left
            glm::vec3(boxHalfWidth, -boxHalfHeight, -boxHalfLength),   // Back right
            glm::vec3(boxHalfWidth, -boxHalfHeight, boxHalfLength),    // Front right
            glm::vec3(-boxHalfWidth, -boxHalfHeight, boxHalfLength),   // Front left
            // Top corners
            glm::vec3(-boxHalfWidth, boxHalfHeight, -boxHalfLength),   // Back left
            glm::vec3(boxHalfWidth, boxHalfHeight, -boxHalfLength),    // Back right
            glm::vec3(boxHalfWidth, boxHalfHeight, boxHalfLength),     // Front right
            glm::vec3(-boxHalfWidth, boxHalfHeight, boxHalfLength)     // Front left
        };

        // Transform and draw the box with a different color for AI cars
        for (auto& corner : corners) {
            // Rotate corner
            glm::vec4 rotatedCorner = rotationMatrix * glm::vec4(corner, 1.0f);
            // Translate to final position
            corner = boxCenter + glm::vec3(rotatedCorner);
        }

        // Use a different color for AI car collision boxes (cyan)
        glm::vec4 aiCarColor(0.0f, 1.0f, 1.0f, 1.0f);

        // Draw bottom rectangle
        AddDebugLine(corners[0], corners[1], aiCarColor);
        AddDebugLine(corners[1], corners[2], aiCarColor);
        AddDebugLine(corners[2], corners[3], aiCarColor);
        AddDebugLine(corners[3], corners[0], aiCarColor);

        // Draw top rectangle
        AddDebugLine(corners[4], corners[5], aiCarColor);
        AddDebugLine(corners[5], corners[6], aiCarColor);
        AddDebugLine(corners[6], corners[7], aiCarColor);
        AddDebugLine(corners[7], corners[4], aiCarColor);

        // Draw vertical edges
        AddDebugLine(corners[0], corners[4], aiCarColor);
        AddDebugLine(corners[1], corners[5], aiCarColor);
        AddDebugLine(corners[2], corners[6], aiCarColor);
        AddDebugLine(corners[3], corners[7], aiCarColor);

        // Draw center reference point
        AddDebugLine(
            boxCenter - glm::vec3(0.1f, 0, 0),
            boxCenter + glm::vec3(0.1f, 0, 0),
            aiCarColor
        );
        AddDebugLine(
            boxCenter - glm::vec3(0, 0, 0.1f),
            boxCenter + glm::vec3(0, 0, 0.1f),
            aiCarColor
        );
    }
    // Camera collision visualization (for ground and free modes)
    if (camera.GetCurrentMode() == Camera::Mode::GROUND ||
        camera.GetCurrentMode() == Camera::Mode::FREE) {
        glm::vec3 camPos = camera.GetPosition();
        float camRadius = 1.3f; // Match Camera.h collision radius
        const int segments = 32;

        for (int i = 0; i < segments; i++) {
            float angle1 = (float)i / segments * 2.0f * 3.14159f;
            float angle2 = (float)(i + 1) / segments * 2.0f * 3.14159f;

            glm::vec3 p1(camPos.x + camRadius * cos(angle1), camPos.y, camPos.z + camRadius * sin(angle1));
            glm::vec3 p2(camPos.x + camRadius * cos(angle2), camPos.y, camPos.z + camRadius * sin(angle2));

            AddDebugLine(p1, p2);
        }
    }
}

/**
 * Adds a debug line with default red color.
 * This is a helper function that makes it easier to add visualization lines.
 */
void GameManager::AddDebugLine(const glm::vec3& start, const glm::vec3& end) {
    DebugVertex v1 = { start, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) };
    DebugVertex v2 = { end, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) };
    debugVertices.push_back(v1);
    debugVertices.push_back(v2);
}

/**
 * Adds a debug line with custom color.
 * This variation lets me choose different colors for different types
 * of visualization elements, making it easier to distinguish them.
 */
void GameManager::AddDebugLine(const glm::vec3& start, const glm::vec3& end, const glm::vec4& color) {
    DebugVertex v1 = { start, color };
    DebugVertex v2 = { end, color };
    debugVertices.push_back(v1);
    debugVertices.push_back(v2);
}

//-----------------------------------------------------------------------------
// Headlight Source Implementation
//-----------------------------------------------------------------------------

/**
 * Sets up the visual representations of headlight sources.
 * These are small glowing spheres that appear at the headlight positions
 * on the car. They don't actually produce light (that's handled by the
 * spotlight in the lighting system), but they give a nice visual cue
 * that the headlights are there.
 */
void GameManager::InitializeHeadlightSources() {
    const float radius = 0.05f;  // Small radius for the light source
    const int segments = 16;
    const int rings = 16;

    for (int h = 0; h < 2; h++) {
        auto& source = headlightSources[h];
        GenerateSphereMesh(source, radius, segments, rings);
        SetupHeadlightBuffers(source);
    }
}

/**
 * Creates a sphere mesh for representing a headlight source.
 * I use spheres for the headlight sources because they look like
 * glowing bulbs when rendered with the right shader.
 */
void GameManager::GenerateSphereMesh(HeadlightSource& source, float radius, int segments, int rings) {
    for (int i = 0; i <= rings; i++) {
        float phi = glm::pi<float>() * (float)i / rings;
        for (int j = 0; j <= segments; j++) {
            float theta = 2.0f * glm::pi<float>() * (float)j / segments;

            float x = radius * sin(phi) * cos(theta);
            float y = radius * cos(phi);
            float z = radius * sin(phi) * sin(theta);

            glm::vec3 vertex(x, y, z);
            source.vertices.push_back(vertex);
            source.normals.push_back(glm::normalize(vertex));
        }
    }

    for (int i = 0; i < rings; i++) {
        for (int j = 0; j < segments; j++) {
            int first = i * (segments + 1) + j;
            int second = first + segments + 1;

            source.indices.push_back(first);
            source.indices.push_back(second);
            source.indices.push_back(first + 1);

            source.indices.push_back(second);
            source.indices.push_back(second + 1);
            source.indices.push_back(first + 1);
        }
    }
}

/**
 * Sets up the OpenGL buffers for headlight source rendering.
 * This configures the VAO, VBO, and EBO for the headlight spheres.
 */
void GameManager::SetupHeadlightBuffers(HeadlightSource& source) {
    glGenVertexArrays(1, &source.VAO);
    glGenBuffers(1, &source.VBO);
    glGenBuffers(1, &source.EBO);

    glBindVertexArray(source.VAO);

    // Setup vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, source.VBO);
    size_t vertexDataSize = source.vertices.size() * sizeof(glm::vec3) +
        source.normals.size() * sizeof(glm::vec3);
    glBufferData(GL_ARRAY_BUFFER, vertexDataSize, nullptr, GL_STATIC_DRAW);

    // Upload vertex and normal data
    glBufferSubData(GL_ARRAY_BUFFER, 0,
        source.vertices.size() * sizeof(glm::vec3), source.vertices.data());
    glBufferSubData(GL_ARRAY_BUFFER, source.vertices.size() * sizeof(glm::vec3),
        source.normals.size() * sizeof(glm::vec3), source.normals.data());

    // Setup index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, source.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, source.indices.size() * sizeof(unsigned int),
        source.indices.data(), GL_STATIC_DRAW);

    // Setup vertex attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0,
        (void*)(source.vertices.size() * sizeof(glm::vec3)));

    glBindVertexArray(0);
}

/**
 * Renders the headlight source objects.
 * These are the glowing spheres that represent the headlight bulbs.
 * I use additive blending to create a nice glow effect.
 */
void GameManager::RenderHeadlightSources() {
    if (!globalLightingEnabled) return;

    headlightSourceShader->use();
    headlightSourceShader->setMat4("projection", projection);
    headlightSourceShader->setMat4("view", view);

    // Setup blending for glow effect
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glDepthMask(GL_FALSE);

    for (int i = 0; i < 2; i++) {
        const auto& source = headlightSources[i];

        // Setup model matrix
        glm::mat4 model = glm::translate(glm::mat4(1.0f), headlights[i].position);
        headlightSourceShader->setMat4("model", model);

        // Set light properties
        headlightSourceShader->setVec3("lightColor", headlights[i].color);
        headlightSourceShader->setFloat("intensity", headlights[i].intensity * 2.0f);

        // Render headlight source
        glBindVertexArray(source.VAO);
        glDrawElements(GL_TRIANGLES, source.indices.size(), GL_UNSIGNED_INT, 0);
    }

    // Restore OpenGL state
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
}

//-----------------------------------------------------------------------------
// Matrices Setup Implementation
//-----------------------------------------------------------------------------

/**
 * Sets up the matrices and lighting parameters for scene rendering.
 * This is where I configure all the shader uniforms that control
 * how the scene is rendered.
 */
void GameManager::SetupMatrices() {
    arenaShader.use();
    arenaShader.setMat4("projection", projection);
    arenaShader.setMat4("view", view);
    arenaShader.setVec3("viewPos", camera.GetPosition());

    // Shadow mapping setup for arena
    arenaShader.setInt("shadowMap", 2);
    arenaShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

    SetupLightingUniforms();
}

/**
 * Sets up the lighting uniform values for shaders.
 * This transfers all the light properties (position, color, etc.)
 * to the shader for use in lighting calculations.
 */
void GameManager::SetupLightingUniforms() {
    // Setup regular lights
    arenaShader.setInt("numLights", static_cast<int>(lights.size()));
    for (size_t i = 0; i < lights.size(); i++) {
        std::string base = "lights[" + std::to_string(i) + "].";
        arenaShader.setVec3(base + "position", lights[i].position);
        arenaShader.setVec3(base + "color", lights[i].color);
        arenaShader.setFloat(base + "intensity", lights[i].intensity);
    }

    // Setup headlights
    arenaShader.setInt("numSpotLights", static_cast<int>(headlights.size()));
    for (size_t i = 0; i < headlights.size(); i++) {
        std::string base = "spotLights[" + std::to_string(i) + "].";
        const auto& light = headlights[i];

        arenaShader.setVec3(base + "position", light.position);
        arenaShader.setVec3(base + "direction", light.direction);
        arenaShader.setVec3(base + "color", light.color);
        arenaShader.setFloat(base + "intensity", light.intensity);
        arenaShader.setFloat(base + "cutOff", light.cutOff);
        arenaShader.setFloat(base + "outerCutOff", light.outerCutOff);
        arenaShader.setFloat(base + "constant", light.constant);
        arenaShader.setFloat(base + "linear", light.linear);
        arenaShader.setFloat(base + "quadratic", light.quadratic);
    }
}

/**
 * Sets up matrices for static model rendering.
 * This configures the shader uniforms for the static environment models
 * like the Ferris wheel and trees.
 */
void GameManager::SetupStaticModelMatrices() {
    staticModelShader.setMat4("projection", projection);
    staticModelShader.setMat4("view", view);
    staticModelShader.setVec3("viewPos", camera.GetPosition());

    // Shadow mapping setup for static models
    staticModelShader.setInt("shadowMap", 2);
    staticModelShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

    // Use main light for static models
    if (!lights.empty()) {
        staticModelShader.setVec3("lightPos", lights[0].position);
        staticModelShader.setVec3("lightColor", lights[0].color);
        staticModelShader.setFloat("lightIntensity", lights[0].intensity);
    }
}

/**
 * Sets up matrices for bumper car rendering.
 * This configures the shader uniforms for the player's car and AI cars.
 * The cars need special lighting and material parameters to look right.
 */
void GameManager::SetupBumperCarMatrices() {
    bumperCarShader.use();
    bumperCarShader.setMat4("projection", projection);
    bumperCarShader.setMat4("view", view);

    // Shadow mapping setup for bumper car
    bumperCarShader.setInt("shadowMap", 2);
    bumperCarShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

    glm::mat4 model = glm::translate(glm::mat4(1.0f), bumperCar.GetPosition());
    model = glm::rotate(model, glm::radians(bumperCar.GetRotation()), glm::vec3(0.0f, 1.0f, 0.0f));
    bumperCarShader.setMat4("model", model);

    bumperCarShader.setVec3("viewPos", camera.GetPosition());
    bumperCarShader.setVec3("lightPos", lights[0].position);
    bumperCarShader.setVec3("lightColor", lights[0].color);
    bumperCarShader.setFloat("lightIntensity", lights[0].intensity);
}

/**
 * Toggles the global lighting system on/off.
 * This is a debugging feature I added to see how the scene looks
 * with and without lighting.
 */
void GameManager::ToggleGlobalLighting() {
    globalLightingEnabled = !globalLightingEnabled;
}