/**
 * ParticleSystem.h
 *
 * My particle system implementation to create visual effects like sparks from the
 * bumper car antennas. I designed this to be reusable and configurable, supporting
 * various effects through parameter adjustments rather than creating separate systems
 * for each effect type.
 */
#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <random>
#include "Shader.h"

 /**
  * Represents a single particle in the system
  *
  * Each particle has its own position, velocity, appearance settings,
  * and lifetime parameters. I track all these properties to create
  * dynamic and realistic particle movement in regards to a subtle electricity effect at the top 
  * of my bumpercar rear pickup.
  */
struct Particle {
    glm::vec3 position;  // Current position in world space
    glm::vec3 velocity;  // Direction and speed of movement
    glm::vec4 color;     // RGBA color including alpha for transparency
    float size;          // Particle size in pixels
    float life;          // Current remaining lifetime in seconds
    float maxLife;       // Maximum lifetime in seconds
};

/**
 * Manages a complete particle effect system
 *
 * This class handles creation, simulation, and rendering of particle effects.
 * I've implemented it to efficiently handle hundreds of particles by using
 * GPU-based point sprites with dynamic updating of only the necessary data.
 */
class ParticleSystem {
public:
    /**
     * Creates a particle system with a specified maximum capacity
     *
     * Initializes the particle system and allocates memory for the specified
     * number of particles. The system doesn't create any active particles yet;
     * they'll be created by calling Emit().
     *
     * maxParticles Maximum number of particles this system can handle at once
     */
    ParticleSystem(unsigned int maxParticles = 100);

    /**
     * Cleans up OpenGL resources
     *
     * Releases the VAO and VBO to prevent memory leaks when the particle
     * system is destroyed.
     */
    ~ParticleSystem();

    /**
     * Updates particle positions and properties
     *
     * I call this every frame to simulate particle physics, update positions,
     * apply forces, update colors based on lifetime, and remove dead particles.
     * The emitter position is used as a reference point, though individual
     * particles move independently once emitted.
     *
     * deltaTime Time since last update in seconds
     * emitterPosition Current world-space position of the particle emitter
     */
    void Update(float deltaTime, const glm::vec3& emitterPosition);

    /**
     * Renders all active particles
     *
     * Draws all particles as point sprites with the provided shader.
     * Uses additive blending for a glowing effect.
     *
     * shader Shader to use for rendering
     * projection Projection matrix
     * view View matrix
     */
    void Render(Shader& shader, const glm::mat4& projection, const glm::mat4& view);

    /**
     * Creates new particles at the specified position
     *
     * I use this to spawn new particles whenever needed, like when the
     * car accelerates to create spark effects from the antenna.
     *
     * position World-space position to emit particles from
     * count Number of new particles to create
     */
    void Emit(const glm::vec3& position, int count);

    /**
     * Enables or disables the particle system
     *
     * When inactive, the system stops updating and rendering particles.
     * This lets me toggle effects on/off without recreating the system.
     *
     * active Whether the system should be active
     */
    void SetActive(bool active) { isActive = active; }

private:
    std::vector<Particle> particles;    // Active particles
    std::vector<float> particleData;    // GPU data buffer
    unsigned int VAO, VBO;              // OpenGL rendering objects
    std::mt19937 rng;                   // Random number generator
    bool isActive;                      // Whether the system is currently active

    /**
     * Sets up OpenGL buffer objects for particle rendering
     *
     * Creates and configures the VAO and VBO with the appropriate
     * vertex attribute pointers for position, color, and size.
     */
    void InitializeBuffers();

    /**
     * Generates a random normalized direction vector
     *
     * I use this to create varied particle trajectories when emitting
     * new particles. The distribution is configured to favor certain
     * directions for more natural-looking effects.
     *
     * return A random normalized direction vector
     */
    glm::vec3 RandomDirection();

    /**
     * Generates a random float in the specified range
     *
     * Helper method for creating random values for particle properties
     * like size, lifetime, and speed.
     *
     * min Minimum value (inclusive)
     * max Maximum value (inclusive)
     * return Random float between min and max
     */
    float RandomFloat(float min, float max);
};