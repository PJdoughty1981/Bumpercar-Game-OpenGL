/**
 * ParticleSystem.cpp
 *
 * Implementation of my particle system for creating dynamic visual effects
 * in the bumper car game. I've focused on making the electric spark effects
 * look convincing while maintaining good performance.
 */
#include "ParticleSystem.h"
#include <algorithm>

 /**
  * Initializes the particle system
  *
  * Sets up the initial capacity, random generator, and OpenGL buffers.
  * The system starts inactive and needs to be explicitly enabled.
  *
  * maxParticles Maximum number of particles this system can handle
  */
ParticleSystem::ParticleSystem(unsigned int maxParticles) : isActive(false) {
    particles.reserve(maxParticles);
    // Each particle needs 8 floats: position(3) + color(4) + size(1)
    particleData.reserve(maxParticles * 8);
    rng.seed(std::random_device()()); // Seed with true random source
    InitializeBuffers();
}

/**
 * Cleans up OpenGL resources
 *
 * Deletes the VAO and VBO to prevent memory leaks.
 */
ParticleSystem::~ParticleSystem() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

/**
 * Sets up the OpenGL buffers for rendering particles
 *
 * I use a single VBO that combines position, color, and size data
 * for each particle. This is more efficient than using separate
 * buffers, especially since all this data changes every frame.
 */
void ParticleSystem::InitializeBuffers() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // Position attribute (3 floats)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);

    // Color attribute (4 floats: RGBA)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));

    // Size attribute (1 float)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(7 * sizeof(float)));
}

/**
 * Updates all active particles
 *
 * This is my main simulation function that:
 * 1. Updates particle positions based on velocity
 * 2. Applies gravitational and acceleration forces
 * 3. Updates colors/alpha based on lifetime
 * 4. Removes dead particles
 * 5. Rebuilds the GPU data buffer with updated values
 *
 * deltaTime Time since last update in seconds
 * emitterPosition Current position of the emitter (used as reference)
 */
void ParticleSystem::Update(float deltaTime, const glm::vec3& emitterPosition) {
    if (!isActive) return;

    // Process each active particle
    auto it = particles.begin();
    while (it != particles.end()) {
        // Decrease lifetime
        it->life -= deltaTime;
        if (it->life <= 0.0f) {
            it = particles.erase(it);
            continue;
        }

        // Update position based on velocity
        it->position += it->velocity * deltaTime;

        // Add upward acceleration for the spark effect
        // I found 3.5 to be a good value for convincing electric sparks
        it->velocity.y += 3.5f * deltaTime;

        // Update color alpha based on life with intensified glow at start
        float lifeRatio = it->life / it->maxLife;
        // Quadratic falloff creates a more intense initial glow that fades quickly
        it->color.a = lifeRatio * lifeRatio;

        ++it;
    }

    // Rebuild the GPU data buffer with updated particle data
    particleData.clear();
    for (const auto& particle : particles) {
        // Position (3 floats)
        particleData.push_back(particle.position.x);
        particleData.push_back(particle.position.y);
        particleData.push_back(particle.position.z);
        // Color (4 floats)
        particleData.push_back(particle.color.r);
        particleData.push_back(particle.color.g);
        particleData.push_back(particle.color.b);
        particleData.push_back(particle.color.a);
        // Size (1 float)
        particleData.push_back(particle.size);
    }

    // Update GPU buffer with new data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, particleData.size() * sizeof(float), particleData.data(), GL_DYNAMIC_DRAW);
}

/**
 * Renders all active particles
 *
 * I render the particles as point sprites using additive blending
 * for a convincing glow effect. Each particle's position, color,
 * and size come from the GPU buffer that was updated in Update().
 *
 * shader Shader to use for rendering
 * projection Current projection matrix
 * view Current view matrix
 */
void ParticleSystem::Render(Shader& shader, const glm::mat4& projection, const glm::mat4& view) {
    if (particles.empty()) return;

    shader.use();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);
    shader.setMat4("model", glm::mat4(1.0f)); // Identity model matrix

    glBindVertexArray(VAO);

    // Set up additive blending for the glow effect
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // Additive blending
    glDepthMask(GL_FALSE); // Don't write to depth buffer (prevents particles from hiding each other)

    // Enable point sprites and set point size
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(15.0f); // Base point size (multiplied by per-particle size)

    // Draw all particles as points
    glDrawArrays(GL_POINTS, 0, particles.size());

    // Restore OpenGL state
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glDisable(GL_PROGRAM_POINT_SIZE);
}

/**
 * Creates new particles at the specified position
 *
 * This is where I actually create the particles with randomized properties
 * to create variation in the effect. I carefully tuned the color selection,
 * velocity range, and lifetime parameters to create convincing electric sparks.
 *
 * position World-space position to emit particles from
 * count Number of new particles to create
 */
void ParticleSystem::Emit(const glm::vec3& position, int count) {
    for (int i = 0; i < count; ++i) {
        // Check if we've reached capacity
        if (particles.size() >= particles.capacity()) break;

        Particle particle;
        particle.position = position;
        // Random direction with random speed between 4 and 7
        particle.velocity = RandomDirection() * RandomFloat(4.0f, 7.0f);

        // Electric spark colors: blue, white, and yellow
        // I distribute them with different probabilities for a more natural look
        float colorChoice = RandomFloat(0.0f, 1.0f);

        if (colorChoice < 0.4f) {
            // Intense electric blue (40% chance)
            particle.color = glm::vec4(0.3f, 0.7f, 1.0f, 1.0f);
        }
        else if (colorChoice < 0.7f) {
            // Bright white with slight blue tint (30% chance)
            particle.color = glm::vec4(0.9f, 0.95f, 1.0f, 1.0f);
        }
        else {
            // Electric yellow (30% chance)
            particle.color = glm::vec4(1.0f, 0.95f, 0.4f, 1.0f);
        }

        // Larger particle sizes for better visibility
        particle.size = RandomFloat(8.0f, 15.0f);

        // Short lifetime for spark-like behavior
        particle.maxLife = RandomFloat(0.08f, 0.2f);
        particle.life = particle.maxLife;

        particles.push_back(particle);
    }
}

/**
 * Generates a random direction vector with an upward bias
 *
 * I've designed this to create a pattern that resembles electric sparks
 * flying upward and outward from the source. The distribution favors
 * upward directions while still allowing for some randomness.
 *
 * return Normalized random direction vector
 */
glm::vec3 ParticleSystem::RandomDirection() {
    std::uniform_real_distribution<float> dist(-1.0f, 0.5f);
    // Create a direction with stronger upward bias and wider spread
    glm::vec3 dir(
        dist(rng) * 0.8f,            // Wider X spread
        std::abs(dist(rng)) * 2.0f,  // Stronger upward bias
        dist(rng) * 0.8f             // Wider Z spread
    );
    return glm::normalize(dir);
}

/**
 * Generates a random float in the specified range
 *
 * Simple utility function to get a random float between min and max.
 *
 * min Minimum value (inclusive)
 * max Maximum value (inclusive)
 * return Random float between min and max
 */
float ParticleSystem::RandomFloat(float min, float max) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}