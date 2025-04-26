/**
 * GameManager.h
 *
 * This is the heart of my bumper car game - the central orchestrator that manages
 * all the game systems and ties everything together. I designed it to be the brain
 * of the application that coordinates physics, rendering, AI, input, and everything
 * else that makes the game work.
 *
 * I've put a lot of effort into making this class comprehensive but still maintainable.
 * It's responsible for:
 * - Managing the game state and main loop
 * - Running the physics simulation and collision detection
 * - Controlling multiple camera views and letting players switch between them
 * - Implementing a dynamic lighting system with realistic shadows
 * - Creating particle effects for visual feedback
 * - Providing debug visualization tools that helped me fix issues
 * - Rendering text and UI elements
 * - Controlling AI behavior for opponent cars
 * - Rendering the skybox and environment
 *
 * This manager gets initialized in main.cpp and then takes care of everything else.
 */
#pragma once

 //-----------------------------------------------------------------------------
 // Includes
 //-----------------------------------------------------------------------------
 // System Headers
#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <GLFW/include/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <stb\stb_image.h>

// Project Headers
#include "Arena.h"
#include "Camera.h"
#include "Shader.h"
#include "BumperCar.h"
#include "ParticleSystem.h"
#include "AIController.h"
#include "StaticModel.h"

// Standard Library
#include <vector>
#include <memory>

// FreeType for text rendering
#include <ft2build.h>
#include FT_FREETYPE_H
#include <map>

//-----------------------------------------------------------------------------
// Helper Structures
//-----------------------------------------------------------------------------
/**
 * Light configuration structure
 *
 * I use this to store the position and color for different lights in the scene.
 * It makes it easy to initialize groups of lights with different properties.
 */
struct LightConfig {
    glm::vec3 position;  // Where the light is in the world
    glm::vec3 color;     // The light's RGB color
};

/**
 * Character glyph data for text rendering
 *
 * This stores all the data needed to render a single character with FreeType.
 * Working with font rendering was tricky, but storing all this data for each
 * character makes it much easier to position and render text properly.
 */
struct Character {
    unsigned int TextureID;  // Glyph texture
    glm::ivec2   Size;       // Size of the glyph
    glm::ivec2   Bearing;    // Offset from baseline to left/top of glyph
    unsigned int Advance;    // Horizontal offset to advance to next character
};

//-----------------------------------------------------------------------------
// Sign Board Structure
//-----------------------------------------------------------------------------
struct SignBoard {
    GLuint VAO, VBO, EBO;                // OpenGL buffers
    unsigned int texture;                // Sign texture
    glm::vec3 position;                  // Position in world space  
    glm::vec3 scale;                     // Scale factors
    float rotation;                      // Rotation angle in degrees
};

//-----------------------------------------------------------------------------
// GameManager Class Declaration
//-----------------------------------------------------------------------------
class GameManager {
private:
    //-----------------------------------------------------------------------------
    // Light Structures
    //-----------------------------------------------------------------------------
    /**
     * Regular point light
     *
     * I use these for general illumination around the arena. They can be static
     * or have flashing effects to create a fun fairground atmosphere.
     */
    struct Light {
        glm::vec3 position;    // Where the light is
        glm::vec3 color;       // Light color (RGB)
        float intensity;       // How bright the light is
        bool isFlashing;       // Whether the light flashes
        float flashSpeed;      // How fast it flashes
        float flashTimer;      // Current flash time
    };

    /**
     * Spotlight for car headlights
     *
     * These are more focused than regular lights and only illuminate in a cone.
     * I use these for the car headlights to create realistic forward lighting
     * Simply to show I could do it.
     */
    struct SpotLight {
        glm::vec3 position;    // Where the light is
        glm::vec3 direction;   // Which way it's pointing
        glm::vec3 color;       // Light color (RGB)
        float intensity;       // Brightness multiplier
        float cutOff;          // Inner cone angle (cosine)
        float outerCutOff;     // Outer cone angle (cosine)

        // Distance falloff factors
        float constant;        // Constant term in attenuation formula
        float linear;          // Linear distance falloff
        float quadratic;       // Quadratic distance falloff
    };

    //-----------------------------------------------------------------------------
    // Visualization Structures
    //-----------------------------------------------------------------------------
    /**
     * Visual element for headlight source
     *
     * These are the small glowing spheres that represent the actual headlight bulbs.
     * They don't produce light themselves (that's handled by the SpotLight), but they
     * give a nice visual cue that the headlights are there.
     */
    struct HeadlightSource {
        GLuint VAO, VBO, EBO;                // OpenGL buffers
        std::vector<glm::vec3> vertices;     // Vertex positions
        std::vector<glm::vec3> normals;      // Vertex normals
        std::vector<unsigned int> indices;   // Element indices
    };

    /**
     * Debug visualization vertex
     *
     * I use these for drawing debug lines and shapes to visualize things like
     * collision boxes, turning centers, and other invisible game elements.
     */
    struct DebugVertex {
        glm::vec3 position;  // Vertex position
        glm::vec4 color;     // Vertex color (RGBA)
    };

    //-----------------------------------------------------------------------------
    // Core Components
    //-----------------------------------------------------------------------------
    Camera camera;                       // Camera system for different viewpoints
    Arena arena;                         // The game environment
    BumperCar bumperCar;                // Player's car
    GLFWwindow* window;                 // Reference to the main window
    SignBoard signBoard;                // sign logo

    //-----------------------------------------------------------------------------
    // Shader Components
    //-----------------------------------------------------------------------------
    Shader arenaShader;                 // For rendering the arena
    Shader bumperCarShader;             // For rendering cars
    Shader shadowShader;                // For shadow mapping
    std::unique_ptr<Shader> headlightSourceShader; // For rendering headlight glows
    Shader staticModelShader;           // For environment objects

    //-----------------------------------------------------------------------------
    // Debug Visualization
    //-----------------------------------------------------------------------------
    std::vector<DebugVertex> debugVertices;  // Vertices for debug lines
    GLuint debugVAO, debugVBO;         // OpenGL buffers for debug rendering
    std::unique_ptr<Shader> debugShader;// Shader for debug visualization

    //-----------------------------------------------------------------------------
    // Matrices
    //-----------------------------------------------------------------------------
    glm::mat4 projection;               // Projection matrix (perspective)
    glm::mat4 view;                     // View matrix (camera)

    //-----------------------------------------------------------------------------
    // Lighting System
    //-----------------------------------------------------------------------------
    std::vector<Light> lights;          // Regular point lights
    std::vector<SpotLight> headlights;  // Car headlights
    HeadlightSource headlightSources[2];// Visual representation of headlights
    bool globalLightingEnabled;         // Master switch for lighting

    //-----------------------------------------------------------------------------
    // Particle System Components
    //-----------------------------------------------------------------------------
    std::unique_ptr<ParticleSystem> antennaParticles;
    std::unique_ptr<Shader> particleShader;
    bool particleEffectsEnabled;

    //-----------------------------------------------------------------------------
    // Freetype text
    //-----------------------------------------------------------------------------
    std::map<GLchar, Character> Characters;
    GLuint textVAO, textVBO;
    std::unique_ptr<Shader> textShader;

    //-----------------------------------------------------------------------------
    // Shadow Mapping
    //-----------------------------------------------------------------------------
    unsigned int depthMapFBO;           // Framebuffer for shadow mapping
    unsigned int depthMap;              // Shadow map texture
    glm::mat4 lightSpaceMatrix;         // Transform to light space
    static const unsigned int SHADOW_WIDTH = 1024;   // Shadow map resolution
    static const unsigned int SHADOW_HEIGHT = 1024;

    //-----------------------------------------------------------------------------
    // Input State
    //-----------------------------------------------------------------------------
    float lastX, lastY;                 // Last mouse position
    bool firstMouse;                    // First mouse input flag
    float deltaTime;                    // Time between frames
    bool keysProcessed[348];            // Keep track of which keys were already processed
    float totalTime;                    // Total time the game has been running

    // Menu related
    bool showKeyBindingsMenu;           // Whether to show controls menu
    bool showCollisions;                // Whether to show collision boxes

    //-----------------------------------------------------------------------------
    // AI Cars
    //-----------------------------------------------------------------------------
    static const int NUM_AI_CARS = 4;
    std::vector<std::unique_ptr<BumperCar>> aiCars;
    std::vector<std::unique_ptr<AIController>> aiControllers;
    bool gameStarted;
    static const std::vector<CarColorScheme> COLOR_SCHEMES;

    //-----------------------------------------------------------------------------
    // Skybox
    //-----------------------------------------------------------------------------
    GLuint skyboxVAO, skyboxVBO;
    GLuint cubemapTexture;
    std::unique_ptr<Shader> skyboxShader;

    /**
     * Loads a cubemap from 6 image files
     *
     * Getting a nice-looking skybox was important for the atmosphere of the game.
     * I spent some time finding good images and tuning the parameters to make
     * it look seamless.
     *
     * faces Vector with paths to the 6 face images
     * return OpenGL texture ID for the cubemap
     */
    unsigned int LoadCubemap(const std::vector<std::string>& faces);

    /**
     * Sets up the skybox rendering
     *
     * The skybox is just a cube with textures on the inside, but there are
     * some tricks needed to make it work correctly (like special depth handling).
     */
    void InitializeSkybox();

    /**
     * Renders the skybox
     *
     * I render the skybox last (except for UI elements) and use a special
     * depth function to make sure it appears behind everything else.
     */
    void RenderSkybox();

    //-----------------------------------------------------------------------------
    // Environment static models
    //-----------------------------------------------------------------------------
    std::vector<std::unique_ptr<StaticModel>> staticModels; 
    
    /**
     * Sets up matrices for static models
     *
     * The environment models (Ferris wheel, trees) need their own matrix
     * setup because they have different properties than the cars or arena.
     */
    void SetupStaticModelMatrices();

    /**
     * Sets up Sign\Board
     */
    void InitializeSignBoard();
    void RenderSignBoard();

public:
    //-----------------------------------------------------------------------------
    // Public Interface
    //-----------------------------------------------------------------------------
    /**
     * Constructor for the game manager
     *
     * This sets up the initial state of all game systems. There's quite a bit
     * to initialize, but I organize it so each system can be set up independently.
     */
    GameManager();

    /**
     * Destructor cleans up resources
     *
     * OpenGL requires explicit cleanup of resources, so I make sure
     * to delete all the buffers, textures, etc. I've created.
     */
    ~GameManager();

    /**
     * Initializes the game
     *
     * This is the main setup function that gets called once at startup.
     * It's like the "opening ceremony" where everything gets ready for the game.
     *
     * windowHandle Handle to the GLFW window
     */
    void Initialize(GLFWwindow* windowHandle);

    /**
     * Updates game state
     *
     * This is the heartbeat of the game, called every frame to update
     * physics, AI, effects, etc. Everything that changes over time
     * gets updated here.
     *
     * dt = deltatime, Time since last update in seconds
     */
    void Update(float dt);

    /**
     * Renders the game
     *
     * Draws all game elements on screen. I use a multi-pass approach:
     * 1. Shadow pass (for shadow mapping)
     * 2. Main scene rendering
     * 3. Effects, UI, debug visualization
     */
    void Render();

    /**
     * Processes all input
     *
     * This is the main input handler that routes keyboard, mouse, and other
     * inputs to the appropriate systems (car controls, camera, menus, etc.)
     *
     * window GLFW window for input queries
     */
    void HandleInput(GLFWwindow* window);

    /**
     * Processes car input
     *
     * Handles the player car controls - acceleration, braking, and turning.
     * I decided to use arrow keys for simplicity and intuitiveness.
     *
     * deltaTime Time since last update
     */
    void HandleCarInput(float deltaTime);

    /**
     * Processes camera input
     *
     * Handles movement for the free and ground camera modes.
     * WASD for movement, Z/X for up/down.
     *
     * deltaTime Time since last update
     */
    void HandleCameraInput(float deltaTime);

    /**
     * Processes camera mode switching
     *
     * I added multiple camera modes to give different perspectives
     * on the action as the brief specified. 
     * This handles the key inputs to switch between them.
     */
    void HandleCameraModeInput();

    /**
     * Handles mouse movement
     *
     * Callback for mouse movement, used for rotating the camera.
     * Getting smooth camera control took some tweaking of sensitivity values.
     *
     * window GLFW window
     * xpos Mouse X position
     * ypos Mouse Y position
     */
    void MouseCallback(GLFWwindow* window, double xpos, double ypos);

    /**
     * Handles mouse scroll
     *
     * Callback for scroll wheel, used for camera zoom in free mode.
     *
     * window GLFW window
     * xoffset Horizontal scroll (unused)
     * yoffset Vertical scroll amount
     */
    void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    /**
     * Processes escape key
     *
     * Checks for escape key to exit the game. I made this its own function
     * since it's the only input that can terminate the game.
     *
     * window GLFW window
     */
    void HandleEscapeKey(GLFWwindow* window);

    /**
     * Changes camera mode
     *
     * Switches between different camera perspectives. The camera system
     * is one of my favorite parts of the game - it gives such different
     * experiences depending on which view you choose.
     *
     * mode Camera mode to switch to
     */
    void SwitchCamera(Camera::Mode mode);

    /**
     * Updates camera position
     *
     * Updates the camera based on the current mode and target positions.
     * Each mode has different behavior - first-person follows from driver's view,
     * third-person trails behind, etc.
     */
    void UpdateCameraPosition();

    /**
     * Toggles lighting on/off
     *
     * Enables or disables all lighting. This was mainly for debugging
     * to see how things look without lighting effects.
     */
    void ToggleGlobalLighting();

    /**
     * Toggles particle effects on/off
     *
     * Turns the electric spark effects on/off. I added this toggle
     * because particles can be a bit performance-heavy on lower-end systems.
     */
    void ToggleParticleEffects();

    /**
     * Toggles key bindings menu
     *
     * Shows or hides the controls information screen. I added this
     * after realizing it was hard for new players to remember all the keys.
     * It was also the best way to easily show the integration of freetype.
     */
    void ToggleKeyBindingsMenu();

    /**
     * Checks for colisions between cars
     *
     * This is the main collision detection system for car-to-car collisions.
     * Getting this right was one of the toughest challenges in the physics system.
     */
    void CheckCarCollisions();

    /**
     * Checks for collision between two boxes
     *
     * Tests if two oriented bounding boxes are intersecting. I use this
     * for car-to-car collision detection.
     *
     * center1 Center of first box
     * rot1 Rotation of first box
     * center2 Center of second box
     * rot2 Rotation of second box
     * return True if boxes are colliding
     */
    bool CheckBoxCollision(const glm::vec3& center1, float rot1,
        const glm::vec3& center2, float rot2);

    /**
     * Gets corners of a rotated box
     *
     * Calculates the 2D corners of a car's collision box in world space.
     *
     * center Box center
     * rotation Box rotation
     * halfWidth Half width of the box
     * halfLength Half length of the box
     * return Vector of 2D corner points
     */
    std::vector<glm::vec2> GetBoxCorners(const glm::vec3& center, float rotation,
        float halfWidth, float halfLength);

    /**
     * Checks for separating axis
     *
     * Part of the Separating Axis Theorem collision detection algorithm.
     * If any axis separates the boxes, they don't collide.
     *
     * corners1 Corners of first box
     * corners2 Corners of second box
     * return True if a separating axis exists (no collision)
     */
    bool HasSeparatingAxis(const std::vector<glm::vec2>& corners1,
        const std::vector<glm::vec2>& corners2);

private:
    //-----------------------------------------------------------------------------
    // Private Methods
    //-----------------------------------------------------------------------------
    /**
     * Sets up all shaders
     *
     * Creates and configures the various shader programs I use for different
     * rendering tasks - arena, cars, shadows, effects, etc.
     */
    void InitializeShaders();

    /**
     * Sets up the lighting system
     *
     * Creates lights and configures their properties. I use a mix of
     * regular point lights and spotlights to create an interesting atmosphere.
     */
    void InitializeLighting();

    /**
     * Sets up shadow mapping
     *
     * Creates framebuffers and textures for shadow mapping. Getting
     * good-looking shadows was tricky and took a lot of tweaking.
     */
    void InitializeShadowMapping();

    /**
     * Sets up debug visualization
     *
     * Creates buffers and shaders for drawing debug lines and shapes.
     * This was incredibly useful during development to see collision boxes,
     * rotation centers, etc.
     */
    void InitializeDebugVisualization();

    /**
     * Sets up headlight sources
     *
     * Creates the small glowing spheres that represent headlight bulbs.
     * These don't actually produce light, but they look good as visual elements.
     */
    void InitializeHeadlightSources();

    /**
     * Sets up environment models
     *
     * Loads and positions static models like the Ferris wheel and trees.
     * These add a lot to the atmosphere and make the arena feel like
     * part of a larger fairground.
     */
    void InitializeStaticModels();

    /**
     * Renders main scene
     *
     * Draws the arena, cars, and static models with proper lighting and shadows.
     * This is the main rendering pass that creates most of what you see.
     */
    void RenderScene();

    /**
     * Renders shadow depth map
     *
     * First pass for shadow mapping to capture depths from light's perspective.
     * This generates the data needed to determine which areas are in shadow.
     */
    void RenderShadowPass();

    /**
     * Renders headlight glows
     *
     * Draws the glowing spheres for car headlights. I use additive blending
     * to create a nice bright glow effect.
     */
    void RenderHeadlightSources();

    /**
     * Renders scene to depth map
     *
     * Draws the scene from the light's perspective for shadow mapping.
     * Similar to normal rendering but only outputs depth values.
     */
    void RenderSceneToDepthMap();

    /**
     * Renders collision visualization
     *
     * Draws debug lines for collision boundaries when enabled with 'C' key.
     * This was originally just for debugging, but I kept it in because
     * it's actually pretty cool to see the physics in action.
     */
    void RenderDebugCollisions();

    /**
     * Renders key bindings menu
     *
     * Draws the controls information screen. This helps new players
     * learn the controls without having to read a manual.
     */
    void RenderKeyBindingsMenu();

    /**
     * Updates lighting parameters
     *
     * Updates light positions, colors, and intensities each frame.
     * Some lights have dynamic effects like flashing.
     */
    void UpdateLighting();

    /**
     * Updates headlight positions
     *
     * Updates the car headlight positions and directions based on the
     * car's current position and orientation.
     */
    void UpdateHeadlightPositions();

    /**
     * Sets lighting uniforms
     *
     * Configures all lighting parameters in the shader programs.
     * This transfers the light properties to the GPU for rendering.
     */
    void SetupLightingUniforms();

    /**
     * Creates a sphere mesh
     *
     * Generates a sphere mesh for visual representation of light sources.
     * I use these for the headlight bulbs.
     *
     * source Headlight source to populate
     * radius Sphere radius
     * segments Horizontal segments
     * rings Vertical rings
     */
    void GenerateSphereMesh(HeadlightSource& source, float radius, int segments, int rings);

    /**
     * Sets up headlight buffers
     *
     * Creates and configures OpenGL buffers for headlight source meshes.
     *
     * source Headlight source to configure
     */
    void SetupHeadlightBuffers(HeadlightSource& source);

    /**
     * Sets up shader matrices
     *
     * Configures model, view, and projection matrices in shaders.
     * These control how objects are positioned and viewed in 3D space.
     */
    void SetupMatrices();

    /**
     * Sets up car matrices
     *
     * Configures matrices specifically for car rendering.
     * Cars need special handling because they move and rotate.
     */
    void SetupBumperCarMatrices();

    /**
     * Updates projection matrix
     *
     * Recalculates the projection matrix based on window size and camera zoom.
     * This should be called whenever the window is resized or zoom changes.
     */
    void UpdateProjectionMatrix();

    /**
     * Updates view matrix
     *
     * Recalculates view matrix based on camera position and orientation.
     * This transforms world-space coordinates to camera-relative coordinates.
     */
    void UpdateViewMatrix();

    /**
     * Updates debug vertices
     *
     * Rebuilds the list of vertices for debug visualization each frame.
     * This includes collision boxes, turning centers, and other elements.
     */
    void UpdateDebugVertices();

    /**
     * Adds a debug line
     *
     * Adds a red line to the debug visualization. Red is the default
     * color for most debug elements.
     *
     * start Line start position
     * end Line end position
     */
    void AddDebugLine(const glm::vec3& start, const glm::vec3& end);

    /**
     * Adds a colored debug line
     *
     * Adds a line with custom color to the debug visualization.
     * I use different colors to distinguish different types of elements.
     *
     * start Line start position
     * end Line end position
     * color Line color (RGBA)
     */
    void AddDebugLine(const glm::vec3& start, const glm::vec3& end, const glm::vec4& color);

    /**
     * Initializes FreeType text rendering
     *
     * Sets up FreeType and loads the font. Text rendering was a bit complex
     * to set up but really improves the UI.
     */
    void InitializeFreetype();

    /**
     * Renders text on screen
     *
     * Draws a string of text at the specified position and size.
     * Each character is rendered as a textured quad.
     *
     * text The string to render
     * x X-position on screen
     * y Y-position on screen
     * scale Size scale factor
     * color Text color (RGB)
     */
    void RenderText(const std::string& text, float x, float y, float scale, glm::vec3 color);

    /**
     * Sets up text rendering
     *
     * Creates and configures OpenGL buffers for text rendering.
     * This needed to be done after FreeType initialization.
     */
    void SetupTextRendering();
};