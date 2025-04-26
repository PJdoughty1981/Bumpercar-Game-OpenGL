/**
 * main.cpp - Entry point for my Bumper Cars application
 *
 * This file handles application initialization, creating the main window,
 * setting up OpenGL, and running the main game loop. I've structured it
 * to separate the core application/window logic from the actual game logic,
 * which is handled by the GameManager class.
 */

 //-----------------------------------------------------------------------------
 // Includes
 //-----------------------------------------------------------------------------
 // System Headers
#include <glad/glad.h>
#include <GLFW/include/glfw3.h>

// Project Headers
#include "GameManager.h"
#include <memory>

// Standard Library
#include <iostream>

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
const char* WINDOW_TITLE = "UEA Bumper Cars";
// Default game resolution
const int GAME_WIDTH = 1920;
const int GAME_HEIGHT = 1080;

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
/**
 * Initializes GLFW library
 *
 * Sets up GLFW with OpenGL 3.3 core profile. This is the first step
 * in my graphics initialization pipeline.
 *
 * return True if initialization succeeded, false otherwise
 */
bool InitializeGLFW();

/**
 * Creates the main game window
 *
 * Creates a fullscreen window using the primary monitor's resolution.
 * The window is set as the current OpenGL context.
 *
 * window Reference to store the created window
 * return True if window creation succeeded, false otherwise
 */
bool CreateWindow(GLFWwindow*& window);

/**
 * Initializes GLAD for OpenGL function loading
 *
 * GLAD is used to load OpenGL function pointers, which is necessary
 * before I can use any OpenGL functions.
 *
 * return True if initialization succeeded, false otherwise
 */
bool InitializeGLAD();

/**
 * Sets up input callbacks
 *
 * Connects input events (mouse movement, scrolling) to the GameManager's
 * handler methods. This gives the game manager control over input.
 *
 * window Window to set callbacks for
 * gameManager Game manager instance to receive callbacks
 */
void SetupCallbacks(GLFWwindow* window, GameManager& gameManager);

/**
 * Updates the OpenGL viewport to match window size
 *
 * This is called when the window is resized to ensure rendering
 * uses the correct dimensions.
 *
 * window Window to get size from
 */
void UpdateViewport(GLFWwindow* window);

//-----------------------------------------------------------------------------
// Main Entry Point
//-----------------------------------------------------------------------------
/**
 * Main entry point of the application
 *
 * Initializes all systems, creates the game manager, runs the main game loop,
 * and cleans up resources when done.
 *
 * return 0 if successful, -1 on error
 */
int main() {
    // Initialize GLFW
    if (!InitializeGLFW()) {
        return -1;
    }

    // Create Window
    GLFWwindow* window = nullptr;
    if (!CreateWindow(window)) {
        glfwTerminate();
        return -1;
    }

    // Initialize GLAD
    if (!InitializeGLAD()) {
        glfwTerminate();
        return -1;
    }

    // Create and initialize game manager
    GameManager gameManager;
    gameManager.Initialize(window);  // Pass window handle to Initialize

    // Setup input callbacks
    SetupCallbacks(window, gameManager);

    // Initial viewport setup
    UpdateViewport(window);

    //-----------------------------------------------------------------------------
    // Main Game Loop
    //-----------------------------------------------------------------------------
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window)) {
        // Time management
        float currentFrame = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check for ESC key to close the window
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, true);
        }

        // Update game state
        gameManager.HandleInput(window);
        gameManager.Update(deltaTime);
        gameManager.Render();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    //-----------------------------------------------------------------------------
    // Cleanup and Exit
    //-----------------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

//-----------------------------------------------------------------------------
// GLFW Initialization Implementation
//-----------------------------------------------------------------------------
/**
 * Initializes the GLFW library
 *
 * Sets up GLFW with OpenGL 3.3 core profile for modern OpenGL features.
 *
 * return True if initialization succeeded, false otherwise
 */
bool InitializeGLFW() {
    if (!glfwInit()) {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // I'm targeting OpenGL 3.3 core profile for my project
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    return true;
}

//-----------------------------------------------------------------------------
// Viewport Management Implementation
//-----------------------------------------------------------------------------
/**
 * Updates the OpenGL viewport to match window size
 *
 * Gets the current framebuffer size and sets the OpenGL viewport accordingly.
 * This ensures correct rendering when the window is resized.
 *
 * window Window to get size from
 */
void UpdateViewport(GLFWwindow* window) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
}

//-----------------------------------------------------------------------------
// Window Creation Implementation
//-----------------------------------------------------------------------------
/**
 * Creates the main game window
 *
 * Creates a fullscreen window using the primary monitor's resolution.
 * I chose fullscreen for an immersive experience and to avoid window
 * management issues.
 *
 * window Reference to store the created window
 * return True if window creation succeeded, false otherwise
 */
bool CreateWindow(GLFWwindow*& window) {
    // Get primary monitor
    GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
    if (!primaryMonitor) {
        std::cout << "Failed to get primary monitor" << std::endl;
        return false;
    }

    // Get monitor video mode
    const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);
    if (!mode) {
        std::cout << "Failed to get video mode" << std::endl;
        return false;
    }

    // Create fullscreen window
    window = glfwCreateWindow(mode->width, mode->height, WINDOW_TITLE, primaryMonitor, NULL);
    if (!window) {
        std::cout << "Failed to create GLFW window" << std::endl;
        return false;
    }

    glfwMakeContextCurrent(window);

    // Add window resize callback
    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* w, int width, int height) {
        UpdateViewport(w);
        });

    return true;
}

//-----------------------------------------------------------------------------
// GLAD Initialization Implementation
//-----------------------------------------------------------------------------
/**
 * Initializes GLAD for OpenGL function loading
 *
 * GLAD loads all OpenGL function pointers so I can use them in my code.
 * This must be done after creating an OpenGL context but before using
 * any OpenGL functions.
 *
 * return True if initialization succeeded, false otherwise
 */
bool InitializeGLAD() {
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return false;
    }
    return true;

}

//-----------------------------------------------------------------------------
// Callback Setup Implementation
//-----------------------------------------------------------------------------
/**
 * Sets up input callbacks for the window
 *
 * Links mouse movement and scrolling events to the GameManager's
 * handler methods. Using lambdas makes it easy to forward events
 * to class member functions.
 *
 * window Window to set callbacks for
 * gameManager Game manager instance to receive callbacks
 */
void SetupCallbacks(GLFWwindow* window, GameManager& gameManager) {
    // Store a pointer to the game manager in the window's user pointer
    glfwSetWindowUserPointer(window, &gameManager);

    // Mouse movement callback
    glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
        static_cast<GameManager*>(glfwGetWindowUserPointer(w))->MouseCallback(w, x, y);
        });

    // Mouse scroll callback
    glfwSetScrollCallback(window, [](GLFWwindow* w, double x, double y) {
        static_cast<GameManager*>(glfwGetWindowUserPointer(w))->ScrollCallback(w, x, y);
        });
}