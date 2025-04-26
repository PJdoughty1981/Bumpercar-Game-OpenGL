# UEA Bumper Cars

A 3D bumper car game with physics simulation, AI opponents, multiple camera modes, and visual effects.

## Project Structure

```
BumperCarRide/
|
├── Resources/
│   ├── models/             # 3D model files (.obj, .mtl)
│   │   ├── bumper_car.obj  # Player car model
│   │   ├── ferriswheel/    # Environment models
│   │   └── tree/           # Environment models
│   ├── textures/           # Texture files
│   │   ├── arena/          # Arena textures (floor, walls)
│   │   ├── environment/    # Environment textures (grass, skybox)
│   │   └── bumper_car/     # Car textures
│   └── font/               # Font files for text rendering
│
├── Shaders/                # GLSL shader files
│   ├── arena.vert          # Arena vertex shader
│   ├── arena.frag          # Arena fragment shader with shadow mapping
│   ├── model_loading.vert  # Bumper car vertex shader
│   ├── model_loading.frag  # Bumper car fragment shader
│   ├── particle.vert       # Particle system vertex shader
│   ├── particle.frag       # Particle system fragment shader
│   ├── shadow_mapping.vert # Shadow depth map generation
│   ├── static_model.vert   # Environment objects vertex shader
│   ├── static_model.frag   # Environment objects fragment shader
│   ├── text.vert           # Text rendering vertex shader
│   ├── text.frag           # Text rendering fragment shader
│   ├── debug.vert          # Debug visualization vertex shader
│   ├── debug.frag          # Debug visualization fragment shader
│   ├── headlight_source.vert # Headlight glow vertex shader
│   ├── headlight_source.frag # Headlight glow fragment shader
│   ├── skybox.vert         # Skybox vertex shader
│   └── skybox.frag         # Skybox fragment shader
│
├── Dependencies/           # External libraries
│   ├── GLFW/               # Window management
│   ├── GLAD/               # OpenGL function loading
│   ├── GLM/                # Mathematics library
│   └── stb/                # Image loading
│   
├── Headers:
│   ├── AIController.h      # AI for computer-controlled cars
│   ├── Arena.h             # The game environment
│   ├── BumperCar.h         # Physics and rendering for cars
│   ├── Camera.h            # Multiple camera modes
│   ├── GameManager.h       # Core game logic and systems
│   ├── ParticleSystem.h    # Visual effects system
│   ├── Shader.h            # GLSL shader management
│   └── StaticModel.h       # Environment object rendering
│
├── Source Files:
│   ├── AIController.cpp    # AI implementation
│   ├── Arena.cpp           # Arena implementation
│   ├── BumperCar.cpp       # Bumper car implementation
│   ├── Camera.cpp          # Camera system implementation
│   ├── GameManager.cpp     # Game manager implementation
│   ├── ParticleSystem.cpp  # Particle system implementation
│   ├── StaticModel.cpp     # Static model implementation
│   ├── main.cpp            # Entry point and initialization
│   ├── glad.c              # OpenGL function loader
│   └── stb_impl.cpp        # STB image implementation
```

## Features

- **Physics Simulation**: Realistic momentum-based physics with elastic collisions
- **AI Opponents**: Computer-controlled cars with state-based behavior
- **Multiple Camera Modes**:
  - Free-flying camera for overview
  - First-person driver view
  - Third-person follow camera
  - Ground-level walking camera
- **Visual Effects**:
  - Dynamic lighting with shadows
  - Particle effects
  - Car headlights
  - Impact shake effects
- **Environment**:
  - Textured arena with walls
  - Skybox background
  - Static environment objects (Ferris wheel, trees)
- **User Interface**:
  - On-screen text rendering
  - Key bindings menu
  - Debug visualization

## Controls

### Game Control
- `ESC`: Exit the game
- `TAB`: Toggle key bindings menu
- `C`: Toggle collision box visualization
- `P`: Toggle particle effects

### Camera Controls
- `1`: Free Camera Mode
  - `W`: Move forward
  - `S`: Move backward
  - `A`: Move left
  - `D`: Move right
  - `Z`: Move up
  - `X`: Move down
  - Mouse: Look around
  - Scroll: Zoom in/out

- `2`: First-Person Camera Mode
- `3`: Third-Person Camera Mode
- `4`: Ground Camera Mode

### Player Car Controls
- `UP ARROW`: Accelerate forward
- `DOWN ARROW`: Brake/Reverse
- `LEFT ARROW`: Turn left
- `RIGHT ARROW`: Turn right

## Technical Implementation

### Physics System

The physics system simulates realistic bumper car behavior with:
- Momentum-based movement
- Semi-elastic collisions
- Friction and angular damping
- Impact response with bounce and spin

### Rendering Pipeline

The rendering pipeline includes:
1. Shadow mapping pass for depth from light perspective
2. Main rendering pass for scene objects
3. Skybox rendering
4. Effects rendering (particles, headlights)
5. Debug visualization (when enabled)
6. UI text rendering

### AI System

The AI system features state-based behavior with four states:
- **Cruising**: Normal driving around the arena
- **Targeting**: Actively pursuing another car
- **Evading**: Avoiding imminent collisions
- **Recovering**: Escaping from being stuck

### Memory Management

The game uses modern C++ memory management:
- `std::unique_ptr` for owned resources
- RAII pattern for resource management
- Proper OpenGL resource cleanup

## Building the Project

### Prerequisites
- C++11 compatible compiler
- OpenGL 3.3+ compatible GPU
- CMake 3.10 or higher

## Credits

- Graphics: OpenGL, GLFW, GLAD, STB Image
- Mathematics: GLM (OpenGL Mathematics)
- Text Rendering: FreeType
- 3D Models: Custom-created models
- Textures: Custom textures and Ambient CG materials