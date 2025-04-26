/**
 * Shader.h - My custom shader management class
 *
 * This is a minimal but effective shader implementation I've created to handle
 * all the GLSL shaders in my bumper car project. I wanted to keep it simple while
 * still providing all the functionality I need for material rendering, lighting,
 * and special effects.
 */
#pragma once
#include <fstream>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <iostream>
#include <sstream>
#include <string>

class Shader {
public:
    unsigned int ID; // Program ID used by OpenGL

    /**
     * Creates and compiles a shader program from vertex and fragment shader files
     *
     * This constructor handles all the boilerplate OpenGL code for me. It loads shader
     * source code from the provided files, compiles them, links the program, and
     * stores the resulting program ID for later use. I use this for all shader creation
     * throughout the project.
     *
     * vertexPath Path to the vertex shader file
     * fragmentPath Path to the fragment shader file
     */
    Shader(const char* vertexPath, const char* fragmentPath) {
        // Read files and compile shaders
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;

        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        std::stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        vShaderFile.close();
        fShaderFile.close();
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();

        unsigned int vertex, fragment;
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);

        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);

        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    /**
     * Activates this shader program for rendering
     *
     * I call this before rendering any object that should use this shader.
     * It tells OpenGL which shader program to use for subsequent draw calls.
     */
    void use() { glUseProgram(ID); }

    /**
     * Sets an integer uniform value in the shader
     *
     * I use this primarily for texture units and boolean flags.
     *
     * name Name of the uniform in the shader
     * value Integer value to set
     */
    void setInt(const std::string& name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }

    /**
     * Sets a float uniform value in the shader
     *
     * I use this for material properties, time values, and other scalar values.
     *
     * name Name of the uniform in the shader
     * value Float value to set
     */
    void setFloat(const std::string& name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }

    /**
     * Sets a vec3 uniform value in the shader (vector version)
     *
     * I use this for colors, positions, and directions.
     *
     * name Name of the uniform in the shader
     * value Vector value to set
     */
    void setVec3(const std::string& name, const glm::vec3& value) const {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }

    /**
     * Sets a vec3 uniform value in the shader (components version)
     *
     * This overload lets me specify the components directly. It's useful when I
     * need to set a vector from individual values rather than a glm::vec3.
     *
     * name Name of the uniform in the shader
     * x X component
     * y Y component
     * z Z component
     */
    void setVec3(const std::string& name, float x, float y, float z) const {
        glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }

    /**
     * Sets a mat4 uniform value in the shader
     *
     * I use this for transformation matrices like model, view, projection,
     * and the light space matrix for shadow mapping.
     *
     * name Name of the uniform in the shader
     * mat Matrix value to set
     */
    void setMat4(const std::string& name, const glm::mat4& mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
};
