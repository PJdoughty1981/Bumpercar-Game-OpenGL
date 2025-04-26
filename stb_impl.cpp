// stb_impl.cpp
/**
 * This file implements the STB Image library.
 *
 * I'm using this single-header library to handle texture loading throughout
 * the project. By defining STB_IMAGE_IMPLEMENTATION in just one file,
 * I avoid multiple definition errors while keeping the actual implementation
 * in my codebase.
 *
 * STB Image gives me PNG, JPG, and other format support without adding
 * complex dependencies to my project, I am using it for some custom image loading and
 * for my sky box images.
 */
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"