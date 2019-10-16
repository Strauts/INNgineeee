#ifndef GLTYPES_H
#define GLTYPES_H

//We put this test inside this file, instead of having it in each file that needs gl.h

#ifdef _WIN32
#include <GL/gl.h>      // OpenGL types and declarations on Windows
#endif
#ifdef __APPLE__
#include <opengl/gl.h>  // OpenGL types and declarations on Mac
#endif

#endif // GLTYPES_H
