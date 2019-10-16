#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

#define NUMSHADERS 4;
#define NUMTEXTURES 4;
#define NUMMATERIALS 4;

namespace gsl  //Game School Lib
{
    const std::string assetFilePath{"../INNgine/Assets/"};
    const std::string shaderFilePath{"../INNgine/Shaders/"};
    const std::string soundFilePath{"../INNgine/Assets/Sounds/"};

    enum ShaderTypes
    {
        COLOR,
        TEXTURE,
        LIGHTCOLOR,
        NORMAL,
        LIGHTTEXTURE, //does not exist
        NUM_SHADERS
    };

    enum ShapeType
    {
        CUBE,
        PLANE,
        SPHERE,
        AXIS
    };

    enum ComponentType
    {
        MESH,
        MATERIAL,
        TRANSFORM,
        CAMERA,

        LAST
    };
}

#endif // CONSTANTS_H
