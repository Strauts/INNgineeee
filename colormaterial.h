#ifndef COLORMATERIAL_H
#define COLORMATERIAL_H

#include "material.h"
#include "mat4.h"
#include "gltypes.h"

class Shader;

class ColorMaterial : public Material
{
    public:
        ColorMaterial(Shader *shader);

        void setMMatrix(gsl::Mat4 &modelMatrixIn) override;
        void setupShader(GLuint VAOin) override;

    private:
        GLint mMVPUniform{0};
        gsl::Mat4 mMVPMatrix;
};
#endif // COLORMATERIAL_H
