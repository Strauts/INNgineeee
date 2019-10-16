 #ifndef TEXTUREMATERIAL_H
#define TEXTUREMATERIAL_H

#include "material.h"
#include "gltypes.h"
#include "mat4.h"

class Shader;
class Texture;

class TextureMaterial : public Material
{

    public:
        TextureMaterial(Shader *shader);

        void setMMatrix(gsl::Mat4 &modelMatrixIn) override;
        void setupShader(GLuint VAOin) override;

        void setTexture1(Texture *texture1);
        void setTexture2(Texture *texture2);

    private:
        GLint mModelMatrixUniform{0};
        GLint mViewMatrixUniform{0};
        GLint mPerspectiveMatrixUniform{0};

        GLint mTextureUniform1{0};
        GLint mTextureUniform2{0};

        Texture *mTexture1{nullptr};
        Texture *mTexture2{nullptr};
};

#endif // TEXTUREMATERIAL_H
