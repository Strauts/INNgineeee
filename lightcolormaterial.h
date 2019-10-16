#ifndef LIGHTCOLORMATERIAL_H
#define LIGHTCOLORMATERIAL_H

#include "gltypes.h"
#include "material.h"
#include "mat4.h"
#include "vec3.h"

class Shader;
class PointLight;
class DirectionalLight;

class LightColorMaterial : public Material
{
    public:
        LightColorMaterial(Shader *shader, DirectionalLight *directionalLight);

        void setMMatrix(gsl::Mat4 &modelMatrixIn) override;
        void setupShader(GLuint VAOin) override;

        void setPointLight(PointLight *pointLight);
        PointLight* getPointLight() const;

        void setDirectionalLight(DirectionalLight *directionalLight);
        DirectionalLight *getDirectionalLight() const;

        void setColors(const gsl::Vec3 &ambientColor, const gsl::Vec3 &diffuseColor, const gsl::Vec3 &specularColor, GLfloat shinyness);

    private:
        GLint mModelMatrixUniform;
        GLint mViewMatrixUniform;
        GLint mPerspectiveMatrixUniform;

        GLint mCameraPosition;
        GLint mLightNormal;

        gsl::Vec3 mMaterialColor;
        gsl::Vec3 mSpecularColor;
        GLfloat mSpecularPower;

        PointLight *mPointLight{nullptr};
        DirectionalLight* mDirectionalLight{nullptr};

        GLint mLightIntensityUniform;
        GLint mLightAmbientColorUniform;
        GLint mLightDiffuseColorUniform;
        GLint mLightSpecularColorUniform;

        GLint mColorUniform;
        GLint mSpecularColorUniform;
        GLint mSpecularPowerUniform;

        gsl::Mat4 mMVPMatrix;
};

#endif // LIGHTCOLORMATERIAL_H
