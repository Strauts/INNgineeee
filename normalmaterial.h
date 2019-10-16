#ifndef NORMALMATEREAL_H
#define NORMALMATEREAL_H

#include "gltypes.h"
#include "material.h"
#include "mat4.h"
#include "vec3.h"

class Texture;
class Shader;
class PointLight;

class NormalMaterial : public Material
{
    public:
        NormalMaterial(Shader *shader, PointLight *pointLight);

        void setMMatrix(gsl::Mat4 &modelMatrixIn) override;
        void setupShader(GLuint VAOin) override;

        void setPointLight(PointLight *pointLight);
        PointLight* getPointLight() const;

        void setColors(const gsl::Vec3 &ambientColor, const gsl::Vec3 &diffuseColor, const gsl::Vec3 &specularColor, GLfloat shinyness);
        void setNormalMap(Texture *normalMap);

    private:
        GLint mModelMatrixUniform;
        GLint mViewMatrixUniform;
        GLint mPerspectiveMatrixUniform;

        GLint mCameraPosition;
        GLint mLightPosition;

        gsl::Vec3 mAmbientColor;
        gsl::Vec3 mDiffuseColor;
        gsl::Vec3 mSpecularColor;
        GLfloat mShinyness;

        PointLight *mPointLight;

        GLint mLightAmbientColor;
        GLint mLightDiffuseColor;
        GLint mLightSpecularColor;

        GLint mMaterialAmbientColor;
        GLint mMaterialDiffuseColor;
        GLint mMaterialSpecularColor;
        GLint mMaterialShininess;

        gsl::Mat4 mMVPMatrix;

        Texture *mNormalMap;

        GLint mNormalMapUniform;
};

#endif // NORMALMATEREAL_H
