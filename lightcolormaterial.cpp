#include "lightcolormaterial.h"
#include "shader.h"
#include "vertex.h"
#include "camera.h"
#include "transform.h"
#include "pointlight.h"
#include "directionallight.h"
#include "camerasystem.h"

LightColorMaterial::LightColorMaterial(Shader *shader, DirectionalLight *directionalLight) : Material{shader}, mDirectionalLight{directionalLight}
{
    mSpecularColor = gsl::Vec3(1.f, 1.f, 1.f);
    mMaterialColor = gsl::Vec3(0.5f, 1.f, 0.5f);
    mSpecularPower = 0.3f;
}

void LightColorMaterial::setupShader(GLuint VAOin)
{
    Material::setupShader(VAOin);

    //nable the uniforms
    mModelMatrixUniform = glGetUniformLocation(mShader->getProgram(), "mMatrix");
    mViewMatrixUniform = glGetUniformLocation(mShader->getProgram(), "vMatrix");
    mPerspectiveMatrixUniform = glGetUniformLocation(mShader->getProgram(), "pMatrix");

    mCameraPosition = glGetUniformLocation(mShader->getProgram(), "cameraPosition");
    mLightNormal= glGetUniformLocation(mShader->getProgram(), "light.normal_cs");

    mLightIntensityUniform = glGetUniformLocation(mShader->getProgram(), "light.intensity");
    mLightAmbientColorUniform = glGetUniformLocation(mShader->getProgram(), "light.ambient");
    mLightDiffuseColorUniform = glGetUniformLocation(mShader->getProgram(), "light.diffuse");
    mLightSpecularColorUniform = glGetUniformLocation(mShader->getProgram(), "light.specular");

    mColorUniform = glGetUniformLocation(mShader->getProgram(), "material.color");
    mSpecularColorUniform = glGetUniformLocation(mShader->getProgram(), "material.specularColor");
    mSpecularPowerUniform = glGetUniformLocation(mShader->getProgram(), "material.specularPower");

    glUseProgram(0);
    glBindVertexArray(0);
}

void LightColorMaterial::setPointLight(PointLight *pointLight)
{
    mPointLight = pointLight;
}

PointLight* LightColorMaterial::getPointLight() const
{
    return mPointLight;
}

void LightColorMaterial::setDirectionalLight(DirectionalLight *directionalLight)
{
    mDirectionalLight = directionalLight;
}

DirectionalLight *LightColorMaterial::getDirectionalLight() const
{
    return mDirectionalLight;
}

void LightColorMaterial::setColors(const gsl::Vec3 &ambientColor, const gsl::Vec3 &diffuseColor, const gsl::Vec3 &specularColor, GLfloat shinyness)
{
    //mAmbientColor = ambientColor;
    //mDiffuseColor = diffuseColor;
    //mSpecularColor = specularColor;
    //mShinyness = shinyness;
}

void LightColorMaterial::setMMatrix(gsl::Mat4 &modelMatrixIn)
{
    CameraComponent cameraComponent = CameraSystem::getInstance()->getCurrentCamera();
    glUniformMatrix4fv(mModelMatrixUniform, 1, GL_TRUE, modelMatrixIn.constData());
    glUniformMatrix4fv(mViewMatrixUniform, 1, GL_TRUE, cameraComponent.mViewMatrix.constData());
    glUniformMatrix4fv(mPerspectiveMatrixUniform, 1, GL_TRUE, cameraComponent.mPerspectiveMatrix.constData());

    gsl::Vec3 lightNormal = mDirectionalLight->getNormal();

    lightNormal = (cameraComponent.mViewMatrix * modelMatrixIn * gsl::Vec4(lightNormal, 0)).toVector3D();

    glUniform3f(mLightNormal, lightNormal.getX(), lightNormal.getY(), lightNormal.getZ());

    gsl::Vec3 cameraPos = gsl::Vec3(-cameraComponent.mViewMatrix(0, 3), -cameraComponent.mViewMatrix(1, 3), -cameraComponent.mViewMatrix(2, 3));
    glUniform3f(mCameraPosition, cameraPos.getX(), cameraPos.getY(), cameraPos.getZ());

    glUniform1f(mLightIntensityUniform, mDirectionalLight->lightIntensity);
    glUniform3f(mLightAmbientColorUniform, mDirectionalLight->mAmbientColor.getX(), mDirectionalLight->mAmbientColor.getY(), mDirectionalLight->mAmbientColor.getZ());
    glUniform3f(mLightDiffuseColorUniform, mDirectionalLight->mLightColor.getX(), mDirectionalLight->mLightColor.getY(), mDirectionalLight->mLightColor.getZ());
    glUniform3f(mLightSpecularColorUniform, mDirectionalLight->mSpecularColor.getX(), mDirectionalLight->mSpecularColor.getY(), mDirectionalLight->mSpecularColor.getZ());

    //Set material properties
    glUniform3f(mColorUniform, mMaterialColor.getX(), mMaterialColor.getY(), mMaterialColor.getZ());
    glUniform3f(mSpecularColorUniform, mSpecularColor.getX(), mSpecularColor.getY(), mSpecularColor.getZ()); //Specular doesn't have full effect on this object's material
    glUniform1f(mSpecularPowerUniform, mSpecularPower);
}
