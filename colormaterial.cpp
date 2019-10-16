#include "colormaterial.h"
#include "shader.h"
#include "camerasystem.h"

ColorMaterial::ColorMaterial(Shader *shader) : Material{shader}
{
}

void ColorMaterial::setupShader(GLuint VAOin)
{
    //Common stuff for all materials / shaders
    Material::setupShader(VAOin);

    //Enable the uniforms
    mMVPUniform = glGetUniformLocation(mShader->getProgram(), "matrix");
    glUseProgram(0);
    glBindVertexArray(0);
}

void ColorMaterial::setMMatrix(gsl::Mat4 &modelMatrixIn)
{
    CameraComponent cameraComponent = CameraSystem::getInstance()->getCurrentCamera();
    mMVPMatrix = cameraComponent.mPerspectiveMatrix * cameraComponent.mViewMatrix * modelMatrixIn;
    glUniformMatrix4fv(mMVPUniform, 1, GL_TRUE, mMVPMatrix.constData());
}
