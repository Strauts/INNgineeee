#include "material.h"
#include "shader.h"
#include "vertex.h"
#include "camera.h"


//Static member - this will be the same for all materials!
Camera *Material::mCurrentCamera{};


Material::Material(Shader *shader) : mShader{shader}
{
}

Material::~Material()
{
}

Shader* Material::getShader() const
{
    return mShader;
}

void Material::setupShader(GLuint VAOin)
{
    initializeOpenGLFunctions();
    glUseProgram(mShader->getProgram());

    glBindVertexArray(VAOin);   //Has to know what VAO to set this up for

    //1st attribute buffer : positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    //2nd attribute buffer : colors or normals
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    //3rd attribute buffer : uvs (ST)
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(6 * sizeof(GLfloat)));
    glEnableVertexAttribArray(2);
}

void Material::setCurrentCamera(Camera *currentCamera)
{
    mCurrentCamera = currentCamera;
}

Camera *Material::getCurrentCamera()
{
    return mCurrentCamera;
}
