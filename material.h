#ifndef MATERIAL_H
#define MATERIAL_H

#include "mat4.h"
#include <QOpenGLFunctions_4_1_Core>

class Shader;
class Camera;

class Material : protected QOpenGLFunctions_4_1_Core
{
    public:
        Material(Shader *shader);
        ~Material();

        Shader* getShader() const;

        virtual void setMMatrix(gsl::Mat4& mMatrix) = 0;
        virtual void setupShader(GLuint VAOin);

        static void setCurrentCamera(Camera *currentCamera);
        static Camera* getCurrentCamera();

    protected:
        static Camera *mCurrentCamera;
        gsl::Mat4 *mMMatrix{nullptr};
        Shader *mShader{nullptr};
};

#endif // MATERIAL_H
