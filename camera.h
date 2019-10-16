#ifndef CAMERA_H
#define CAMERA_H

#include "vec3.h"
#include "mat4.h"
#include "gltypes.h"
#include "componentbase.h"

class Camera
{
    public:
        Camera(GLint width = 900, GLint height = 600);
        ~Camera();

        void setPerspectiveMatrix(GLint width, GLint height);
        void setViewMatrix();

        void setTarget(const gsl::Vec3 &target);
        void setDirection(GLfloat horizontalAngle, GLfloat verticalAngle);
        void movement(bool w, bool s, bool d, bool a, GLfloat deltaTime);

        gsl::Mat4* getViewMatrix();
        gsl::Mat4* getPerspectiveMatrix() const;

        TransformComponent& getTransform();

    private:
        TransformComponent mTransform;

        gsl::Mat4 *mViewMatrix{nullptr};
        gsl::Mat4 *mPerspectiveMatrix{nullptr};

        GLfloat mNearPlane{0.1f};
        GLfloat mFarPlane{1000.f};
        GLfloat mFieldOfView{45.f};

        gsl::Vec3 mTarget;       //What to look at
        gsl::Vec3 mDirection{0.f, 0.f, -1.f};    //Direction to look
        gsl::Vec3 mRight{1.f, 0.f, 0.f};
        gsl::Vec3 mUp{0.f, 1.f, 0.f};

        bool mOrtho{false};

        GLint mWidth;
        GLint mHeight;
};

#endif // CAMERA_H
