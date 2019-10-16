#include "camera.h"
#include "transform.h"
#include <cmath>


Camera::Camera(GLint width, GLint height) : mWidth{width}, mHeight{height}
{
    //Initialize matrixes
    mViewMatrix = new gsl::Mat4;
    mViewMatrix->setToIdentity();
    mPerspectiveMatrix = new gsl::Mat4;
}

Camera::~Camera()
{
    mPerspectiveMatrix = new gsl::Mat4;
}

void Camera::setPerspectiveMatrix(GLint width, GLint height)
{
    mWidth = width;
    mHeight = height;
    mPerspectiveMatrix->setToIdentity();

    if (mOrtho)
    {
        GLfloat scale{0.005f};
        mPerspectiveMatrix->ortho(-mWidth*scale/2.f, mWidth*scale/2.f, -mHeight*scale/2.f , mHeight*scale/2.f, mNearPlane, mFarPlane);
    }
    else
    {
        GLfloat aspectRatio = static_cast<GLfloat>(mWidth)/static_cast<GLfloat>(mHeight ? mHeight : 1); //Height never == 0
        mPerspectiveMatrix->perspective(mFieldOfView, aspectRatio, mNearPlane, mFarPlane);
    }
}

void Camera::setViewMatrix()
{
    mViewMatrix->lookAt(mTransform.mTransform.getPosition(), mTransform.mTransform.getPosition() + mDirection, mUp);
}

void Camera::setTarget(const gsl::Vec3 &target)
{
    mTarget = target;
}

void Camera::setDirection(GLfloat horizontalAngle, GLfloat verticalAngle)
{
    mDirection = gsl::Vec3(std::cos(verticalAngle) * std::sin(horizontalAngle),
                           std::sin(verticalAngle),
                           std::cos(verticalAngle) * std::cos(horizontalAngle));

    mRight = gsl::Vec3(std::sin(horizontalAngle - 3.14f/2.0f),
                       0,
                       std::cos(horizontalAngle - 3.14f/2.0f));

    mUp = gsl::Vec3::cross(mRight, mDirection);
    gsl::Vec3 correctedViewPosition{-mDirection.getX(), -mDirection.getY(), -mDirection.getZ()};
}

void Camera::movement(bool w, bool s, bool d, bool a, GLfloat deltaTime)
{
    float speed{300.f};
    gsl::Vec3 currentPosition = mTransform.mTransform.getPosition();

    if (w)
    {
        currentPosition = currentPosition + mDirection * deltaTime * speed;
    }
    //Move backward
    if (s)
    {
        currentPosition = currentPosition - mDirection * deltaTime * speed;
    }
    //Strafe right
    if (d)
    {
        currentPosition =  currentPosition + mRight * deltaTime * speed;
    }
    //Strafe left
    if (a)
    {
        currentPosition =  currentPosition - mRight * deltaTime * speed;
    }

    mTransform.mTransform.setPosition(currentPosition);
}

gsl::Mat4* Camera::getPerspectiveMatrix() const
{
    return mPerspectiveMatrix;
}

TransformComponent& Camera::getTransform()
{
    return mTransform;
}

gsl::Mat4* Camera::getViewMatrix()
{
    gsl::Mat4* View = new gsl::Mat4;
    View->lookAt(mTransform.mTransform.getPosition(), mTransform.mTransform.getPosition()+mDirection, mUp);
    return View;
}
