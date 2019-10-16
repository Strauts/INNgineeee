#include "controlledball.h"
#include "math_constants.h"


ControlledBall::ControlledBall()
{
    mRadius = 1.f;
    mVelocity = gsl::Vec3{0.f, 0.f, 0.f};
    mMovementSpeed = 2.f;
}

void ControlledBall::update(GLfloat deltaTime)
{
    gsl::Vec3 rotationAxis = gsl::Vec3::cross(mVelocity, gsl::Vec3{0.f, 1.f, 0.f});
    rotationAxis.normalize();

    mVelocity = mVelocity * deltaTime * mMovementSpeed;

}
