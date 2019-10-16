#include "transform.h"
#include "entity.h"
#include "mat4.h"


Transform::Transform() : mLocalScale{1.f, 1.f, 1.f}
{
}

gsl::Vec3& Transform::getPosition()
{
    return mLocalPosition;
}

gsl::Quaternion& Transform::getRotation()
{
    return mLocalRotation;
}

gsl::Vec3& Transform::getScale()
{
    return mLocalScale;
}

gsl::Mat4 Transform::getModelMatrix()
{
    //Rotate, then scale, then translate
    gsl::Mat4 model(true);
    model.translate(mLocalPosition.getX(), mLocalPosition.getY(), mLocalPosition.getZ());
    model.scale(mLocalScale);
    model.rotate(mLocalRotation);

    return model;
}

void Transform::setPosition(GLfloat x, GLfloat y, GLfloat z)
{
    mLocalPosition = gsl::Vec3(x, y, z);
}

void Transform::setPosition(gsl::Vec3 pos)
{
    mLocalPosition = pos;
}

void Transform::setRotation(GLfloat x, GLfloat y, GLfloat z)
{
    mLocalRotation = gsl::Quaternion(x, y, z);
}

void Transform::setRotation(gsl::Quaternion rot)
{
    mLocalRotation = rot;
}

void Transform::setScale(GLfloat x, GLfloat y, GLfloat z)
{
    mLocalScale = gsl::Vec3(x, y, z);
}

void Transform::setScale(gsl::Vec3 scale)
{
    mLocalScale = scale;
}

void Transform::translate(GLfloat x, GLfloat y, GLfloat z)
{
    mLocalPosition = mLocalPosition + gsl::Vec3{x, y, z};
}

void Transform::translate(gsl::Vec3 vecIn)
{
    mLocalPosition = mLocalPosition + vecIn;
}

void Transform::rotate(GLfloat angle_deg, gsl::Vec3 axis)
{
    mLocalRotation = gsl::Quaternion(angle_deg, axis) * mLocalRotation;
}

void Transform::rotate(gsl::Quaternion quat)
{
    mLocalRotation = mLocalRotation * quat;
}

void Transform::rotate(GLfloat x, GLfloat y, GLfloat z)
{
    mLocalRotation = gsl::Quaternion(x, y, z) * mLocalRotation;
}
