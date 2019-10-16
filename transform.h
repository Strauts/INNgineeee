#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "vec3.h"
#include "quaternion.h"
#include "gltypes.h"

class Entity;

class Transform
{

    public:
        Transform();
        gsl::Vec3& getPosition();
        gsl::Quaternion& getRotation();
        gsl::Vec3& getScale();

        gsl::Mat4 getModelMatrix();

        void setPosition(GLfloat x, GLfloat y, GLfloat z);
        void setPosition(gsl::Vec3 pos);

        void setRotation(GLfloat x, GLfloat y, GLfloat z);
        void setRotation(gsl::Quaternion rot);

        void setScale(GLfloat x, GLfloat y, GLfloat z);
        void setScale(gsl::Vec3 scale);

        void translate(GLfloat x, GLfloat y, GLfloat z);
        void translate(gsl::Vec3 vecIn);

        void rotate(GLfloat angle_deg, gsl::Vec3 axis);
        void rotate(gsl::Quaternion quat);
        void rotate(GLfloat x, GLfloat y, GLfloat z);

        gsl::Vec3 mLocalPosition;
        gsl::Quaternion mLocalRotation;
        gsl::Vec3 mLocalScale{1.f, 1.f, 1.f};
};

#endif // TRANSFORM_H
