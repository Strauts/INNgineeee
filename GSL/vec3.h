#ifndef VEC3_H
#define VEC3_H

#include "gltypes.h"
#include <cmath>
#include <iostream>

namespace gsl
{

    class Vec3
    {
        public:
            //Constructors
            Vec3(GLfloat x_in = 0.f, GLfloat y_in = 0.f, GLfloat z_in = 0.f);
            Vec3(const int v);
            Vec3(const double v);

            //Operators
            const Vec3& operator=(const Vec3 &rhs);     // v = v
            Vec3 operator+(const Vec3 &rhs) const;      // v + v
            Vec3 operator-(const Vec3 &rhs) const;      // v - v
            Vec3& operator+=(const Vec3 &rhs);          // v += v
            Vec3& operator-=(const Vec3 &rhs);          // v -= v
            Vec3 operator-() const;                     // -v
            GLfloat operator*(const Vec3& rhs) const;      // v * v
            Vec3 operator*(GLfloat rhs) const;          // v * f
            Vec3 operator^(const Vec3& rhs) const;      // v x v
            Vec3 operator/(GLfloat rhs) const;          // v / f

            //Functions
            GLfloat length() const;
            void normalize();
            Vec3 normalized();
            static Vec3 cross(const Vec3 &v1, const Vec3 &v2);
            static GLfloat dot(const Vec3 &v1, const Vec3 &v2);

            void rotateX(GLfloat angle);
            void rotateY(GLfloat angle);
            void rotateZ(GLfloat angle);

            //Getters and setters
            GLfloat getX() const;
            void setX(const GLfloat &value);

            GLfloat getY() const;
            void setY(const GLfloat &value);

            GLfloat getZ() const;
            void setZ(const GLfloat &value);

            //Friend functions
            friend std::ostream& operator<<(std::ostream &output, const Vec3 &rhs )
            {
                output << "X = " << rhs.x << ", Y = " << rhs.y << ", Z = " << rhs.z;
                return output;
            }

            GLfloat x;
            GLfloat y;
            GLfloat z;
    };

} //namespace

#endif // VEC3_H
