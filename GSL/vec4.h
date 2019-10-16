#ifndef VEC4_H
#define VEC4_H

#include "gltypes.h"
#include <cmath>
#include <iostream>
#include <cassert>

namespace gsl
{

    class Vec3;

    class Vec4
    {
        public:
            //Constructors
            Vec4(GLfloat x_in = 0.f, GLfloat y_in = 0.f, GLfloat z_in = 0.f, GLfloat w_in = 0.f);
            Vec4(Vec3 vec3_in, GLfloat w_in);
            Vec4(const Vec3 &vec3_in);
            Vec4(const int v);
            Vec4(const double v);

            //Operators:
            const Vec4& operator=(const Vec4 &rhs);     // v = v
            Vec4 operator+(const Vec4 &rhs) const;      // v + v
            Vec4 operator-(const Vec4 &rhs) const;      // v - v
            Vec4& operator+=(const Vec4 &rhs);          // v += v
            Vec4& operator-=(const Vec4 &rhs);          // v -= v
            Vec4 operator-() const;                     // -v
            Vec4 operator*(GLfloat rhs) const;          // v * f
            //Vec4 operator*(Matrix4x4 q) const;        // v * m

            GLfloat& operator[](const int index)
            {
                assert(index <4 && index >=0);

                switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                case 3:
                    return w;
                }
                return x;   //to silence compiler warnings
            }

            //Functions:
            GLfloat length();
            Vec3 toVector3D();
            void normalize();
            Vec4 normalized();
            static GLfloat dot(const Vec4 &v1, const Vec4 &v2);

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

            GLfloat getW() const;
            void setW(const GLfloat &value);

            //Friend functions
            friend std::ostream& operator<<(std::ostream &output, const Vec4 &rhs )
            {
                output << "X = " << rhs.x << ", Y = " << rhs.y <<
                          ", Z = " << rhs.z << ", W = " << rhs.w;
                return output;
            }

        private:
            GLfloat x;
            GLfloat y;
            GLfloat z;
            GLfloat w;
    };

} //namespace

#endif // VEC4_H
