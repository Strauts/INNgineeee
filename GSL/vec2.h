#ifndef VEC2_H
#define VEC2_H

#include "gltypes.h"
#include <cmath>
#include <iostream>
#include "vec3.h"

namespace gsl
{

    class Vec2
    {
        public:
            //Constructors
            Vec2(GLfloat x_in = 0.f, GLfloat y_in = 0.f);
            Vec2(const int v);
            Vec2(const double v);

            //Operators
            Vec2 operator+(const Vec2 &rhs) const;      // v + v
            Vec2 operator-(const Vec2 &rhs) const;      // v - v
            Vec2& operator+=(const Vec2 &rhs);          // v += v
            Vec2& operator-=(const Vec2 &rhs);          // v -= v
            Vec2 operator-() const;                     // -v
            Vec2 operator*(GLfloat lhs) const;          // v * f
            const Vec2& operator =(const Vec2 &rhs);    // v = v
            Vec3 operator^(const Vec2 &rhs);      // v x v

            //Functions
            GLfloat length() const;
            void normalize();
            Vec2 normalized();
            static GLfloat cross(const Vec2 &v1, const Vec2 &v2);
            static GLfloat dot(const Vec2 &v1, const Vec2 &v2);
            class Vec3 barycentricCoordinates(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3);

            //Getters and setters
            GLfloat getX() const;
            void setX(const GLfloat &value);

            GLfloat getY() const;
            void setY(const GLfloat &value);

            //Friend functions
            friend std::ostream& operator<<(std::ostream &output, const Vec2 &rhs)
            {
                output << "(" << rhs.x << "," << rhs.y << ")";
                return output;
            }

        private:
            GLfloat x;
            GLfloat y;
    };

} //namespace

#endif // VEC2_H
