#include "vec2.h"
#include "vec3.h"
#include <cmath>

namespace gsl
{

    Vec2::Vec2(GLfloat x_in, GLfloat y_in) : x{x_in}, y{y_in}
    {

    }


    Vec2::Vec2(const int v) : x{static_cast<GLfloat>(v)}, y{static_cast<GLfloat>(v)}
    {

    }


    Vec2::Vec2(const double v) : x{static_cast<GLfloat>(v)}, y{static_cast<GLfloat>(v)}
    {

    }

    Vec3 Vec2::operator^(const Vec2 &rhs)
    {
        return Vec3(x, 0, y) ^ Vec3(rhs.getX(), 0, rhs.getY());
    }

    Vec2 Vec2::operator-() const
    {
        return { -x, -y };
    }

    const Vec2& Vec2::operator=(const Vec2 &rhs)
    {
        x = rhs.getX();
        y = rhs.getY();
        return *this;
    }


    Vec2 Vec2::operator+(const Vec2 &rhs) const
    {
        return {x + rhs.getX(), y + rhs.getY()};
    }


    Vec2 Vec2::operator-(const Vec2 &rhs) const
    {
        return {x - rhs.getX(), y - rhs.getY()};
    }


    Vec2& Vec2::operator+=(const Vec2 &rhs)
    {
        x += rhs.getX();
        y += rhs.getY();

        return *this;
    }

    Vec3 Vec2::barycentricCoordinates(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3)
    {
        Vec2 p12 = p2-p1;
        Vec2 p13 = p3-p1;

        Vec3 n = p12^p13;

        float areal_123 = n.length(); // dobbelt areal

        Vec3 baryc; // til retur. Husk u

        Vec2 p = p2 - *this;

        Vec2 q = p3 - *this;

        n = p^q;

        baryc.setX(n.getY()/areal_123);

        // v
        p = p3 - *this;
        q = p1 - *this;

        n = p^q;

        baryc.setY(n.getY() / areal_123);

        // w
        p = p1 - *this;
        q = p2 - *this;
        n = p^q;

        baryc.setZ(n.getY() / areal_123);

        return baryc;
    }

    Vec2& Vec2::operator-=(const Vec2 &rhs)
    {
        x -= rhs.getX();
        y -= rhs.getY();

        return *this;
    }


    Vec2 Vec2::operator*(float lhs) const
    {
        return {x * lhs, y * lhs};
    }


    GLfloat Vec2::length() const
    {
        return std::sqrt(std::pow(x, 2.f) + std::pow(y, 2.f));
    }


    void Vec2::normalize()
    {
        GLfloat l = length();

        if (l > 0.f)
        {
            x = (x / l);
            y = (y / l);
        }
    }


    Vec2 Vec2::normalized()
    {
        Vec2 normalized;
        GLfloat l = length();

        if (l > 0.f)
        {
            normalized.setX(x / l);
            normalized.setY(y / l);
        }

        return normalized;
    }


    GLfloat Vec2::cross(const Vec2 &v1, const Vec2 &v2)
    {
        return std::abs((v1.getX() * v2.getY()) - (v1.getY() * v2.getX()));
    }


    GLfloat Vec2::dot(const Vec2  &v1, const Vec2  &v2)
    {
        return ((v1.getX() * v2.getX()) + (v1.getY() * v2.getY()));
    }

    GLfloat Vec2::getX() const
    {
        return x;
    }


    void Vec2::setX(const GLfloat &value)
    {
        x = value;
    }


    GLfloat Vec2::getY() const
    {
        return y;
    }


    void Vec2::setY(const GLfloat &value)
    {
        y = value;
    }

} //namespace
