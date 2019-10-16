#include "vec3.h"
#include "math_constants.h"
#include "gsl_math.h"
#include <cmath>

namespace gsl
{

    Vec3::Vec3(GLfloat x_in, GLfloat y_in, GLfloat z_in) : x{x_in}, y{y_in}, z{z_in}
    {

    }


    Vec3::Vec3(const int v) : x{static_cast<float>(v)}, y{static_cast<float>(v)}, z{static_cast<float>(v)}
    {

    }


    Vec3::Vec3(const double v) : x{static_cast<float>(v)}, y{static_cast<float>(v)}, z{static_cast<float>(v)}
    {

    }

    Vec3 Vec3::operator*(float rhs) const
    {
        return Vec3(x * rhs, y * rhs, z * rhs);
    }

    //OEF: må lages som friend funksjon
    //Vec3 operator*(const Vec3 &lhs, const Vec3 &rhs)
    //{
    //    return { lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z };
    //}

    const Vec3& Vec3::operator=(const Vec3 &rhs)
    {
        x = rhs.getX();
        y = rhs.getY();
        z = rhs.getZ();

        return *this;
    }


    Vec3 Vec3::operator+(const Vec3 &rhs) const
    {
        return {x + rhs.getX(), y + rhs.getY(), z + rhs.getZ()};
    }


    Vec3 Vec3::operator-(const Vec3 &rhs) const
    {
        return {x - rhs.getX(), y - rhs.getY(), z - rhs.getZ()};
    }


    Vec3& Vec3::operator+=(const Vec3 &rhs)
    {
        x += rhs.getX();
        y += rhs.getY();
        z += rhs.getZ();

        return *this;
    }


    Vec3& Vec3::operator-=(const Vec3 &rhs)
    {
        x -= rhs.getX();
        y -= rhs.getY();
        z -= rhs.getZ();

        return *this;
    }


    Vec3 Vec3::operator-() const
    {
        return {-x, -y, -z};
    }

    GLfloat Vec3::operator*(const Vec3 &rhs) const
    {
        return x*rhs.getX() + y*rhs.getY() + z*rhs.getZ();
    }

    Vec3 Vec3::operator^(const Vec3 &rhs) const
    {
        return {y * rhs.getZ() - z * rhs.getY(), z * rhs.getX() - x * rhs.getZ(), x * rhs.getY() - y * rhs.getX()};
    }

    GLfloat Vec3::length() const
    {
        return std::sqrt(std::pow(x, 2.f) + std::pow(y, 2.f) + std::pow(z, 2.f));
    }


    void Vec3::normalize()
    {
        GLfloat l = length();

        if (l > 0.f)
        {
            x = x / l;
            y = y / l;
            z = z / l;
        }
    }


    Vec3 Vec3::normalized()
    {
        Vec3 normalized;
        GLfloat l = length();

        if (l > 0.f)
        {
            normalized.setX(x / l);
            normalized.setY(y / l);
            normalized.setZ(z / l);
        }

        return normalized;
    }


    Vec3 Vec3::cross(const Vec3 &v1, const Vec3 &v2)
    {
        return {((v1.getY() * v2.getZ()) - (v1.getZ() * v2.getY())), ((v1.getZ() * v2.getX()) - (v1.getX() * v2.getZ())), ((v1.getX() * v2.getY()) - (v1.getY() * v2.getX()))};
    }


    GLfloat Vec3::dot(const Vec3 &v1, const Vec3 &v2)
    {
        return ((v1.getX() * v2.getX()) + (v1.getY() * v2.getY()) + (v1.getZ() * v2.getZ()));
    }


    void Vec3::rotateX(GLfloat angle)
    {
        Vec3 dir;
        angle = gsl::deg2rad(angle);

        dir.setX(x);
        dir.setY((y * std::cos(angle)) + (z * (-std::sin(angle))));
        dir.setZ((y * std::sin(angle)) + (z * std::cos(angle)));

        x = dir.getX();
        y = dir.getY();
        z = dir.getZ();
    }


    void Vec3::rotateY(GLfloat angle)
    {
        Vec3 dir;
        angle = gsl::deg2rad(angle);

        dir.setX((x * std::cos(angle)) + (z * std::sin(angle)));
        dir.setY(y);
        dir.setZ((x * (-std::sin(angle))) + (z * std::cos(angle)));

        x = dir.getX();
        y = dir.getY();
        z = dir.getZ();
    }


    void Vec3::rotateZ(GLfloat angle)
    {
        Vec3 dir;
        angle = gsl::deg2rad(angle);

        dir.setX((x * std::cos(angle)) + (y * (-std::sin(angle))));
        dir.setY((x * std::sin(angle)) + (y * std::cos(angle)));
        dir.setZ(z);

        x = dir.getX();
        y = dir.getY();
        z = dir.getZ();
    }


    GLfloat Vec3::getX() const
    {
        return x;
    }


    void Vec3::setX(const GLfloat &value)
    {
        x = value;
    }


    GLfloat Vec3::getY() const
    {
        return y;
    }


    void Vec3::setY(const GLfloat &value)
    {
        y = value;
    }


    GLfloat Vec3::getZ() const
    {
        return z;
    }


    void Vec3::setZ(const GLfloat &value)
    {
        z = value;
    }

} //namespace
