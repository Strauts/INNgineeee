#include "vec4.h"
#include "vec3.h"
#include <cmath>

namespace gsl
{

    Vec4::Vec4(GLfloat x_in, GLfloat y_in, GLfloat z_in, GLfloat w_in) : x{x_in}, y{y_in}, z{z_in}, w{w_in}
    {

    }


    Vec4::Vec4(Vec3 vec3_in, GLfloat w_in) : x{vec3_in.getX()}, y{vec3_in.getY()}, z{vec3_in.getZ()}, w{w_in}
    {

    }


    Vec4::Vec4(const Vec3 &vec3_in) : x{vec3_in.getX()}, y{vec3_in.getY()}, z{vec3_in.getZ()}, w{1.f}
    {

    }


    Vec4::Vec4(const int v) : x{static_cast<GLfloat>(v)}, y{static_cast<GLfloat>(v)}, z{static_cast<GLfloat>(v)}, w{1.f}
    {

    }


    Vec4::Vec4(const double v) : x{static_cast<GLfloat>(v)}, y{static_cast<GLfloat>(v)}, z{static_cast<GLfloat>(v)}, w{1.f}
    {

    }


    const Vec4& Vec4::operator=(const Vec4& rhs)
    {
        x = rhs.getX();
        y = rhs.getY();
        z = rhs.getZ();
        w = rhs.getW();

        return *this;
    }


    Vec4 Vec4::operator+(const Vec4& rhs) const
    {
        return {x + rhs.getX(), y + rhs.getY(), z + rhs.getZ(), w + rhs.getW()};
    }


    Vec4 Vec4::operator-(const Vec4& rhs) const
    {
        return {x - rhs.getX(), y - rhs.getY(), z - rhs.getZ(), w - rhs.getW()};
    }


    Vec4& Vec4::operator+=(const Vec4 &rhs)
    {
        x += rhs.getX();
        y += rhs.getY();
        z += rhs.getZ();
        w += rhs.getW();

        return *this;
    }


    Vec4& Vec4::operator-=(const Vec4 &rhs)
    {
        x -= rhs.getX();
        y -= rhs.getY();
        z -= rhs.getZ();
        w -= rhs.getW();

        return *this;
    }


    Vec4 Vec4::operator-() const
    {
        return {-x, -y, -z, -w};
    }


    Vec4 Vec4::operator*(GLfloat rhs) const
    {
        return {x * rhs, y * rhs, z * rhs, w * rhs};
    }


    /*Vec4 operator*(Mat4 q) const
    {
    }*/


    GLfloat Vec4::length()
    {
        return std::sqrt(x*x + y*y + z*z + w*w);
    }


    Vec3 Vec4::toVector3D()
    {
        return Vec3(x, y, z);
    }


    void Vec4::normalize()
    {
        GLfloat l = length();

        if(l > 0.f)
        {
            x = x / l;
            y = y / l;
            z = z / l;
            w = w / l;
        }
    }


    Vec4 Vec4::normalized()
    {
        Vec4 normalized;
        GLfloat l = length();

        if (l > 0.f)
        {
            normalized.setX(x / l);
            normalized.setY(y / l);
            normalized.setZ(z / l);
            normalized.setW(w / l);
        }

        return normalized;
    }


    GLfloat Vec4::dot(const Vec4 &v1, const Vec4 &v2)
    {
        return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY()) + (v1.getZ() * v2.getZ()) + (v1.getW() * v2.getW());
    }


    void Vec4::rotateX(GLfloat angle)
    {
        Vec3 v = toVector3D();
        v.rotateX(angle);

        x = v.getX();
        y = v.getY();
        z = v.getZ();
    }


    void Vec4::rotateY(GLfloat angle)
    {
        Vec3 v = toVector3D();
        v.rotateY(angle);

        x = v.getX();
        y = v.getY();
        z = v.getZ();
    }


    void Vec4::rotateZ(GLfloat angle)
    {
        Vec3 v = toVector3D();
        v.rotateZ(angle);

        x = v.getX();
        y = v.getY();
        z = v.getZ();
    }


    GLfloat Vec4::getX() const
    {
        return x;
    }


    void Vec4::setX(const GLfloat &value)
    {
        x = value;
    }


    GLfloat Vec4::getY() const
    {
        return y;
    }


    void Vec4::setY(const GLfloat &value)
    {
        y = value;
    }


    GLfloat Vec4::getZ() const
    {
        return z;
    }


    void Vec4::setZ(const GLfloat &value)
    {
        z = value;
    }


    GLfloat Vec4::getW() const
    {
        return w;
    }


    void Vec4::setW(const GLfloat &value)
    {
        if (value == 0.f || value == 1.f)    //w should be only 0 or 1
            w = value;
    }

} //namespace
