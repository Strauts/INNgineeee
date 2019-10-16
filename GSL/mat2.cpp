#include "mat2.h"
#include "mat3.h"
#include "mat4.h"

namespace gsl
{

    Mat2::Mat2(bool isIdentity)
    {
        if(isIdentity)
        {
            identity();
        }
        else
        {
            for(int i = 0; i < 4; i++)
                matrix[i] = 0.f;
        }
    }


    Mat2::Mat2(std::initializer_list<GLfloat> values)
    {
        int i = 0;
        for(auto value : values)
            matrix[i++] = value;
    }


    Mat2 Mat2::identity()
    {
        setToIdentity();

        return *this;
    }


    void Mat2::setToIdentity()
    {
        *this =
        {
            1, 0,
            0, 1
        };
    }


    GLfloat Mat2::determinant()
    {
        return (matrix[0]*matrix[3] - matrix[1]*matrix[2]);
    }


    bool Mat2::inverse()
    {
        GLfloat det = determinant();

        if(det == 0.f)
            return false;

        det = 1.f / det;

        *this =
        {
             det*matrix[3], -det*matrix[1],
            -det*matrix[2],  det*matrix[0]
        };

        return true;
    }


    void Mat2::transpose()
    {
        std::swap(matrix[1], matrix[2]);
    }


    Mat3 Mat2::toMatrix3()
    {
        return Mat3
        {
            matrix[0], matrix[1], 0,
            matrix[2], matrix[3], 0,
                0,        0,      1
        };
    }


    Mat4 Mat2::toMatrix4()
    {
        return Mat4
        {
            matrix[0], matrix[1], 0, 0,
            matrix[2], matrix[3], 0, 0,
                0,        0,      1, 0,
                0,        0,      0, 1
        };
    }


    Mat2 Mat2::operator*(const Mat2 &other)
    {
        return Mat2
        {
            matrix[0] * other.matrix[0] + matrix[1] * other.matrix[2], matrix[0] * other.matrix[1] + matrix[1] * other.matrix[3],
            matrix[2] * other.matrix[0] + matrix[3] * other.matrix[2], matrix[2] * other.matrix[1] + matrix[3] * other.matrix[3]
        };
    }


    Vec2 Mat2::operator*(const Vec2 &v)
    {
        return Vec2(matrix[0] * v.getX() + matrix[1] * v.getY(), matrix[2] * v.getX() + matrix[3] * v.getY());
    }


    GLfloat& Mat2::operator()(int y, int x)
    {
        return matrix[y * 2 + x];
    }


    GLfloat Mat2::operator()(int y, int x) const
    {
        return matrix[y * 2 + x];
    }

} //namespace
