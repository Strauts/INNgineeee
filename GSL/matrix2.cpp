#include "matrix2.h"
#include "matrix3.h"
#include "matrix4.h"

namespace gsl
{
    Matrix2::Matrix2(bool isIdentity)
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

    Matrix2::Matrix2(std::initializer_list<GLfloat> values)
    {
        int i = 0;
        for(auto value : values)
            matrix[i++] = value;
    }

    Matrix2 Matrix2::identity()
    {
        setToIdentity();

        return *this;
    }

    void Matrix2::setToIdentity()
    {
        *this =
        {
            1, 0,
            0, 1
        };
    }

    Matrix2::GLfloat Matrix2::determinant()
    {
        return (matrix[0]*matrix[3] - matrix[1]*matrix[2]);
    }

    bool Matrix2::inverse()
    {
        GLfloat det = (matrix[0]*matrix[3] - matrix[1]*matrix[2]);

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

    void Matrix2::transpose()
    {
        std::swap(matrix[1], matrix[2]);
    }

    Matrix3 Matrix2::toMatrix3()
    {
        return Matrix3
        {
            matrix[0], matrix[1], 0,
            matrix[2], matrix[3], 0,
                0,        0,      1
        };
    }

    Matrix4 Matrix2::toMatrix4()
    {
        return Matrix4
        {
            matrix[0], matrix[1], 0, 0,
            matrix[2], matrix[3], 0, 0,
                0,        0,      1, 0,
                0,        0,      0, 1
        };
    }

    Matrix2 Matrix2::operator*(const Matrix2 &other)
    {
        return Matrix2
        {
            matrix[0] * other.matrix[0] + matrix[1] * other.matrix[2], matrix[0] * other.matrix[1] + matrix[1] * other.matrix[3],
            matrix[2] * other.matrix[0] + matrix[3] * other.matrix[2], matrix[2] * other.matrix[1] + matrix[3] * other.matrix[3]
        };
    }

    Vec2 Matrix2::operator*(const Vec2 &v)
    {
        return Vec2(matrix[0] * v.getX() + matrix[1] * v.getY(), matrix[2] * v.getX() + matrix[3] * v.getY());
    }

    GLfloat &Matrix2::operator()(int y, int x)
    {
        return matrix[y * 2 + x];
    }

    GLfloat Matrix2::operator()(int y, int x) const
    {
        return matrix[y * 2 + x];
    }
}
