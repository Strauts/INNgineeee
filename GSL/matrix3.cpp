#include "matrix3.h"
#include "matrix2.h"
#include "matrix4.h"

namespace gsl
{
    Matrix3::Matrix3(bool isIdentity)
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

    Matrix3::Matrix3(std::initializer_list<float> values)
    {
        int i = 0;
        for(auto value : values)
            matrix[i++] = value;
    }

    Matrix3 Matrix3::identity()
    {
        setToIdentity();

        return *this;
    }

    void Matrix3::setToIdentity()
    {
        *this =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };
    }

    Matrix3::GLfloat Matrix3::determinant()
    {
        //det = a(ei − fh) − b(di − fg) + c(dh − eg)
        // der
        //    a, b, c,
        //    d, e, f
        //    g, h, i

        return matrix[0]*(matrix[4]*matrix[8]-matrix[5]*matrix[7])
             - matrix[1]*(matrix[3]*matrix[8]-matrix[5]*matrix[6])
             + matrix[2]*(matrix[3]*matrix[7]-matrix[4]*matrix[6]);
    }

    bool Matrix3::inverse()
    {
        GLfloat a = (*this)(0, 0), b = (*this)(0, 1), c = (*this)(0, 2),
                d = (*this)(1, 0), e = (*this)(1, 1), f = (*this)(1, 2),
                g = (*this)(2, 0), h = (*this)(2, 1), i = (*this)(2, 2);

        GLfloat A =  (e*i-f*h), B = -(d*i-f*g), C =  (d*h-e*g),
                D = -(b*i-c*h), E =  (a*i-c*g), F = -(a*h-b*g),
                G =  (b*f-c*e), H = -(a*f-c*d), I =  (a*e-b*d);

        GLfloat det = determinant();

        if(det == 0.f)
            return false;

        det = 1.f/det;

        *this =
        {
            det*A, det*D, det*G,
            det*B, det*E, det*H,
            det*C, det*F, det*I
        };

        return true;
    }

    Matrix2 Matrix3::toMatrix2()
    {
        return Matrix2
        {
            matrix[0], matrix[1],
            matrix[3], matrix[4]
        };
    }

    Matrix4 Matrix3::toMatrix4()
    {
        return Matrix4
        {
            matrix[0], matrix[1], matrix[2], 0,
            matrix[3], matrix[4], matrix[5], 0,
            matrix[6], matrix[7], matrix[8], 0,
                0,        0,         0,      0
        };
    }

    Matrix3 Matrix3::operator*(const Matrix3 &other)
    {
        // column0 matrix[0], matrix[1], matrix[2]
        // column1 matrix[3], matrix[4], matrix[5]
        // column2 matrix[6], matrix[7], matrix[8]

        // row0 other.matrix[0], other.matrix[3], other.matrix[6]
        // row1 other.matrix[1], other.matrix[4], other.matrix[7]
        // row2 other.matrix[2], other.matrix[5], other.matrix[8]

        //{
        //    column0*row0, column0*row1, column0*row2,
        //    column1*row0, column1*row1, column1*row2,
        //    column2*row0, column2*row1, column2*row2,
        //}

        return
        {
            matrix[0] * other.matrix[0] + matrix[1] * other.matrix[3] + matrix[2] * other.matrix[6],
            matrix[0] * other.matrix[1] + matrix[1] * other.matrix[4] + matrix[2] * other.matrix[7],
            matrix[0] * other.matrix[2] + matrix[1] * other.matrix[5] + matrix[2] * other.matrix[8],

            matrix[3] * other.matrix[0] + matrix[4] * other.matrix[3] + matrix[5] * other.matrix[6],
            matrix[3] * other.matrix[1] + matrix[4] * other.matrix[4] + matrix[5] * other.matrix[7],
            matrix[3] * other.matrix[2] + matrix[4] * other.matrix[5] + matrix[5] * other.matrix[8],

            matrix[6] * other.matrix[0] + matrix[7] * other.matrix[3] + matrix[8] * other.matrix[6],
            matrix[6] * other.matrix[1] + matrix[7] * other.matrix[4] + matrix[8] * other.matrix[7],
            matrix[6] * other.matrix[2] + matrix[7] * other.matrix[5] + matrix[8] * other.matrix[8]
        };
    }

    Vec3 Matrix3::operator*(const Vec3 &v)
    {
        return Vec3(matrix[0] * v.getX() + matrix[1] * v.getY() + matrix[2] * v.getZ(),
                    matrix[3] * v.getX() + matrix[4] * v.getY() + matrix[5] * v.getZ(),
                    matrix[6] * v.getX() + matrix[7] * v.getY() + matrix[8] * v.getZ());
    }

    Matrix3::GLfloat &Matrix3::operator()(int y, int x)
    {
        return matrix[y * 3 + x];
    }

    Matrix3::GLfloat Matrix3::operator()(int y, int x) const
    {
        return matrix[y * 3 + x];
    }
}
