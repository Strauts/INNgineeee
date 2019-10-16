#include "mat3.h"
#include "mat2.h"
#include "mat4.h"

namespace gsl
{

    Mat3::Mat3(bool isIdentity)
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


    Mat3::Mat3(std::initializer_list<GLfloat> values)
    {
        int i = 0;
        for(auto value : values)
            matrix[i++] = value;
    }


    Mat3 Mat3::identity()
    {
        setToIdentity();

        return *this;
    }


    void Mat3::setToIdentity()
    {
        *this =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };
    }


    GLfloat Mat3::determinant()
    {
        //det = a(ei − fh) − b(di − fg) + c(dh − eg)
        //where
        //    a, b, c,
        //    d, e, f
        //    g, h, i

        return matrix[0]*(matrix[4]*matrix[8]-matrix[5]*matrix[7])
             - matrix[1]*(matrix[3]*matrix[8]-matrix[5]*matrix[6])
             + matrix[2]*(matrix[3]*matrix[7]-matrix[4]*matrix[6]);
    }


    bool Mat3::inverse()
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


    void Mat3::transpose()
    {
        std::swap(matrix[1], matrix[4]);
        std::swap(matrix[2], matrix[6]);
        std::swap(matrix[5], matrix[7]);
    }


    Mat2 Mat3::toMatrix2()
    {
        return Mat2
        {
            matrix[0], matrix[1],
            matrix[3], matrix[4]
        };
    }


    Mat4 Mat3::toMatrix4()
    {
        return Mat4
        {
            matrix[0], matrix[1], matrix[2], 0,
            matrix[3], matrix[4], matrix[5], 0,
            matrix[6], matrix[7], matrix[8], 0,
                0,        0,         0,      0
        };
    }


    Mat3 Mat3::operator*(const Mat3 &other)
    {
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


    Vec3 Mat3::operator*(const Vec3 &v)
    {
        return Vec3(matrix[0] * v.getX() + matrix[1] * v.getY() + matrix[2] * v.getZ(),
                    matrix[3] * v.getX() + matrix[4] * v.getY() + matrix[5] * v.getZ(),
                    matrix[6] * v.getX() + matrix[7] * v.getY() + matrix[8] * v.getZ());
    }


    GLfloat& Mat3::operator()(int y, int x)
    {
        return matrix[y * 3 + x];
    }


    GLfloat Mat3::operator()(int y, int x) const
    {
        return matrix[y * 3 + x];
    }

} //namespace
