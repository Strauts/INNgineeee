#include "matrix4.h"
#include "quaternion.h"
#include "math_constants.h"
#include "matrix2.h"
#include "matrix3.h"

#include <cmath>
#include <utility>

namespace gsl
{
    Matrix4::Matrix4(bool isIdentity)
    {
        if(isIdentity)
        {
            identity();
        }
        else
        {
            for(int i = 0; i < 16; i++)
                matrix[i] = 0.f;
        }
    }

    Matrix4::Matrix4(std::initializer_list<GLfloat> values)
    {
        // initialiser matrisen klassen på samme måte som en 2d-array
        int i = 0;
        for(auto value : values)
            matrix[i++] = value;
    }

    Matrix4 Matrix4::identity()
    {
        setToIdentity();

        return *this;
    }

    void Matrix4::setToIdentity()
    {
        *this =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
    }

    void Matrix4::translateX(GLfloat x)
    {
        translate(x, 0.f, 0.f);
    }

    void Matrix4::translateY(GLfloat y)
    {
        translate(0.f, y, 0.f);
    }

    void Matrix4::translateZ(GLfloat z)
    {
        translate(0.f, 0.f, z);
    }

    void Matrix4::rotateX(GLfloat degrees)
    {
        GLfloat rad = (degrees * PI)/180;

        Matrix4 temp =
        {
            1.f,   0.f,       0.f,    0.f,
            0.f, std::cos(rad),  std::sin(rad), 0.f,
            0.f, -std::sin(rad), std::cos(rad), 0.f,
            0.f,   0.f,       0.f,    1.f
        };

        *this = (*this)*temp;
    }

    void Matrix4::rotateY(GLfloat degrees)
    {
        GLfloat rad = (degrees * PI)/180;

        Matrix4 temp =
        {
            std::cos(rad), 0.f, -std::sin(rad), 0.f,
                0.f,       1.f,      0.f,       0.f,
            std::sin(rad), 0.f,  std::cos(rad), 0.f,
                0.f,       0.f,      0.f,       1.f
        };

        *this = (*this)*temp;
    }

    void Matrix4::rotateZ(GLfloat degrees)
    {
        GLfloat rad = (degrees * PI)/180;

        Matrix4 temp =
        {
             std::cos(rad),  std::sin(rad), 0.f, 0.f,
            -std::sin(rad),  std::cos(rad), 0.f, 0.f,
                 0.f,            0.f,       1.f, 0.f,
                 0.f,            0.f,       0.f, 1.f
        };

        *this = (*this)*temp;
    }

    void Matrix4::qRotateX(GLfloat degrees)
    {
        rotate(degrees, Vec3(1, 0, 0));
    }

    void gsl::Matrix4::qRotateY(GLfloat degrees)
    {
        rotate(degrees, Vec3(0, 1, 0));
    }

    void gsl::Matrix4::qRotateZ(GLfloat degrees)
    {
        rotate(degrees, Vec3(0, 0, 1));
    }

    void Matrix4::rotate(GLfloat angle, const Vec3 &vector)
    {
        //should multiply this matrix by another that rotates coordinates through angle degrees about vector.
        Quaternion q(angle, vector);

        *this = (*this)*q.toRotationMatrix4();
    }

    void Matrix4::rotate(GLfloat angle, GLfloat xIn, GLfloat yIn, GLfloat zIn)
    {
        //should multiply this matrix by another that rotates coordinates through angle degrees about vector.
        Quaternion q(angle, Vec3(xIn, yIn, zIn));

        *this = (*this)*q.toRotationMatrix4();
    }

    void Matrix4::scale(GLfloat uniformScale)
    {
        scale(uniformScale, uniformScale, uniformScale);
    }

    void Matrix4::scale(GLfloat scaleX, GLfloat scaleY, GLfloat scaleZ)
    {
        Matrix4 temp =
        {
            scaleX,  0.f,    0.f,   0.f,
             0.f,   scaleY,  0.f,   0.f,
             0.f,    0.f,   scaleZ, 0.f,
             0.f,    0.f,    0.f,   1.f
        };

        *this = (*this)*temp;
    }

    GLfloat *Matrix4::constData()
    {
        return &matrix[0];
    }

    void Matrix4::transpose()
    {
        std::swap(matrix[1], matrix[4]);
        std::swap(matrix[2], matrix[8]);
        std::swap(matrix[3], matrix[12]);

        std::swap(matrix[6], matrix[9]);
        std::swap(matrix[7], matrix[13]);
        std::swap(matrix[11], matrix[14]);
    }

    void Matrix4::ortho(GLfloat l, GLfloat r, GLfloat b, GLfloat t, GLfloat nearPlane, GLfloat farPlane)
    {
        *this =
        {
            2/(r-l), 0.f, 0.f, 0.f,
            0.f, 2/(t-b), 0.f, 0.f,
            0.f, 0.f, -2/(farPlane-nearPlane), 0.f,
            -(r+l)/(r-l), -(t+b)/(t-b), -(farPlane+nearPlane)/(farPlane-nearPlane), 1.f
        };
    }

    void Matrix4::perspective(GLfloat verticalAngle, GLfloat aspectRatio, GLfloat nearPlane, GLfloat farPlane)
    {
        //checking numbers for no division on zero:
        if (verticalAngle <= 0.f)
            verticalAngle = 30.f;
        if (aspectRatio <= 0.f)
            aspectRatio = 1.3f;
        if (farPlane - nearPlane == 0.f)
        {
            nearPlane = 1.f;
            farPlane = 100.f;
        }

        // finn right, og utled resten fra dette
        float scale = tan(verticalAngle * PI / 360) * nearPlane;
        float r = aspectRatio * scale;
        float t = scale;

        // Lag perspektiv-frustrum
        *this =
        {
            nearPlane/r, 0.f, 0.f, 0.f,
            0.f, nearPlane/t, 0.f, 0.f,
            0.f, 0.f, -(farPlane+nearPlane)/(farPlane-nearPlane), -2*farPlane*nearPlane/(farPlane-nearPlane),
            0.f, 0.f, -1.f, 0.f
        };
    }

    void Matrix4::lookAt(const Vec3 &eye, const Vec3 &center, const Vec3 &up_axis)
    {
        Vec3 f = center-eye;
        f.normalize();
        Vec3 s = f^up_axis; // kryssprodukt
        s.normalize();
        Vec3 u = s^f;

        *this =
        {
             s.getX(),  s.getY(),  s.getZ(), -(s*eye),
             u.getX(),  u.getY(),  u.getZ(), -(u*eye),
            -f.getX(), -f.getY(), -f.getZ(), f*eye,
            0.f, 0.f, 0.f, 1.f
        };

    }

    void Matrix4::translate(GLfloat x, GLfloat y, GLfloat z)
    {
        Matrix4 mat =
        {
            1.f, 0.f, 0.f, x,
            0.f, 1.f, 0.f, y,
            0.f, 0.f, 1.f, z,
            0.f, 0.f, 0.f, 1.f
        };

        *this = (*this)*mat;
    }

    void Matrix4::translate(Vec3 positionIn)
    {
        Matrix4 mat =
        {
            1.f, 0.f, 0.f, positionIn.getX(),
            0.f, 1.f, 0.f, positionIn.getY(),
            0.f, 0.f, 1.f, positionIn.getZ(),
            0.f, 0.f, 0.f, 1.f
        };

        *this = (*this)*mat;
    }

    Matrix2 Matrix4::toMatrix2()
    {
        return
        {
            matrix[0], matrix[1],
            matrix[4], matrix[5]
        };
    }

    gsl::Matrix3 gsl::Matrix4::toMatrix3()
    {
        return
        {
            matrix[0], matrix[1], matrix[2],
            matrix[4], matrix[5], matrix[6],
            matrix[8], matrix[9], matrix[10]
        };
    }

    GLfloat &Matrix4::operator()(const int &y, const int &x)
    {
        return matrix[y * 4 + x];
    }

    GLfloat Matrix4::operator()(const int &y, const int &x) const
    {
        return matrix[y * 4 + x];
    }

    Matrix4 Matrix4::operator*(const Matrix4 &other)
    {
        // column0 matrix[0], matrix[1],  matrix[2],  matrix[3]
        // column1 matrix[4], matrix[5],  matrix[6],  matrix[7]
        // column2 matrix[8], matrix[9],  matrix[10], matrix[11]
        // column3 matrix[12, matrix[13], matrix[14], matrix[15]

        // row0 other.matrix[0], other.matrix[4], other.matrix[8],  other.matrix[12]
        // row1 other.matrix[1], other.matrix[5], other.matrix[9],  other.matrix[13]
        // row2 other.matrix[2], other.matrix[6], other.matrix[10], other.matrix[14]
        // row3 other.matrix[3], other.matrix[7], other.matrix[11], other.matrix[15]

        //{
        //  column0*row0, column0*row1, column0*row2, column0*row3
        //  column1*row0, column1*row1, column1*row2, column1*row3
        //  column2*row0, column2*row1, column2*row2, column2*row3
        //  column3*row0, column3*row1, column3*row2, column3*row3
        //}

        return
        {
            matrix[0]  * other.matrix[0] + matrix[1]  * other.matrix[4] + matrix[2]  * other.matrix[8]  + matrix[3]  * other.matrix[12],
            matrix[0]  * other.matrix[1] + matrix[1]  * other.matrix[5] + matrix[2]  * other.matrix[9]  + matrix[3]  * other.matrix[13],
            matrix[0]  * other.matrix[2] + matrix[1]  * other.matrix[6] + matrix[2]  * other.matrix[10] + matrix[3]  * other.matrix[14],
            matrix[0]  * other.matrix[3] + matrix[1]  * other.matrix[7] + matrix[2]  * other.matrix[11] + matrix[3]  * other.matrix[15],

            matrix[4]  * other.matrix[0] + matrix[5]  * other.matrix[4] + matrix[6]  * other.matrix[8]  + matrix[7]  * other.matrix[12],
            matrix[4]  * other.matrix[1] + matrix[5]  * other.matrix[5] + matrix[6]  * other.matrix[9]  + matrix[7]  * other.matrix[13],
            matrix[4]  * other.matrix[2] + matrix[5]  * other.matrix[6] + matrix[6]  * other.matrix[10] + matrix[7]  * other.matrix[14],
            matrix[4]  * other.matrix[3] + matrix[5]  * other.matrix[7] + matrix[6]  * other.matrix[11] + matrix[7]  * other.matrix[15],

            matrix[8]  * other.matrix[0] + matrix[9]  * other.matrix[4] + matrix[10] * other.matrix[8]  + matrix[11] * other.matrix[12],
            matrix[8]  * other.matrix[1] + matrix[9]  * other.matrix[5] + matrix[10] * other.matrix[9]  + matrix[11] * other.matrix[13],
            matrix[8]  * other.matrix[2] + matrix[9]  * other.matrix[6] + matrix[10] * other.matrix[10] + matrix[11] * other.matrix[14],
            matrix[8]  * other.matrix[3] + matrix[9]  * other.matrix[7] + matrix[10] * other.matrix[11] + matrix[11] * other.matrix[15],

            matrix[12] * other.matrix[0] + matrix[13] * other.matrix[4] + matrix[14] * other.matrix[8]  + matrix[15] * other.matrix[12],
            matrix[12] * other.matrix[1] + matrix[13] * other.matrix[5] + matrix[14] * other.matrix[9]  + matrix[15] * other.matrix[13],
            matrix[12] * other.matrix[2] + matrix[13] * other.matrix[6] + matrix[14] * other.matrix[10] + matrix[15] * other.matrix[14],
            matrix[12] * other.matrix[3] + matrix[13] * other.matrix[7] + matrix[14] * other.matrix[11] + matrix[15] * other.matrix[15]
        };
    }

    Vec4 Matrix4::operator*(const Vec4 &v)
    {
        return Vec4(matrix[0]*v.getX()  + matrix[1]*v.getY()  + matrix[2]*v.getZ()  + matrix[3] *v.getW(),
                    matrix[4]*v.getX()  + matrix[5]*v.getY()  + matrix[6]*v.getZ()  + matrix[7] *v.getW(),
                    matrix[8]*v.getX()  + matrix[9]*v.getY()  + matrix[10]*v.getZ() + matrix[11] *v.getW(),
                    matrix[12]*v.getX() + matrix[13]*v.getY() + matrix[14]*v.getZ() + matrix[15] *v.getW());
    }



    bool Matrix4::inverse()
    {
        GLfloat inv[16], det;
        GLfloat invOut[16];

        int i;

        inv[0] = matrix[5]  * matrix[10] * matrix[15] -
                matrix[5]  * matrix[11] * matrix[14] -
                matrix[9]  * matrix[6]  * matrix[15] +
                matrix[9]  * matrix[7]  * matrix[14] +
                matrix[13] * matrix[6]  * matrix[11] -
                matrix[13] * matrix[7]  * matrix[10];

        inv[4] = -matrix[4]  * matrix[10] * matrix[15] +
                matrix[4]  * matrix[11] * matrix[14] +
                matrix[8]  * matrix[6]  * matrix[15] -
                matrix[8]  * matrix[7]  * matrix[14] -
                matrix[12] * matrix[6]  * matrix[11] +
                matrix[12] * matrix[7]  * matrix[10];

        inv[8] = matrix[4]  * matrix[9] * matrix[15] -
                matrix[4]  * matrix[11] * matrix[13] -
                matrix[8]  * matrix[5] * matrix[15] +
                matrix[8]  * matrix[7] * matrix[13] +
                matrix[12] * matrix[5] * matrix[11] -
                matrix[12] * matrix[7] * matrix[9];

        inv[12] = -matrix[4]  * matrix[9] * matrix[14] +
                matrix[4]  * matrix[10] * matrix[13] +
                matrix[8]  * matrix[5] * matrix[14] -
                matrix[8]  * matrix[6] * matrix[13] -
                matrix[12] * matrix[5] * matrix[10] +
                matrix[12] * matrix[6] * matrix[9];

        inv[1] = -matrix[1]  * matrix[10] * matrix[15] +
                matrix[1]  * matrix[11] * matrix[14] +
                matrix[9]  * matrix[2] * matrix[15] -
                matrix[9]  * matrix[3] * matrix[14] -
                matrix[13] * matrix[2] * matrix[11] +
                matrix[13] * matrix[3] * matrix[10];

        inv[5] = matrix[0]  * matrix[10] * matrix[15] -
                matrix[0]  * matrix[11] * matrix[14] -
                matrix[8]  * matrix[2] * matrix[15] +
                matrix[8]  * matrix[3] * matrix[14] +
                matrix[12] * matrix[2] * matrix[11] -
                matrix[12] * matrix[3] * matrix[10];

        inv[9] = -matrix[0]  * matrix[9] * matrix[15] +
                matrix[0]  * matrix[11] * matrix[13] +
                matrix[8]  * matrix[1] * matrix[15] -
                matrix[8]  * matrix[3] * matrix[13] -
                matrix[12] * matrix[1] * matrix[11] +
                matrix[12] * matrix[3] * matrix[9];

        inv[13] = matrix[0]  * matrix[9] * matrix[14] -
                matrix[0]  * matrix[10] * matrix[13] -
                matrix[8]  * matrix[1] * matrix[14] +
                matrix[8]  * matrix[2] * matrix[13] +
                matrix[12] * matrix[1] * matrix[10] -
                matrix[12] * matrix[2] * matrix[9];

        inv[2] = matrix[1]  * matrix[6] * matrix[15] -
                matrix[1]  * matrix[7] * matrix[14] -
                matrix[5]  * matrix[2] * matrix[15] +
                matrix[5]  * matrix[3] * matrix[14] +
                matrix[13] * matrix[2] * matrix[7] -
                matrix[13] * matrix[3] * matrix[6];

        inv[6] = -matrix[0]  * matrix[6] * matrix[15] +
                matrix[0]  * matrix[7] * matrix[14] +
                matrix[4]  * matrix[2] * matrix[15] -
                matrix[4]  * matrix[3] * matrix[14] -
                matrix[12] * matrix[2] * matrix[7] +
                matrix[12] * matrix[3] * matrix[6];

        inv[10] = matrix[0]  * matrix[5] * matrix[15] -
                matrix[0]  * matrix[7] * matrix[13] -
                matrix[4]  * matrix[1] * matrix[15] +
                matrix[4]  * matrix[3] * matrix[13] +
                matrix[12] * matrix[1] * matrix[7] -
                matrix[12] * matrix[3] * matrix[5];

        inv[14] = -matrix[0]  * matrix[5] * matrix[14] +
                matrix[0]  * matrix[6] * matrix[13] +
                matrix[4]  * matrix[1] * matrix[14] -
                matrix[4]  * matrix[2] * matrix[13] -
                matrix[12] * matrix[1] * matrix[6] +
                matrix[12] * matrix[2] * matrix[5];

        inv[3] = -matrix[1] * matrix[6] * matrix[11] +
                matrix[1] * matrix[7] * matrix[10] +
                matrix[5] * matrix[2] * matrix[11] -
                matrix[5] * matrix[3] * matrix[10] -
                matrix[9] * matrix[2] * matrix[7] +
                matrix[9] * matrix[3] * matrix[6];

        inv[7] = matrix[0] * matrix[6] * matrix[11] -
                matrix[0] * matrix[7] * matrix[10] -
                matrix[4] * matrix[2] * matrix[11] +
                matrix[4] * matrix[3] * matrix[10] +
                matrix[8] * matrix[2] * matrix[7] -
                matrix[8] * matrix[3] * matrix[6];

        inv[11] = -matrix[0] * matrix[5] * matrix[11] +
                matrix[0] * matrix[7] * matrix[9] +
                matrix[4] * matrix[1] * matrix[11] -
                matrix[4] * matrix[3] * matrix[9] -
                matrix[8] * matrix[1] * matrix[7] +
                matrix[8] * matrix[3] * matrix[5];

        inv[15] = matrix[0] * matrix[5] * matrix[10] -
                matrix[0] * matrix[6] * matrix[9] -
                matrix[4] * matrix[1] * matrix[10] +
                matrix[4] * matrix[2] * matrix[9] +
                matrix[8] * matrix[1] * matrix[6] -
                matrix[8] * matrix[2] * matrix[5];

        det = matrix[0] * inv[0] + matrix[1] * inv[4] + matrix[2] * inv[8] + matrix[3] * inv[12];

        if (det == 0.f)
            return false;

        det = 1.f / det;

        for (i = 0; i < 16; i++)
            invOut[i] = inv[i] * det;

        memcpy(matrix, invOut, 16*sizeof(GLfloat));

        return true;
    }
} //namespace
