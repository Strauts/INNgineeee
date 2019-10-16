#ifndef MATRIX3_H
#define MATRIX3_H

#include <utility>
#include <iomanip>

#include "vec3.h"

namespace gsl
{
    class Matrix2;
    class Matrix4;

    class Matrix3
    {
        typedef float GLfloat;

    public:
        Matrix3(bool isIdentity = false);
        Matrix3(std::initializer_list<float> values);

        Matrix3 identity();
        void setToIdentity();

        GLfloat determinant();

        bool inverse();

        void transpose();

        Matrix2 toMatrix2();
        Matrix4 toMatrix4();

        Matrix3 operator*(const Matrix3& other);
        Vec3 operator*(const Vec3& v);
        GLfloat &operator()(int y, int x);
        GLfloat operator()(int y, int x) const;
        friend std::ostream &operator<<(std::ostream &output, const Matrix3 &mIn)
        {
            output << std::setprecision(4) <<
                      "{" << mIn.matrix[0] << "\t, " << mIn.matrix[1] << "\t, " << mIn.matrix[2] << "}\n" <<
                      "{" << mIn.matrix[3] << "\t, " << mIn.matrix[4] << "\t, " << mIn.matrix[5] << "}\n" <<
                      "{" << mIn.matrix[6] << "\t, " << mIn.matrix[7] << "\t, " << mIn.matrix[8] << "}\n";
            return output;
        }
    private:
        GLfloat matrix[9];
    };
}
#endif // MATRIX3_H
