#ifndef MATRIX2_H
#define MATRIX2_H

#include "vec2.h"
#include "vec3.h"
#include <utility>
#include <iomanip>

namespace gsl
{
    class Matrix3;
    class Matrix4;

    class Matrix2
    {
        typedef float GLfloat;

    public:
        Matrix2(bool isIdentity = false);
        Matrix2(std::initializer_list<GLfloat> values);

        Matrix2 identity();
        void setToIdentity();

        GLfloat determinant();
        bool inverse();

        void transpose();

        Matrix3 toMatrix3();
        Matrix4 toMatrix4();

        Matrix2 operator*(const Matrix2& other);
        Vec2 operator*(const Vec2& v);
        GLfloat &operator()(int y, int x);
        GLfloat operator()(int y, int x) const;

        friend std::ostream &operator<<(std::ostream &output, const Matrix2 &mIn)
        {
            output << std::setprecision(4) <<
                      "{" << mIn.matrix[0] << "\t, " << mIn.matrix[1] << "}\n" <<
                      "{" << mIn.matrix[2] << "\t, " << mIn.matrix[3] << "}\n";
            return output;
        }
    private:
        GLfloat matrix[4];
    };
}
#endif // MATRIX2_H
