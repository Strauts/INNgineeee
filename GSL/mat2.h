#ifndef MAT2_H
#define MAT2_H

#include "vec2.h"
#include "vec3.h"
#include "gltypes.h"
#include <utility>
#include <iomanip>

namespace gsl
{

    class Mat3;
    class Mat4;

    class Mat2
    {
        public:
            Mat2(bool isIdentity = false);
            Mat2(std::initializer_list<GLfloat> values);

            Mat2 identity();
            void setToIdentity();

            GLfloat determinant();
            bool inverse();

            void transpose();

            Mat3 toMatrix3();
            Mat4 toMatrix4();

            Mat2 operator*(const Mat2 &other);
            Vec2 operator*(const Vec2 &v);
            GLfloat& operator()(int y, int x);
            GLfloat operator()(int y, int x) const;

            friend std::ostream& operator<<(std::ostream &output, const Mat2 &mIn)
            {
                output << std::setprecision(4) <<
                          "{" << mIn.matrix[0] << "\t, " << mIn.matrix[1] << "}\n" <<
                          "{" << mIn.matrix[2] << "\t, " << mIn.matrix[3] << "}\n";
                return output;
            }

        private:
            GLfloat matrix[4];
    };

} //namespace
#endif // MAT2_H
