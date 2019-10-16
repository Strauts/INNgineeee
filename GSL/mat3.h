#ifndef MAT3_H
#define MAT3_H

#include "vec3.h"
#include "gltypes.h"
#include <utility>
#include <iomanip>

namespace gsl
{

    class Mat2;
    class Mat4;

    class Mat3
    {
        public:
            Mat3(bool isIdentity = false);
            Mat3(std::initializer_list<GLfloat> values);

            Mat3 identity();
            void setToIdentity();

            GLfloat determinant();

            bool inverse();

            void transpose();

            Mat2 toMatrix2();
            Mat4 toMatrix4();

            Mat3 operator*(const Mat3 &other);
            Vec3 operator*(const Vec3 &v);
            GLfloat& operator()(int y, int x);
            GLfloat operator()(int y, int x) const;

            friend std::ostream& operator<<(std::ostream &output, const Mat3 &mIn)
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

} //namespace
#endif // MAT3_H
