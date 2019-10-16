#ifndef QUATERNION_H
#define QUATERNION_H

#include "math_constants.h"
#include "mat3.h"
#include "mat4.h"
#include "gltypes.h"

namespace gsl
{

    class Quaternion
    {
        public:
            Quaternion();

            //From Euler angles
            Quaternion(GLdouble roll, GLdouble pitch, GLdouble yaw);
            Quaternion(const Vec3 &rollpitchyaw);
            //From quaternion
            Quaternion(GLdouble ox, GLdouble oy, GLdouble oz, GLdouble ow);
            //From angle-axis
            Quaternion(GLdouble angle_deg, Vec3 axis);

            //Multiply Quaternion with Quaternion to Add the total rotations
            Quaternion operator*(const Quaternion &q);
            GLdouble& operator[](const GLint index);

            //Operator overloading as friend function
            friend std::ostream& operator<<(std::ostream &output, const Quaternion &rhs)
            {
                output << "X = " << rhs.x << ", Y = " << rhs.y <<
                          ", Z = " << rhs.z << ", W = " << rhs.w;
                return output;
            }

            void normalize();
            GLdouble magnitude();

            Mat4 toRotationMatrix4();
            Mat3 toRotationMatrix3();

            void toEulerAngles(GLdouble &roll, GLdouble &pitch, GLdouble &yaw);
            Vec3 toEulerAngles();

        private:
            GLdouble x{0}, y{0}, z{0}, w{1};
    };

} //namespace

#endif // QUATERNION_H
