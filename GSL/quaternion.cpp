#include "quaternion.h"
#include "vec3.h"
#include "gsl_math.h"
#include <cmath>

namespace gsl
{

    Quaternion::Quaternion()
    {

    }


    Quaternion::Quaternion(GLdouble roll, GLdouble pitch, GLdouble yaw)
    {
        //Abbreviations for the various angular functions
        GLdouble cy = std::cos(deg2rad(yaw) * 0.5);
        GLdouble sy = std::sin(deg2rad(yaw) * 0.5);
        GLdouble cr = std::cos(deg2rad(roll) * 0.5);
        GLdouble sr = std::sin(deg2rad(roll) * 0.5);
        GLdouble cp = std::cos(deg2rad(pitch) * 0.5);
        GLdouble sp = std::sin(deg2rad(pitch) * 0.5);

        w = cy * cr * cp + sy * sr * sp;
        x = cy * sr * cp - sy * cr * sp;
        y = cy * cr * sp + sy * sr * cp;
        z = sy * cr * cp - cy * sr * sp;

        //normalize();
    }


    Quaternion::Quaternion(const Vec3 &rollpitchyaw)
    {
        //Abbreviations for the various angular functions
        GLdouble cy = std::cos(deg2rad(static_cast<GLdouble>(rollpitchyaw.getZ())) * 0.5);
        GLdouble sy = std::sin(deg2rad(static_cast<GLdouble>(rollpitchyaw.getZ())) * 0.5);
        GLdouble cr = std::cos(deg2rad(static_cast<GLdouble>(rollpitchyaw.getX())) * 0.5);
        GLdouble sr = std::sin(deg2rad(static_cast<GLdouble>(rollpitchyaw.getX())) * 0.5);
        GLdouble cp = std::cos(deg2rad(static_cast<GLdouble>(rollpitchyaw.getY())) * 0.5);
        GLdouble sp = std::sin(deg2rad(static_cast<GLdouble>(rollpitchyaw.getY())) * 0.5);

        w = cy * cr * cp + sy * sr * sp;
        x = cy * sr * cp - sy * cr * sp;
        y = cy * cr * sp + sy * sr * cp;
        z = sy * cr * cp - cy * sr * sp;

        //normalize();
    }


    Quaternion::Quaternion(GLdouble ox, GLdouble oy, GLdouble oz, GLdouble ow) : x{ox}, y{oy}, z{oz}, w{ow}
    {
        //normalize();
    }


    Quaternion::Quaternion(GLdouble angle_deg, Vec3 axis)
    {
        angle_deg = deg2rad(angle_deg);
        w = std::cos(angle_deg/2.0);
        x = static_cast<GLdouble>(axis.getX()) * std::sin(angle_deg/2.0);
        y = static_cast<GLdouble>(axis.getY()) * std::sin(angle_deg/2.0);
        z = static_cast<GLdouble>(axis.getZ()) * std::sin(angle_deg/2.0);

        //normalize();
    }


    Quaternion Quaternion::operator*(const Quaternion &q)
    {
        GLdouble xx =  (x * q.w) + (y * q.z) - (z * q.y) + (w * q.x);
        GLdouble yy = -(x * q.z) + (y * q.w) + (z * q.x) + (w * q.y);
        GLdouble zz =  (x * q.y) - (y * q.x) + (z * q.w) + (w * q.z);
        GLdouble ww = -(x * q.x) - (y * q.y) - (z * q.z) + (w * q.w);

        return Quaternion(xx, yy, zz, ww);
    }


    GLdouble& Quaternion::operator[](const GLint index)
    {
        switch (index)
        {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            case 3:
                return w;
        }

        return w;
    }


    void Quaternion::normalize()
    {
        GLdouble mag = magnitude();

        if (mag != 0.0)
        {
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }


    GLdouble Quaternion::magnitude()
    {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }


    Mat3 Quaternion::toRotationMatrix3()
    {
        //normalize();

        Mat3 m
        {
            static_cast<GLfloat>(1.0 - (2.0*y*y) - (2.0*z*z)),  static_cast<GLfloat>((2.0*x*y) - (2.0*z*w)),        static_cast<GLfloat>((2.0*x*z) + (2.0*y*w)),
            static_cast<GLfloat>((2.0*x*y) + (2.0*z*w)),        static_cast<GLfloat>(1.0 - (2.0*x*x) - (2.0*z*z)),  static_cast<GLfloat>((2.0*y*z) - (2.0*x*w)),
            static_cast<GLfloat>((2.0*x*z) - (2.0*y*w)),        static_cast<GLfloat>((2.0*y*z) + (2.0*x*w)),        static_cast<GLfloat>(1.0 - (2.0*x*x) - (2.0*y*y))
        };

        //m.transpose();

        return m;
    }


    Mat4 Quaternion::toRotationMatrix4()
    {
        //normalize();

        Mat4 m
        {
                    static_cast<GLfloat>(1.0 - (2.0*y*y) - (2.0*z*z)),  static_cast<GLfloat>((2.0*x*y) - (2.0*z*w)),        static_cast<GLfloat>((2.0*x*z) + (2.0*y*w)),        0.f,
                    static_cast<GLfloat>((2.0*x*y) + (2.0*z*w)),        static_cast<GLfloat>(1.0 - (2.0*x*x) - (2.0*z*z)),  static_cast<GLfloat>((2.0*y*z) - (2.0*x*w)),        0.f,
                    static_cast<GLfloat>((2.0*x*z) - (2.0*y*w)),        static_cast<GLfloat>((2.0*y*z) + (2.0*x*w)),        static_cast<GLfloat>(1.0 - (2.0*x*x) - (2.0*y*y)),  0.f,
                                        0.f,                                            0.f,                                                    0.f,                            1.f
        };

        m.transpose();

        return m;
    }


    void Quaternion::toEulerAngles(GLdouble &roll, GLdouble &pitch, GLdouble &yaw)
    {
        //Roll (x-axis rotation)
        GLdouble sinr = 2.0 * ((w * x) + (y * z));
        GLdouble cosr = 1.0 - (2.0 * ((x * x) + (y * y)));
        roll = rad2deg(std::atan2(sinr, cosr));

        //Pitch (y-axis rotation)
        GLdouble sinp = 2.0 * ((w * y) - (z * x));
        if (std::fabs(sinp) >= 1.0)
            pitch = rad2deg(std::copysign(PI / 2.0, sinp)); //Use 90 degrees if out of range
        else
            pitch = rad2deg(std::asin(sinp));

        //Yaw (z-axis rotation)
        GLdouble siny = 2.0 * ((w * z) + (x * y));
        GLdouble cosy = 1.0 - (2.0 * ((y * y) + (z * z)));
        yaw = rad2deg(std::atan2(siny, cosy));
    }


    Vec3 Quaternion::toEulerAngles()
    {
        Vec3 ret;

        //Roll (x-axis rotation)
        GLdouble sinr = 2.0 * ((w * x) + (y * z));
        GLdouble cosr = 1.0 - (2.0 * ((x * x) + (y * y)));
        ret.setX(static_cast<GLfloat>(rad2deg(std::atan2(sinr, cosr))));

        //Pitch (y-axis rotation)
        GLdouble sinp = 2.0 * ((w * y) - (z * x));
        if (std::fabs(sinp) >= 1.0)
            ret.setY(static_cast<GLfloat>(rad2deg(std::copysign(PI / 2.0, sinp)))); //Use 90 degrees if out of range
        else
            ret.setY(static_cast<GLfloat>(rad2deg(std::asin(sinp))));

        //Yaw (z-axis rotation)
        GLdouble siny = 2.0 * ((w * z) + (x * y));
        GLdouble cosy = 1.0 - (2.0 * ((y * y) + (z * z)));
        ret.setZ(static_cast<GLfloat>(rad2deg(std::atan2(siny, cosy))));

        return ret;
    }

} //namespace
