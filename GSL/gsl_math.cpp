#include "gsl_math.h"
#include <array>
#include <vector>

namespace gsl
{

    GLdouble rad2deg(GLdouble rad)
    {
        return rad * (180.0 / PI);
    }


    GLdouble deg2rad(GLdouble deg)
    {
        return deg * (PI / 180.0);
    }


    //Calculates the points on a bezier curve. Input t from 0 to 1
    Vec3 bezierCurve(std::vector<Vec3> points, GLfloat t, unsigned long long degree)
    {
        for (unsigned long long k = 0; k < degree; k++)
        {
            for (unsigned long long i = 0; i < degree - k - 1; i++)
            {
                points[i] = points[i] * (1.f - t) + points[i + 1] * t;
            }
        }

        return points[0];
    }


    //Calculates the points on a basis spline curve. Input t from 0 to 1.
    Vec3 bSpline(const std::vector<Vec3>& points, const std::vector<GLfloat> &t, GLfloat x, unsigned long long degree)
    {
        //CALCULATE VALID KNOT INTERVAL 'MY'
        unsigned long long my;
        for (my = 0; my < points.size(); my++)
        {
            if (x < t[my+1])
                break;
        }

        //CALCULATE BASIS FUNCTIONS
        std::vector<GLfloat> basis(t.size());
        for (unsigned long long j = 0; j <= degree; j++)
        {
            for (unsigned long long i = (my-j); i <= my; i++)
            {
                if (j == 0)
                {
                    basis[i] = 1.f;
                }
                else
                {
                    GLfloat left, right;

                    if (basis[i] != 0.f)
                        left = ((x - t[i]) / (t[i+j] - t[i])) * basis[i];
                    else
                        left = 0.f;

                    if (basis[i+1] != 0.f)
                        right = ((t[i+j+1] - x) / (t[i+j+1] - t[i+1])) * basis[i+1];
                    else
                        right = 0.f;

                    basis[i] = left + right;
                }
            }
        }

        //MULTIPLY POINTS WITH BASIS FUNCTIONS
        Vec3 result;
        for (unsigned long long i = 0; i < points.size(); i++)
        {
            result += points[i] * basis[i];
        }

        return result;
    }


    Vec3 up()
    {
        return Vec3{0.f, 1.f, 0.f};
    }


    Vec3 right()
    {
        return Vec3{1.f, 0.f, 0.f};
    }


    Vec3 forward()
    {
        return Vec3{0.f, 0.f, 1.f};
    }


    Vec3 one()
    {
        return Vec3{1.f, 1.f, 1.f};
    }


    Vec3 zero()
    {
        return Vec3{0.f, 0.f, 0.f};
    }


    GLfloat clamp(GLfloat x, GLfloat min, GLfloat max)
    {
        if (x < min)
            x = min;
        else if (x > max)
            x = max;

        return x;
    }


    Vec2 lerp2D(GLfloat time, Vec2 a, Vec2 b)
    {
        return (a * (1.f - time)) + (b * time);
    }


    Vec3 lerp3D(GLfloat time, Vec3 a, Vec3 b)
    {
        return (a * (1.f - time)) + (b * time);
    }


    Quaternion qLerp(GLfloat time, Quaternion start, Quaternion end)
    {
        GLfloat delta = 1.f - time;

        GLfloat x = delta*start[0] + time*end[0];
        GLfloat y = delta*start[1] + time*end[1];
        GLfloat z = delta*start[2] + time*end[2];
        GLfloat w = delta*start[3] + time*end[3];

        Quaternion result(x, y, z, w);
        result.normalize();

        return result;
    }


    Quaternion qLerp(GLfloat time, Vec3 start, Vec3 end)
    {
        Quaternion A(start), B(end);

        Quaternion result((1.f-time)*A[0] + time*B[0],
                          (1.f-time)*A[1] + time*B[1],
                          (1.f-time)*A[2] + time*B[2],
                          (1.f-time)*A[3] + time*B[3]);
        result.normalize();

        return result;
    }


    Vec3 qLerpEuler(GLfloat time, Quaternion start, Quaternion end)
    {
        Quaternion result((1.f-time)*start[0] + time*end[0],
                          (1.f-time)*start[1] + time*end[1],
                          (1.f-time)*start[2] + time*end[2],
                          (1.f-time)*start[3] + time*end[3]);
        result.normalize();

        return result.toEulerAngles();
    }


    Vec3 qLerpEuler(GLfloat time, Vec3 start, Vec3 end)
    {
        Quaternion A(start), B(end);

        Quaternion result((1.f-time)*A[0] + time*B[0],
                          (1.f-time)*A[1] + time*B[1],
                          (1.f-time)*A[2] + time*B[2],
                          (1.f-time)*A[3] + time*B[3]);
        result.normalize();

        return result.toEulerAngles();
    }

} //namespace
