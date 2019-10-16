#include "octahedron.h"
#include "vertex.h"
#include <cmath>


Octahedron::Octahedron(GLint n) : mRecursions{n}, mIndex{0}
{
    mDrawType = GL_TRIANGLES;

    mNumberOfVertices = static_cast<GLuint>(3 * 8 * std::pow(4, mRecursions));
    mVertices = new Vertex[mNumberOfVertices];
    makeUnitOctahedron();
}

void Octahedron::makeTriangle(const gsl::Vec3 &v1, const gsl::Vec3 &v2, const gsl::Vec3 &v3)
{
    mVertices[mIndex].set_xyz(v1.getX(), v1.getY(), v1.getZ());
    mVertices[mIndex].set_normal(v1.getX(), v1.getY(), v1.getZ());
    mVertices[mIndex].set_st(0.f, 0.f);
    ++mIndex;

    mVertices[mIndex].set_xyz(v2.getX(), v2.getY(), v2.getZ());
    mVertices[mIndex].set_normal(v2.getX(), v2.getY(), v2.getZ());
    mVertices[mIndex].set_st(1.f, 0.f);
    ++mIndex;

    mVertices[mIndex].set_xyz(v3.getX(), v3.getY(), v3.getZ());
    mVertices[mIndex].set_normal(v3.getX(), v3.getY(), v3.getZ());
    mVertices[mIndex].set_st(0.5f, 1.f);
    ++mIndex;
}

void Octahedron::subDivide(const gsl::Vec3 &a, const gsl::Vec3 &b, const gsl::Vec3 &c, GLint n)
{
    if (n > 0)
    {
        gsl::Vec3 v1 = a + b;
        v1.normalize();

        gsl::Vec3 v2 = a + c;
        v2.normalize();

        gsl::Vec3 v3 = c + b;
        v3.normalize();

        subDivide(a, v1, v2, n-1);
        subDivide(c, v2, v3, n-1);
        subDivide(b, v3, v1, n-1);
        subDivide(v3, v2, v1, n-1);
    }
    else
    {
        makeTriangle(a, b, c);
    }
}

void Octahedron::makeUnitOctahedron()
{
    gsl::Vec3 v0(0.f, 0.f, 1.f);
    gsl::Vec3 v1(1.f, 0.f, 0.f);
    gsl::Vec3 v2(0.f, 1.f, 0.f);
    gsl::Vec3 v3(-1.f, 0.f, 0.f);
    gsl::Vec3 v4(0.f, -1.f, 0.f);
    gsl::Vec3 v5(0.f, 0.f, -1.f);

    subDivide(v0, v1, v2, mRecursions);
    subDivide(v0, v2, v3, mRecursions);
    subDivide(v0, v3, v4, mRecursions);
    subDivide(v0, v4, v1, mRecursions);
    subDivide(v5, v2, v1, mRecursions);
    subDivide(v5, v3, v2, mRecursions);
    subDivide(v5, v4, v3, mRecursions);
    subDivide(v5, v1, v4, mRecursions);
}
