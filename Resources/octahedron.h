#ifndef OCTAHEDRON_H
#define OCTAHEDRON_H

#include "meshbase.h"
#include "vec3.h"
#include "gltypes.h"

class Octahedron : public MeshBase
{
    public:
        Octahedron(GLint n = 0);
        GLint getRecursions() const {return mRecursions;}

    private:
        void makeTriangle(const gsl::Vec3 &v1, const gsl::Vec3 &v2, const gsl::Vec3 &v3);
        void subDivide(const gsl::Vec3 &a, const gsl::Vec3 &b, const gsl::Vec3 &c, GLint n);
        void makeUnitOctahedron();

        GLint mRecursions;
        GLint mIndex;               //Used in recursion to build vertices
};

#endif // OCTAHEDRON_H
