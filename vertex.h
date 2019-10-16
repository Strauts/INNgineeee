#ifndef VERTEX_H
#define VERTEX_H

#include "vec3.h"
#include "vec2.h"
#include "gltypes.h"
#include <iostream>
#include <fstream>

class Vertex
{
    public:
        Vertex();
        Vertex(gsl::Vec3 a, gsl::Vec3 b, gsl::Vec2 c);
        ~Vertex();
        void set_xyz(GLfloat *xyz);
        void set_xyz(GLfloat x, GLfloat y, GLfloat z);
        void set_xyz(gsl::Vec3 xyz_in);
        void set_rgb(GLfloat *rgb);
        void set_rgb(GLfloat r, GLfloat g, GLfloat b);
        void set_normal(GLfloat *normal);
        void set_normal(GLfloat x, GLfloat y, GLfloat z);
        void set_normal(gsl::Vec3 normal_in);
        void set_st(GLfloat *st);
        void set_st(GLfloat s, GLfloat t);
        void set_uv(GLfloat u, GLfloat v);
        gsl::Vec3 get_xyz();
        GLfloat get_x();
        GLfloat get_y();
        GLfloat get_z();

    private:
        gsl::Vec3 mXYZ;
        gsl::Vec3 mNormal;
        gsl::Vec2 mST;

        friend std::ostream& operator<<(std::ostream&, const Vertex&);
};

#endif // VERTEX_H
