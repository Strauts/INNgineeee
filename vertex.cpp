#include "vertex.h"


Vertex::Vertex()
{
}

Vertex::Vertex(gsl::Vec3 a, gsl::Vec3 b , gsl::Vec2 c)
{
    mXYZ = a;
    mNormal = b;
    mST = c;
}

Vertex::~Vertex()
{
}

void Vertex::set_xyz(GLfloat *xyz)
{
    mXYZ.setX(xyz[0]);
    mXYZ.setY(xyz[1]);
    mXYZ.setZ(xyz[2]);
}

void Vertex::set_xyz(GLfloat x, GLfloat y, GLfloat z)
{
    mXYZ.setX(x);
    mXYZ.setY(y);
    mXYZ.setZ(z);
}

void Vertex::set_xyz(gsl::Vec3 xyz_in)
{
    mXYZ = xyz_in;
}

void Vertex::set_rgb(GLfloat *rgb)
{
    mNormal.setX(rgb[0]);
    mNormal.setY(rgb[1]);
    mNormal.setZ(rgb[2]);
}

void Vertex::set_rgb(GLfloat r, GLfloat g, GLfloat b)
{
    mNormal.setX(r);
    mNormal.setY(g);
    mNormal.setZ(b);
}

void Vertex::set_normal(GLfloat *normal)
{
    mNormal.setX(normal[0]);
    mNormal.setY(normal[1]);
    mNormal.setZ(normal[2]);
}

void Vertex::set_normal(GLfloat x, GLfloat y, GLfloat z)
{
    mNormal.setX(x); mNormal.setY(y); mNormal.setZ(z);
}

void Vertex::set_normal(gsl::Vec3 normal_in)
{
    mNormal = normal_in;
}

void Vertex::set_st(GLfloat *st)
{
    mST.setX(st[0]);
    mST.setY(st[1]);
}

void Vertex::set_st(GLfloat s, GLfloat t)
{
    mST.setX(s); mST.setY(t);
}

void Vertex::set_uv(GLfloat u, GLfloat v)
{
    mST.setX(u);
    mST.setY(v);
}

gsl::Vec3 Vertex::get_xyz()
{
    return mXYZ;
}

GLfloat Vertex::get_x()
{
    return mXYZ.getX();
}

GLfloat Vertex::get_y()
{
    return mXYZ.getY();
}

GLfloat Vertex::get_z()
{
    return mXYZ.getZ();
}

std::ostream& operator<<(std::ostream& os, const Vertex& v)
{
   os << "(" << v.mXYZ.getX() << ", " << v.mXYZ.getY() << ", " << v.mXYZ.getZ() << ") ";
   os << "(" << v.mNormal.getX() << ", " << v.mNormal.getY() << ", " << v.mNormal.getZ() << ") ";

   return os;
}
