#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include "vertex.h"
#include "octahedron.h"
#include "meshbase.h"

MeshBase* makeCube()
{
    MeshBase *temp = new MeshBase(GL_TRIANGLE_STRIP,
    {
        //Vertex data for face 0
        {gsl::Vec3(-1.f, -1.f, 1.f),    gsl::Vec3(0.f, 0.f, 1.0f),  gsl::Vec2(0.f,   0.f)},     //v0
        {gsl::Vec3( 1.f, -1.f, 1.f),    gsl::Vec3(0.f, 0.f, 1.0f),  gsl::Vec2(0.33f, 0.f)},     //v1
        {gsl::Vec3(-1.f,  1.f, 1.f),    gsl::Vec3(0.f, 0.f, 1.0f),  gsl::Vec2(0.f,   0.5f)},    //v2
        {gsl::Vec3( 1.f,  1.f, 1.f),    gsl::Vec3(0.f, 0.f, 1.0f),  gsl::Vec2(0.33f, 0.5f)},    //v3

        //Vertex data for face 1
        {gsl::Vec3(1.f, -1.f,  1.f),    gsl::Vec3(1.f, 0.f, 0.f),   gsl::Vec2(0.f,   0.5f)},    //v4
        {gsl::Vec3(1.f, -1.f, -1.f),    gsl::Vec3(1.f, 0.f, 0.f),   gsl::Vec2(0.33f, 0.5f)},    //v5
        {gsl::Vec3(1.f,  1.f,  1.f),    gsl::Vec3(1.f, 0.f, 0.f),   gsl::Vec2(0.f,   1.f)},     //v6
        {gsl::Vec3(1.f,  1.f, -1.f),    gsl::Vec3(1.f, 0.f, 0.f),   gsl::Vec2(0.33f, 1.f)},     //v7

        //Vertex data for face 2
        {gsl::Vec3( 1.f, -1.f, -1.f),   gsl::Vec3(0.f, 0.f, -1.f),  gsl::Vec2(0.66f, 0.5f)},    //v8
        {gsl::Vec3(-1.f, -1.f, -1.f),   gsl::Vec3(0.f, 0.f, -1.f),  gsl::Vec2(1.f,   0.5f)},    //v9
        {gsl::Vec3( 1.f,  1.f, -1.f),   gsl::Vec3(0.f, 0.f, -1.f),  gsl::Vec2(0.66f, 1.f)},     //v10
        {gsl::Vec3(-1.f,  1.f, -1.f),   gsl::Vec3(0.f, 0.f, -1.f),  gsl::Vec2(1.f,   1.f)},     //v11

        //Vertex data for face 3
        {gsl::Vec3(-1.f, -1.f, -1.f),   gsl::Vec3(-1.f, 0.f, 0.f),  gsl::Vec2(0.66f, 0.f)},     //v12
        {gsl::Vec3(-1.f, -1.f,  1.f),   gsl::Vec3(-1.f, 0.f, 0.f),  gsl::Vec2(1.f,   0.f)},     //v13
        {gsl::Vec3(-1.f,  1.f, -1.f),   gsl::Vec3(-1.f, 0.f, 0.f),  gsl::Vec2(0.66f, 0.5f)},    //v14
        {gsl::Vec3(-1.f,  1.f,  1.f),   gsl::Vec3(-1.f, 0.f, 0.f),  gsl::Vec2(1.f,   0.5f)},    //v15

        //Vertex data for face 4
        {gsl::Vec3(-1.f, -1.f, -1.f),   gsl::Vec3(0.f, -1.f, 0.f),  gsl::Vec2(0.33f, 0.f)},     //v16
        {gsl::Vec3( 1.f, -1.f, -1.f),   gsl::Vec3(0.f, -1.f, 0.f),  gsl::Vec2(0.66f, 0.f)},     //v17
        {gsl::Vec3(-1.f, -1.f,  1.f),   gsl::Vec3(0.f, -1.f, 0.f),  gsl::Vec2(0.33f, 0.5f)},    //v18
        {gsl::Vec3( 1.f, -1.f,  1.f),   gsl::Vec3(0.f, -1.f, 0.f),  gsl::Vec2(0.66f, 0.5f)},    //v19

        //Vertex data for face 5
        {gsl::Vec3(-1.f, 1.f,  1.f),    gsl::Vec3(0.f, 1.f, 0.f),   gsl::Vec2(0.33f, 0.5f)},    //v20
        {gsl::Vec3( 1.f, 1.f,  1.f),    gsl::Vec3(0.f, 1.f, 0.f),   gsl::Vec2(0.66f, 0.5f)},    //v21
        {gsl::Vec3(-1.f, 1.f, -1.f),    gsl::Vec3(0.f, 1.f, 0.f),   gsl::Vec2(0.33f, 1.f)},     //v22
        {gsl::Vec3( 1.f, 1.f, -1.f),    gsl::Vec3(0.f, 1.f, 0.f),   gsl::Vec2(0.66f, 1.f)}      //v23
    },
    {
        0,  1,  2,  3,  3,      //Face 0 - triangle strip (v0,  v1,  v2,  v3)
        4,  4,  5,  6,  7,  7,  //Face 1 - triangle strip (v4,  v5,  v6,  v7)
        8,  8,  9, 10, 11, 11,  //Face 2 - triangle strip (v8,  v9, v10,  v11)
        12, 12, 13, 14, 15, 15, //Face 3 - triangle strip (v12, v13, v14, v15)
        16, 16, 17, 18, 19, 19, //Face 4 - triangle strip (v16, v17, v18, v19)
        20, 20, 21, 22, 23      //Face 5 - triangle strip (v20, v21, v22, v23)
    });

    temp->mMeshName = "cube";

    //Bounding box:
    temp->mBoundingBoxLeftDownBack = gsl::Vec3(-1.f, -1.f, -1.f);
    temp->mBoundingBoxRightUpFront = gsl::Vec3(1.f, 1.f, 1.f);

    return temp;
}


MeshBase* makePlane()
{
    MeshBase *temp = new MeshBase(GL_TRIANGLES,
    {
        //Positions                     //Normals                   //UVs
        {gsl::Vec3{-0.5f,  0.f,  0.5f}, gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{0.f, 0.f}},   //0
        {gsl::Vec3{ 0.5f,  0.f,  0.5f}, gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{1.f, 0.f}},   //1
        {gsl::Vec3{ 0.5f,  0.f, -0.5f}, gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{1.f, 1.f}},   //2
        {gsl::Vec3{-0.5f,  0.f, -0.5f}, gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{0.f, 1.f}}    //3
    },
    {
        0, 1, 2,    //First triangle
        0, 2, 3     //Second triangle
    });

    temp->mMeshName = "plane";

    //Bounding box:
    temp->mBoundingBoxLeftDownBack = gsl::Vec3(-1.f, -1.f, -1.f);
    temp->mBoundingBoxRightUpFront = gsl::Vec3(1.f, 1.f, 1.f);

    return temp;
}


MeshBase* makeSphere(GLint recursions = 2)
{
    MeshBase *temp = new Octahedron(recursions);

    temp->mMeshName = "sphere";

    //Bounding box:
    temp->mBoundingBoxLeftDownBack = gsl::Vec3(-1.f, -1.f, -1.f);
    temp->mBoundingBoxRightUpFront = gsl::Vec3(1.f, 1.f, 1.f);

    return temp;
}


MeshBase* makeAxis()
{
    MeshBase *temp = new MeshBase(GL_LINES,
    {   //Positions                         //Colors                    //UVs
        {gsl::Vec3{-500.f, 0.f,   0.f},     gsl::Vec3{1.f, 0.f, 0.f},   gsl::Vec2{0.f, 0.f}},   //X-left
        {gsl::Vec3{ 500.f, 0.f,   0.f},     gsl::Vec3{1.f, 0.f, 0.f},   gsl::Vec2{0.f, 0.f}},   //X-right
        {gsl::Vec3{ 0.f,  -500.f, 0.f},     gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{0.f, 0.f}},   //Y-bottom
        {gsl::Vec3{ 0.f,   500.f, 0.f},     gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{0.f, 0.f}},   //Y-top
        {gsl::Vec3{ 0.f,   0.f,  -500.f},   gsl::Vec3{0.f, 0.f, 1.f},   gsl::Vec2{0.f, 0.f}},   //Z-back
        {gsl::Vec3{ 0.f,   0.f,   500.f},   gsl::Vec3{0.f, 0.f, 1.f},   gsl::Vec2{0.f, 0.f}}    //Z-front
    });

    temp->mMeshName = "axis";

    //No Bounding Box made - makes no sense...
    return temp;
}


MeshBase* makeTriangle()
{
    MeshBase *temp =  new MeshBase(GL_TRIANGLES,
    {
        //Positions                     //Colors                    //UVs
        {gsl::Vec3{-0.5f, -0.5f, 0.f},  gsl::Vec3{1.f, 0.f, 0.f},   gsl::Vec2{0.f,  0.f}},  //Bottom Left
        {gsl::Vec3{0.5f,  -0.5f, 0.f},  gsl::Vec3{0.f, 1.f, 0.f},   gsl::Vec2{1.f,  0.f}},  //Bottom Right
        {gsl::Vec3{0.f,    0.5f, 0.f},  gsl::Vec3{0.f, 0.f, 1.f},   gsl::Vec2{0.5f, 1.f}}   //Top
    });

    temp->mMeshName = "triangle";

    //Boudning box:
    temp->mBoundingBoxLeftDownBack = gsl::Vec3(-0.5f, -0.5f, 0.f);
    temp->mBoundingBoxRightUpFront = gsl::Vec3(0.5f, 0.5f, 0.f);

    return temp;
}

#endif // PRIMITIVES_H
