#include "meshbase.h"
#include "vertex.h"
#include "ecsmanager.h"
#include "constants.h"
#include "material.h"
#include "collisionsystem.h"
#include <QDebug>


MeshBase::MeshBase()
{
}

MeshBase::MeshBase(GLenum drawType, std::initializer_list<Vertex> vlist)
{
    mDrawType = drawType;
    mNumberOfVertices = vlist.size();

    GLint k;

    if (mNumberOfVertices > 0)
    {
        k = 0;
        mVertices = new Vertex[mNumberOfVertices];
        for (auto v : vlist)
            mVertices[k++] = v;
    }
}

MeshBase::MeshBase(GLenum drawType, std::initializer_list<Vertex> vlist, std::initializer_list<GLuint> ilist)
{
    mDrawType = drawType;
    mNumberOfVertices = vlist.size();
    mNumberOfIndices = ilist.size();

    GLint k;

    if (mNumberOfVertices > 0)
    {
        k = 0;
        mVertices = new Vertex[mNumberOfVertices];
        for (auto v : vlist)
            mVertices[k++] = v;
    }

    if (mNumberOfIndices > 0)
    {
        k = 0;
        mIndices = new GLuint[mNumberOfIndices];
        for (auto i : ilist)
            mIndices[k++] = i;
    }
}

MeshBase::~MeshBase()
{
}

void MeshBase::init()
{
    initializeOpenGLFunctions();

    averageNormals();

    if(!mVertices)
        return;

    glGenVertexArrays(1, &mVAO);
    glBindVertexArray(mVAO);

    //Reuse VBO if it exists
    if (mVBO == 0)
    {
        glGenBuffers(1, &mVBO);
        glBindBuffer(GL_ARRAY_BUFFER, mVBO);
        glBufferData(GL_ARRAY_BUFFER, mNumberOfVertices * sizeof(Vertex), mVertices, GL_STATIC_DRAW);
    }
    else
        glBindBuffer(GL_ARRAY_BUFFER, mVBO);

    if (mNumberOfIndices > 0)
    {
        //Reuse EAO if it exists
        if (mEAO == 0)
        {
            glGenBuffers(1, &mEAO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEAO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, mNumberOfIndices * sizeof(GLuint), mIndices, GL_STATIC_DRAW);
        }
        else
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEAO);
    }

    glBindVertexArray(0);
}

void MeshBase::makeBoundingBox()
{
    //If no boundingbox data is set or if OBB_VAO is made already
    if (mBoundingBoxRightUpFront.getX() < 0.01f || mOBB_VAO != 0)
        return;

    //Setting start values
    GLfloat xMax = mBoundingBoxRightUpFront.getX();
    GLfloat xMin = mBoundingBoxLeftDownBack.getX();
    GLfloat yMax = mBoundingBoxRightUpFront.getY();
    GLfloat yMin = mBoundingBoxLeftDownBack.getY();
    GLfloat zMax = mBoundingBoxRightUpFront.getZ();
    GLfloat zMin = mBoundingBoxLeftDownBack.getZ();


    //The color of the bounding box
    gsl::Vec3 color(1.f, 0.f, 1.f);

    //Position, Normal, UV
    Vertex vertices[] = {
        //Vertex data for top
        {gsl::Vec3(xMax, yMax, zMax),  color,   gsl::Vec2(0.f, 0.f)}, //v0
        {gsl::Vec3(xMax, yMax, zMin),  color,   gsl::Vec2(0.f, 0.f)}, //v1
        {gsl::Vec3(xMin, yMax, zMin),  color,   gsl::Vec2(0.f, 0.f)}, //v2
        {gsl::Vec3(xMin, yMax, zMax),  color,   gsl::Vec2(0.f, 0.f)}, //v3

        // Vertex data for bottom
        {gsl::Vec3(xMax, yMin, zMax),   color,  gsl::Vec2(0.f, 0.f)}, //v0
        {gsl::Vec3(xMax, yMin, zMin),   color,  gsl::Vec2(0.f, 0.f)}, //v1
        {gsl::Vec3(xMin, yMin, zMin),   color,  gsl::Vec2(0.f, 0.f)}, //v2
        {gsl::Vec3(xMin, yMin, zMax),   color,  gsl::Vec2(0.f, 0.f)}  //v3
    };

    //Indices for GL_LINE_LOOP
    GLushort indices[] = {0, 1, 2, 3, 0,        //Upper square
                          4, 5, 6, 7, 4,        //Lower square
                          5, 1, 2, 6, 7, 3};    //The rest

    //initializeOpenGLFunctions();

    glGenVertexArrays(1, &mOBB_VAO);
    glBindVertexArray(mOBB_VAO);

    glGenBuffers(1, &mOBB_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, mOBB_VBO);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(Vertex), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &mOBB_EAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mOBB_EAO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 16 * sizeof(GLushort), indices, GL_STATIC_DRAW);

    //OEF: set up material for this!!!
    ECSManager::getInstance()->mMaterials[gsl::COLOR]->setupShader(mOBB_VBO);

    glBindVertexArray(0);
}

void MeshBase::averageNormals()
{
    std::vector<GLint> triIndices;
    // For each vertex we need to check which triangles are connected to it

    // Find the first triangle that has the index...
    size_t ti = 0;

    Triangle thisTri, downTri, leftTri, downLeftTri, leftleftTri, downLeftLeftTri;

    for(ti = 0; ti < mNumberOfTriangles; ti += 2)
    {
        triIndices.push_back(static_cast<int>(ti));
        thisTri = mTriangles[ti];

        if(thisTri.e2 != -1)
        {
            triIndices.push_back(thisTri.e2);
            downTri = mTriangles[thisTri.e2];

            if(downTri.e3 != -1)
            {
                triIndices.push_back(downTri.e3);
                downLeftTri = mTriangles[downTri.e3];

                if(downLeftTri.e1 != -1)
                {
                    triIndices.push_back(downLeftTri.e1);
                }
            }
        }

        if(thisTri.e1 != -1)
        {
            triIndices.push_back(thisTri.e1);

            leftTri = mTriangles[thisTri.e1];

            if(leftTri.e3 != -1)
            {
                triIndices.push_back(leftTri.e3);
            }
        }

        // Set the average vertex
        gsl::Vec3 sum;
        size_t numIndices = triIndices.size();
        for(size_t i = 0; i < numIndices; i++)
        {
            sum = sum + normalFromTriangle(triIndices[i], mTriangles, mVertices);
        }

        sum = sum * (1.f / numIndices);

        sum.normalize();

        mVertices[thisTri.k].set_normal(sum);

        triIndices.clear();
    }
}

gsl::Vec3 MeshBase::normalFromTriangle(int triangleIndex, Triangle* triangles, Vertex* vertices)
{
    Triangle t = triangles[triangleIndex];
    gsl::Vec3 v1 = vertices[t.i].get_xyz();
    gsl::Vec3 v2 = vertices[t.j].get_xyz();
    gsl::Vec3 v3 = vertices[t.k].get_xyz();

    gsl::Vec3 u, w, faceNormal;

    bool even = (triangleIndex % 2 == 0);

    // The vectors are different based on the type of triangle
    if(even)
    {
        u = v3 - v1;
        w = v2 - v1;
        faceNormal = w^u;
    }
    else
    {
        u = v2 - v1;
        w = v3 - v1;
        faceNormal = u^w;
    }

    faceNormal.normalize();
    return faceNormal;
}

gsl::Vec3 MeshBase::centerFromTriangle(int triangleIndex, Triangle *triangles, Vertex *vertices)
{
    Triangle t = triangles[triangleIndex];
    gsl::Vec3 v1 = vertices[t.i].get_xyz();
    gsl::Vec3 v2 = vertices[t.j].get_xyz();
    gsl::Vec3 v3 = vertices[t.k].get_xyz();

    return gsl::Vec3((v1.x + v2.x + v3.x) / 3, (v1.y + v2.y + v3.y) / 3, (v1.z + v2.z + v3.z) / 3);
}

int MeshBase::findTriangleIndexFromWorldPosition(int triangleIndex, Vec3 worldPos, Mat4 &terrainMat, Triangle *triangles, Vertex *vertices)
{
    Vec3 localPos = (terrainMat * Vec4(worldPos, 1)).toVector3D();

    gsl::Vec2 p1, p2, p3, currentPoint;
    gsl::Vec3 v3_p1, v3_p2, v3_p3, area;

    int i = triangleIndex;
    Triangle t;
    gsl::Vec3 normal, triangleCenter;

    //Find the triangle with the position
    for(; i != -1;)
    {
        t = mTriangles[i];
        v3_p1 = mVertices[t.i].get_xyz();
        v3_p2 = mVertices[t.j].get_xyz();
        v3_p3 = mVertices[t.k].get_xyz();

        normal = MeshBase::normalFromTriangle(i, triangles, vertices);
        triangleCenter = centerFromTriangle(i, triangles, vertices);

        float dist = Vec3::dot((localPos - triangleCenter), normal);

        p1 = gsl::Vec2(v3_p1.getX(), v3_p1.getZ());
        p2 = gsl::Vec2(v3_p2.getX(), v3_p2.getZ());
        p3 = gsl::Vec2(v3_p3.getX(), v3_p3.getZ());

        currentPoint = gsl::Vec2(localPos.getX(), localPos.getZ());
        area = currentPoint.barycentricCoordinates(p1, p2, p3);

        if(area.getX() >= 0 && area.getY() >= 0 && area.getZ() >= 0)
        {
            //The point is inside a triangle
            return i;
        }

        //Checking distance to the triangle from the center

        //We are in the first triangle of the quad
        if(i % 2 == 0)
        {
            //Go right - up
            if(area.getX() < 0 && area.getY() >= 0 && area.getZ() >= 0)
            {
                i = t.e3;
            }
            //Go left
            else if(area.getY() < 0 && area.getX() >= 0 && area.getZ() >= 0)
            {
                i = t.e1;
            }
            //Go down
            else if(area.getZ() < 0 && area.getX() >= 0 && area.getY() >= 0)
            {
                i = t.e2;
            }
            else
            {
                //Go down
                if(area.getZ() < area.getY())
                {
                    i = t.e2;
                }
                //Go left
                else
                {
                    i = t.e1;
                }
            }
        }
        else
        {
            //Go left - down
            if(area.getY() < 0 && area.getZ() >= 0 && area.getX() >= 0)
            {
                i = t.e3;
            }
            //Go up
            else if(area.getX() < 0 && area.getY() >= 0 && area.getZ() >= 0)
            {
                i = t.e1;
            }
            //Go right
            else if(area.getZ() < 0 && area.getX() >= 0 && area.getY() >= 0)
            {
                i = t.e2;
            }
            //Go left - down
            else if(area.getX() < 0 && area.getY() < 0 && area.getZ() >= 0)
            {
                i = t.e3;
            }
            //Go right
            else if(area.getX() < 0 && area.getZ() < 0 && area.getY() >= 0)
            {
                i = t.e2;
            }
            else
            {
                //Go up
                if(area.getY() < area.getZ())
                {
                    i = t.e1;
                }
                //Go left - down
                else
                {
                    i = t.e3;
                }
            }
        }
    }
    return -1;
}

void MeshBase::printMeshData()
{
    qDebug() << " ---Meshbase data---";

    std::string drawtype;
    switch (mDrawType)
    {
    case 0:
        drawtype = "GL_POINTS";
        break;
    case 1:
        drawtype = "GL_LINES";
        break;
    case 2:
        drawtype = "GL_LINE_LOOP";
        break;
    case 3:
        drawtype = "GL_LINE_STRIP";
        break;
    case 4:
        drawtype = "GL_TRIANGLES";
        break;
    case 5:
        drawtype = "GL_TRIANGLE_STRIP";
        break;
    default:
        drawtype = std::to_string(mDrawType);
        break;
    }

    qDebug() << "Mesh type: " << mMeshName.c_str();
    qDebug() << "DrawType: " << drawtype.c_str();
    qDebug() << "mNumberOfVertices "<< mNumberOfVertices;
    qDebug() << "mNumberOfIndices " << mNumberOfIndices;

    //For the main mesh
    qDebug() << "VAO " << mVAO;
    qDebug() << "VBO " << mVBO;
    qDebug() << "EAO " << mEAO;

    //For the Bounding Box
    qDebug() << "OBB_VAO " << mOBB_VAO;
    qDebug() << "OBB_VBO " << mOBB_VBO;
    qDebug() << "OBB_EAO " << mOBB_EAO;

    qDebug() << "mFilePath " << mFilePath.c_str();
    qDebug() << " ----------------";
}

Vertex* MeshBase::vertices() const
{
    return &mVertices[0];
}
