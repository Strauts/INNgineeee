#ifndef MESHBASE_H
#define MESHBASE_H

#include "gltypes.h"
#include "vec3.h"
#include "componentbase.h"
#include <QOpenGLFunctions_4_1_Core>
#include <initializer_list>

class Vertex;

class MeshBase : protected QOpenGLFunctions_4_1_Core
{
    public:
        MeshBase();
        MeshBase(GLenum drawType, std::initializer_list<Vertex> vlist);
        MeshBase(GLenum drawType, std::initializer_list<Vertex> vlist, std::initializer_list<GLuint> ilist);
        ~MeshBase();

        void init();
        void makeBoundingBox();
        void averageNormals();
        static gsl::Vec3 normalFromTriangle(int triangleIndex, Triangle *triangles, Vertex *vertices);
        static gsl::Vec3 centerFromTriangle(int triangleIndex, Triangle *triangles, Vertex *vertices);
        void printMeshData();

        int findTriangleIndexFromWorldPosition(int triangleIndex, Vec3 worldPos, Mat4&terrainMat, Triangle *triangles, Vertex *vertices);

        Vertex* vertices() const;
        GLint* indices() const;
        std::string getFilePath() const {return mFilePath;}

        Vertex *mVertices{nullptr};
        GLuint *mIndices{nullptr};
        Triangle *mTriangles{nullptr};

        std::string mFilePath;
        std::string mMeshName;

        GLenum mDrawType;
        GLuint mNumberOfVertices{0};
        GLuint mNumberOfIndices{0};
        GLuint mNumberOfTriangles{0};

        GLuint mVAO{0};
        GLuint mVBO{0};
        GLuint mEAO{0};

        //For the Bounding Box
        GLuint mOBB_VAO{0};
        GLuint mOBB_VBO{0};
        GLuint mOBB_EAO{0};

        gsl::Vec3 mBoundingBoxRightUpFront;
        gsl::Vec3 mBoundingBoxLeftDownBack;
};

#endif // MESHBASE_H
