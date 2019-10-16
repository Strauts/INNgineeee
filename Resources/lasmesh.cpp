#include "lasmesh.h"
#include "vertex.h"
#include "gltypes.h"
#include "lasreader.h"


LasMesh::LasMesh(const std::string &filePath)
{
    mDrawType = GL_TRIANGLES;
    mFilePath = filePath;

    readLasFile();
}

void LasMesh::readLasFile()
{
    LASReader reader(mFilePath);

    GLuint realHeight;
    GLuint realWidth;
    GLdouble** downscaled = reader.makeHeightmapFromPointData(realHeight, realWidth);

    GLfloat sX = -static_cast<GLfloat>(realWidth)/2.f, sZ = -static_cast<GLfloat>(realHeight)/2.f;

    mNumberOfVertices = static_cast<GLuint>(realHeight*realWidth);
    mVertices = new Vertex[mNumberOfVertices];

    //Create a terrain that is realWidth x realHeight where we plot in our heightmap data
    for (GLuint i = 0; i < realHeight; i++)
    {
        for (GLuint j = 0; j < realWidth; j++)
        {
            mVertices[i*realWidth + j] = {Vec3(sX, static_cast<GLfloat>(downscaled[i][j]), sZ), Vec3(0, 1, 0), Vec2()};
            sX += 1;
        }
        sZ += 1;
        sX = -static_cast<GLfloat>(realWidth)/2.f;
    }

    //Create an index array to save space
    mNumberOfIndices = 6*(realHeight-1)*(realWidth-1);
    mNumberOfTriangles = 2*(realHeight-1)*(realWidth-1);
    mTriangles = new Triangle[mNumberOfTriangles];
    mIndices = new GLuint[mNumberOfIndices];

    GLuint k{0};
    GLuint nt{0};

    for(GLuint i = 0; i < realHeight-1; i++)
    {
        for (GLuint j = 0; j < realWidth-1; j++)
        {
            mIndices[k] = realWidth*i + realWidth + j;
            mIndices[k+1] = realWidth*i + j+1;
            mIndices[k+2] = realWidth*i + j;

            mTriangles[nt].e1 = (j > 0) ? static_cast<int>(nt - 1) : - 1;
            mTriangles[nt].e2 = (i > 0) ? static_cast<int>(nt - (realWidth - 1)*2) : -1;
            mTriangles[nt].e3 = static_cast<int>(nt + 1);

            mTriangles[nt].i = static_cast<int>(mIndices[k]);
            mTriangles[nt].j = static_cast<int>(mIndices[k+1]);
            mTriangles[nt].k = static_cast<int>(mIndices[k+2]);
            nt++;

            mIndices[k+3] = realWidth*i + realWidth + j;
            mIndices[k+4] = realWidth*i + realWidth + 1 + j;
            mIndices[k+5] = realWidth*i + j+1;

            mTriangles[nt].e1 = (j < realWidth - 2) ? static_cast<int>(nt + 1) : -1;
            mTriangles[nt].e2 = (i < realHeight - 2) ? static_cast<int>(nt + (realWidth-1)*2 - 1) : -1;
            mTriangles[nt].e3 = static_cast<int>(nt - 1);

            mTriangles[nt].i = static_cast<int>(mIndices[k+3]);
            mTriangles[nt].j = static_cast<int>(mIndices[k+4]);
            mTriangles[nt].k = static_cast<int>(mIndices[k+5]);
            nt++;

            k += 6;
        }
    }

    //We have no need for the heightmap anymore
    for (GLuint i = 0; i < realHeight; i++)
        delete[] downscaled[i];

    delete[] downscaled;
}
