#include "objmesh.h"
#include "constants.h"
#include "vertex.h"
#include "vec3.h"
#include "vec2.h"
#include <fstream>
#include <vector>
#include <sstream>


ObjMesh::ObjMesh(std::string filePath)
{
    mDrawType = GL_TRIANGLES;
    mFilePath = filePath;

    readObjFile();
}

void ObjMesh::readObjFile()
{
    std::stringstream s;
    std::string path = gsl::assetFilePath + mFilePath;

    std::ifstream inputFileStream(path.c_str(), std::ios::in);

    std::vector<gsl::Vec3> vert_positions;
    std::vector<gsl::Vec3> vert_normals;
    std::vector<gsl::Vec2> vert_uvs;

    std::vector<GLuint> vertexIndices;
    std::vector<GLuint> uvIndices;
    std::vector<GLuint> normalIndices;

    GLfloat x, y, z;

    std::string line = "";

    if (inputFileStream.is_open())
    {
        while (!inputFileStream.eof())
        {
            std::vector<std::string> args;
            getline(inputFileStream, line);

            split(line, ' ', args);

            //Check the first argument
            if (args.size() > 1)
            {
                //Vertex data follows
                if (args[0].compare("v") == 0)
                {
                    //Push back the next three strings into sstream
                    x = static_cast<GLfloat>(atof(args[1].c_str()));
                    y = static_cast<GLfloat>(atof(args[2].c_str()));
                    z = static_cast<GLfloat>(atof(args[3].c_str()));

                    vert_positions.push_back(gsl::Vec3(x, y, z));

                    //Making Bounding Box data:
                    if (mBoundingBoxRightUpFront.getX() < x)
                        mBoundingBoxRightUpFront.setX(x);
                    else if (mBoundingBoxLeftDownBack.getX() > x)
                        mBoundingBoxLeftDownBack.setX(x);
                    if (mBoundingBoxRightUpFront.getY() < y)
                        mBoundingBoxRightUpFront.setY(y);
                    else if (mBoundingBoxLeftDownBack.getY() > y)
                        mBoundingBoxLeftDownBack.setY(y);
                    if (mBoundingBoxRightUpFront.getZ() < z)
                        mBoundingBoxRightUpFront.setZ(z);
                    else if (mBoundingBoxLeftDownBack.getZ() > z)
                        mBoundingBoxLeftDownBack.setZ(z);
                }

                //UV data follows
                if (args[0].compare("vt") == 0)
                {
                    //Push back the next two strings into sstream
                    x = static_cast<GLfloat>(atof(args[1].c_str()));
                    y = static_cast<GLfloat>(atof(args[2].c_str()));

                    vert_uvs.push_back(gsl::Vec2(x, y));
                }

                //Vertex Normal data follows
                if (args[0].compare("vn") == 0)
                {
                    //Push back the next three strings into sstream
                    x = static_cast<GLfloat>(atof(args[1].c_str()));
                    y = static_cast<GLfloat>(atof(args[2].c_str()));
                    z = static_cast<GLfloat>(atof(args[3].c_str()));

                    vert_normals.push_back(gsl::Vec3(x, y, z));
                }

                //How many meshes are contained in obj file
                if (args[0].compare("s") == 0)
                {

                }

                //What is the name of current mesh
                if (args[0].compare("g") == 0)
                {

                }

                //UseMTL
                if (args[0].compare("usemtl") == 0)
                {

                }

                //Face-data of mesh follows
                if (args[0].compare("f") == 0)
                {
                    //Check if its a quad or triangle you are loading!
                    struct quad
                    {
                        GLuint vI[4];
                        GLuint uI[4];
                        GLuint nI[4];
                    };

                    if (args.size() == 4)
                    {
                        std::vector<std::string> fData1;
                        std::vector<std::string> fData2;
                        std::vector<std::string> fData3;

                        split(args[1], '/', fData1);
                        split(args[2], '/', fData2);
                        split(args[3], '/', fData3);

                        GLuint vI[3];
                        GLuint uI[3];
                        GLuint nI[3];

                        vI[0] = static_cast<GLuint>(atoi(fData1[0].c_str()));
                        uI[0] = static_cast<GLuint>(atoi(fData1[1].c_str()));
                        nI[0] = static_cast<GLuint>(atoi(fData1[2].c_str()));

                        vI[1] = static_cast<GLuint>(atoi(fData2[0].c_str()));
                        uI[1] = static_cast<GLuint>(atoi(fData2[1].c_str()));
                        nI[1] = static_cast<GLuint>(atoi(fData2[2].c_str()));

                        vI[2] = static_cast<GLuint>(atoi(fData3[0].c_str()));
                        uI[2] = static_cast<GLuint>(atoi(fData3[1].c_str()));
                        nI[2] = static_cast<GLuint>(atoi(fData3[2].c_str()));

                        vertexIndices.push_back(vI[0]);
                        vertexIndices.push_back(vI[1]);
                        vertexIndices.push_back(vI[2]);

                        uvIndices.push_back(uI[0]);
                        uvIndices.push_back(uI[1]);
                        uvIndices.push_back(uI[2]);

                        normalIndices.push_back(nI[0]);
                        normalIndices.push_back(nI[1]);
                        normalIndices.push_back(nI[2]);
                    }
                    else if (args.size() == 5)
                    {
                        quad q;
                        for (size_t i = 4; i >= 1; i--)
                        {
                            std::vector<std::string> fData;

                            split(args[i], '/', fData);

                            q.vI[4-i] = static_cast<GLuint>(atoi(fData[0].c_str()));
                            q.uI[4-i] = static_cast<GLuint>(atoi(fData[1].c_str()));
                            q.nI[4-i] = static_cast<GLuint>(atoi(fData[2].c_str()));
                        }

                        //Convert the quad to two triangles...
                        vertexIndices.push_back(q.vI[0]);
                        vertexIndices.push_back(q.vI[1]);
                        vertexIndices.push_back(q.vI[2]);

                        vertexIndices.push_back(q.vI[2]);
                        vertexIndices.push_back(q.vI[3]);
                        vertexIndices.push_back(q.vI[0]);

                        uvIndices.push_back(q.uI[0]);
                        uvIndices.push_back(q.uI[1]);
                        uvIndices.push_back(q.uI[2]);

                        uvIndices.push_back(q.uI[2]);
                        uvIndices.push_back(q.uI[3]);
                        uvIndices.push_back(q.uI[0]);

                        normalIndices.push_back(q.nI[0]);
                        normalIndices.push_back(q.nI[1]);
                        normalIndices.push_back(q.nI[2]);

                        normalIndices.push_back(q.nI[2]);
                        normalIndices.push_back(q.nI[3]);
                        normalIndices.push_back(q.nI[0]);
                    }
                    else
                    {
                        std::cout << "Please refrain from having ngons in your .obj file OMG geez!" << std::endl;
                    }
                }
            }
        }

        //Load over to vertices
        mNumberOfVertices = vertexIndices.size();
        mVertices = new Vertex[mNumberOfVertices];

        for (size_t i = 0; i < vertexIndices.size(); i++)
        {
            GLuint vI = vertexIndices[i] - 1;
            GLuint nI = normalIndices[i] - 1;
            GLuint uI = uvIndices[i] - 1;

            mVertices[i].set_xyz(vert_positions[vI]);
            mVertices[i].set_normal(vert_normals[nI]);
            mVertices[i].set_uv(vert_uvs[uI].getX(), vert_uvs[uI].getY());
        }
    }
    else
    {
       std::cout << "Failed to open: " << mFilePath << std::endl;
    }
}

void ObjMesh::split(std::string &str, const char &delim, std::vector<std::string> &outArr)
{
    size_t lastFind = 0;
    size_t find = str.find(delim);

    std::string temp;
    while (find != std::string::npos)
    {
        temp = str.substr(lastFind, find-lastFind);
        outArr.push_back(temp);

        lastFind = find + 1;
        find = str.find(delim, lastFind);
    }

    outArr.push_back(str.substr(lastFind, str.length()-lastFind));
}
