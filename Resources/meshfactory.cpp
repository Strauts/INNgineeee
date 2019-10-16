#include "meshfactory.h"
#include "meshbase.h"
#include "octahedron.h"
#include "objmesh.h"
#include "lasmesh.h"
#include "vertex.h"
#include "vec3.h"
#include "vec2.h"
#include "primitives.h"


MeshFactory::MeshFactory()
{
    mMeshes["cube"] = makeCube();
    mMeshes["plane"] = makePlane();
    mMeshes["sphere"] = makeSphere(3);
    mMeshes["axis"] = makeAxis();
    mMeshes["triangle"] = makeTriangle();
}

MeshFactory::~MeshFactory()
{
    for (auto i = mMeshes.begin(); i != mMeshes.end(); i++)
    {
        delete (*i).second;
    }

    mMeshes.clear();
}

MeshBase* MeshFactory::getMesh(const std::string &identifier)
{
    return mMeshes[identifier];
}

MeshComponent MeshFactory::getMeshComponent(const std::string &identifier)
{
    MeshBase *mesh = mMeshes[identifier];
    //mesh->init();
    MeshComponent ret;

    if (!mesh)
    {
        //Load mesh
        std::string fileExt = getFileExtension(identifier);

        //Load obj file here
        if (fileExt.compare("obj") == 0)
        {
            mesh = new ObjMesh(identifier);
            mesh->init();
            mMeshes[identifier] = mesh;

            ret.mData = mesh;
            ret.mDrawType = mesh->mDrawType;
            ret.mNumberOfVertices = mesh->mNumberOfVertices;
            ret.mNumberOfIndices = mesh->mNumberOfIndices;
            ret.mVAO = mesh->mVAO;

            return ret;
        }
        //Load las file here
        else if (fileExt.compare("las") == 0)
        {
            mesh = new LasMesh(identifier);
            mesh->init();
            mMeshes[identifier] = mesh;

            ret.mData = mesh;
            ret.mDrawType = mesh->mDrawType;
            ret.mNumberOfVertices = mesh->mNumberOfVertices;
            ret.mNumberOfIndices = mesh->mNumberOfIndices;
            ret.mVAO = mesh->mVAO;

            return ret;
        }
        else
        {
            std::cout << "Filename not recognized: " << identifier << std::endl;
        }
    }
    else
    {
        mesh->init();
        ret.mData = mesh;
        ret.mDrawType = mesh->mDrawType;
        ret.mNumberOfVertices = mesh->mNumberOfVertices;
        ret.mNumberOfIndices = mesh->mNumberOfIndices;
        ret.mVAO = mesh->mVAO;
        ret.mOBB_VAO = mesh->mOBB_VAO;

        return ret;
    }

    return MeshComponent();
}

std::string MeshFactory::getFileExtension(const std::string &filename)
{
    GLuint find = filename.find_last_of('.');

    return (find > 0) ? filename.substr(find + 1, filename.length() - find - 1) : "";
}
