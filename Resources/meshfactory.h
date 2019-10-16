#ifndef MESHFACTORY_H
#define MESHFACTORY_H

#include "constants.h"
#include "gltypes.h"
#include "componentbase.h"
#include <string>
#include <map>

class MeshBase;

class MeshFactory
{
    public:
        MeshFactory();
        ~MeshFactory();

        //MeshBase* getMesh(gsl::ShapeType shape, GLint recursions = 2);
        MeshBase* getMesh(const std::string &identifier);
        MeshComponent getMeshComponent(const std::string &identifier);

    private:
        std::string getFileExtension(const std::string &filename);
        std::map<std::string, MeshBase*> mMeshes;
};

#endif // MESHFACTORY_H
