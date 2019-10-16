#ifndef OBJMESH_H
#define OBJMESH_H

#include "meshbase.h"
#include <string>

class ObjMesh : public MeshBase
{
    public:
        ObjMesh(std::string filePath);

    private:
        void readObjFile();
        void split(std::string &str, const char &delim, std::vector<std::string> &outArr);
};

#endif // OBJMESH_H
