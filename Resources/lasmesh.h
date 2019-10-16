#ifndef LASMESH_H
#define LASMESH_H

#include "meshbase.h"

class LasMesh : public MeshBase
{
    public:
        LasMesh(const std::string &filePath);
        std::string getFilesPath() const {return mFilePath;}
    private:
        void readLasFile();
};

#endif // LASMESH_H
