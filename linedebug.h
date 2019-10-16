#ifndef LINEDEBUG_H
#define LINEDEBUG_H

#include <QOpenGLFunctions_4_1_Core>

#include "vec3.h"

using namespace gsl;
using namespace std;

struct line
{
    line(Vec3 s, Vec3 e, Vec3 c)
    {
        start = s;
        end = e;
        color = c;
    }
    Vec3 start;
    Vec3 end;
    Vec3 color;
};

class Material;
class Vertex;
class ECSManager;

class LineDebug : protected QOpenGLFunctions_4_1_Core
{
public:
    LineDebug();
    ~LineDebug();

    static LineDebug* getInstance();

    void createObject();
    void drawObject();

    void addLine(const Vec3& Start, const Vec3& End, const Vec3& Color);
private:
    GLuint mVAO, mVBO;
    Vertex* mVertices{nullptr};
    size_t mNumVertices;

    ECSManager* mOwner{nullptr};
    Material* mMaterial;
    std::vector<line> lines;

    static LineDebug* mInstance;
};

#endif // LINEDEBUG_H
