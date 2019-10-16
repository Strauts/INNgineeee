#ifndef SYSTEMBASE_H
#define SYSTEMBASE_H

#include "gltypes.h"

class SystemBase
{
    public:
        SystemBase();
        virtual ~SystemBase();

        virtual void init();
        virtual void update();
        virtual void update(GLfloat deltatime);
        GLuint getSystemID() const;

    private:
        static GLuint ID;
        GLuint mSystemID;
};

#endif // SYSTEMBASE_H
