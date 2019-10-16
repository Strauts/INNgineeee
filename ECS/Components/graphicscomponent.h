#ifndef GRAPHICSCOMPONENT_H
#define GRAPHICSCOMPONENT_H

#include <QOpenGLFunctions_4_1_Core>
#include "componentbase.h"

class GraphicsComponent : public ComponentBase, protected QOpenGLFunctions_4_1_Core
{
public:
    GraphicsComponent();
    ~GraphicsComponent();
private:
    GLuint mVAO;

    // Should know about this if we are going to animate something
    GLuint mVBO;
};

#endif // GRAPHICSCOMPONENT_H
