#include "linedebug.h"
#include "material.h"
#include "constants.h"
#include "ecsmanager.h"
#include "vertex.h"
#include "shader.h"
#include "renderwindow.h"

LineDebug* LineDebug::mInstance = nullptr;

LineDebug::LineDebug()
{
    mOwner = ECSManager::getInstance();
    mInstance = this;
}

LineDebug::~LineDebug()
{
}

LineDebug *LineDebug::getInstance()
{
    return mInstance;
}

void LineDebug::createObject()
{
    // Put OpenGL functions into scope
    initializeOpenGLFunctions();

    mMaterial = mOwner->mMaterials[gsl::COLOR];

    // Generate Vertex Array Object
    glGenVertexArrays(1, &mVAO);

    // Tell OpenGL we are using this VAO
    glBindVertexArray(mVAO);

    // Create a Vertex Buffer Object to hold our
    // Vertex Data
    glGenBuffers(1, &mVBO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);

    // The vertexes can change, hence GL_DYNAMIC_DRAW
    glBufferData(GL_ARRAY_BUFFER, static_cast<int>(mNumVertices * sizeof(Vertex)), mVertices, GL_DYNAMIC_DRAW);

    mMaterial->setupShader(mVAO);
}

void LineDebug::drawObject()
{
    // if we have drawn lines in the past
    if(mVertices)
    {
        delete[] mVertices;
        mVertices = nullptr;
    }

    // If there are lines present
    if(lines.size() > 0)
    {
        // Create lines from two points, hence lines.size()*2
        mNumVertices = lines.size()*2;
        mVertices = new Vertex[mNumVertices];

        // Each line has two points, start and end.
        int c = 0;
        for(size_t i = 0; i < lines.size(); i++)
        {
            mVertices[c  ] = {lines[i].start, lines[i].color, Vec2(1.f, 1.f)};
            mVertices[c+1] = {lines[i].end, lines[i].color, Vec2(1.f, 1.f)};

            c += 2;
        }

        // Empty lines as we don't need to draw them next frame
        lines.clear();

        // Tell OpenGL to use our shader
        mMaterial->getShader()->Use();

        // Tell OpenGL we are using this VAO
        glBindVertexArray(mVAO);

        // Bind the VBO we are using, and buffer in new data to draw
        glBindBuffer(GL_ARRAY_BUFFER, mVBO);
        glBufferData(GL_ARRAY_BUFFER, static_cast<int>(mNumVertices * sizeof(Vertex)), mVertices, GL_DYNAMIC_DRAW);

        // Get camera from our owner
        Mat4 model(true);
        mMaterial->setMMatrix(model);

        // Tell OpenGL to draw all the vertices inside the VAO
        glDrawArrays(GL_LINES, 0, static_cast<int>(mNumVertices));

        mOwner->mRenderWindow->checkForGLerrors();
    }
}

void LineDebug::addLine(const Vec3 &Start, const Vec3 &End, const Vec3 &Color)
{
    lines.push_back(line(Start, End, Color));
}
