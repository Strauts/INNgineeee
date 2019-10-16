#include "graphicscomponent.h"
#include "vertex.h"
#include "material.h"
#include "shader.h"

using namespace gsl;

GraphicsComponent::GraphicsComponent()
{

}

GraphicsComponent::~GraphicsComponent()
{
//    if(mOwner->vertices())
//    {
//        glDeleteBuffers(1, &mVBO);
//        glDeleteVertexArrays(1, &mVAO);
//    }
//    if(mOwner->vertices() && mOwner->indices())
//        glDeleteBuffers(1, &mEAO);
}
