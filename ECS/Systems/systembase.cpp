#include "systembase.h"


GLuint SystemBase::ID{0};


SystemBase::SystemBase()
{
    mSystemID = ID;
    ID++;
}

SystemBase::~SystemBase()
{
}

void SystemBase::init()
{
}

void SystemBase::update()
{
}

void SystemBase::update(GLfloat deltatime)
{
}

GLuint SystemBase::getSystemID() const
{
    return mSystemID;
}
