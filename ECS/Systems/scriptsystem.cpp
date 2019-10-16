#include "scriptsystem.h"
#include "ecsmanager.h"

ScriptSystem::ScriptSystem()
{
}

void ScriptSystem::update(GLfloat deltaTime)
{
    ECSManager* manager = ECSManager::getInstance();
    ScriptComponent** components = &manager->mScriptComponents.back();
    size_t numComponents = manager->mScriptComponents.size();

    for(size_t i = 0; i < numComponents; i++)
    {
        components[i]->update(deltaTime);
    }
}
