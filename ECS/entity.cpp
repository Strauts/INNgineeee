#include "entity.h"
#include "vertex.h"
#include "transform.h"
#include "material.h"
#include "shader.h"
#include "mat4.h"
#include "meshbase.h"
#include "ecsmanager.h"
#include <cassert>

//Init static var
GLuint Entity::ID{0};


Entity::Entity()
{
    mEntityID = ID;
    ID++;
}

Entity::~Entity()
{
}


void Entity::createObject()
{
}

Entity* Entity::getParent() const
{
    return mParent;
}

void Entity::setParent(Entity *parent)
{
    mParent = parent;
}

TransformComponent *Entity::getFirstTransformComponent(GLuint entityID)
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<TransformComponent> &components = manager->mTransformComponents;
    size_t size = manager->mTransformComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == entityID)
            return &components[i];
    }

    return nullptr;
}

Transform& Entity::getTransform()
{
    // Transform exists in transform component now
    return mMeshComp->getFirstTransformComponent()->mTransform;
}

void Entity::setMaterial(Material *material)
{
    mMaterialComp = new MaterialComponent;
    mMaterialComp->mMaterial = material;
    mMaterialComp->mMaterial->setupShader(mMeshComp->mVAO);
}

Material* Entity::getMaterial()
{
    return mMaterialComp->mMaterial;
}

void Entity::setObjectName(const std::string &objectName)
{
    mObjectName = objectName;
}

std::string Entity::getObjectName() const
{
    return mObjectName;
}
