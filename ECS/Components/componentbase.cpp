#include "componentbase.h"
#include "ecsmanager.h"
#include <typeinfo>

//Adds force at center of mass
void RigidBodyComponent::addForce(const gsl::Vec3 &force)
{
    totalForceApplied += force;
}

//Adds force so that the objects gets an angular velocity
void RigidBodyComponent::addForceAtLocation(const gsl::Vec3 &force, const gsl::Vec3 &location)
{
}

void RigidBodyComponent::addRelativeForce(const gsl::Vec3 &force)
{
}

void RigidBodyComponent::addRelavtiveTorque(const gsl::Vec3 &torque)
{
}

void RigidBodyComponent::addTorque(const gsl::Vec3 &torque)
{
    totalAngularForce += torque;
}

void RigidBodyComponent::movePosition(const gsl::Vec3 &pos)
{
    position = pos;
}

void RigidBodyComponent::moveRotation(const gsl::Quaternion &rot)
{
    rotation = rot;
}

void RigidBodyComponent::resetCenterOfMass()
{
}

void RigidBodyComponent::Sleep()
{
    sleepOneFrame = true;
}

void RigidBodyComponent::WakeUp()
{
}

void RigidBodyComponent::SweepTest()
{
}

void RigidBodyComponent::SweepTestAll()
{
}

NPCComponent* ComponentBase::getFirstNPCComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<NPCComponent> &components = manager->mNPCComponents;
    size_t size = manager->mNPCComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if(components[i].ownerEntityID == ownerEntityID)
        {
            return &components[i];
        }
    }
    return nullptr;
}

TransformComponent* ComponentBase::getFirstTransformComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<TransformComponent> &components = manager->mTransformComponents;
    size_t size = manager->mTransformComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

MeshComponent* ComponentBase::getFirstMeshComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    MeshComponent *components = &manager->mMeshComponents.back();
    size_t size = manager->mMeshComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

MaterialComponent* ComponentBase::getFirstMaterialComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<MaterialComponent> &components = manager->mMaterialComponents;
    size_t size = manager->mMaterialComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

InputComponent* ComponentBase::getFirstInputComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<InputComponent> &components = manager->mInputComponents;
    size_t size = manager->mInputComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

ScriptComponent* ComponentBase::getFirstScriptComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<ScriptComponent*> &components = manager->mScriptComponents;
    size_t size = manager->mScriptComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i]->ownerEntityID == ownerEntityID)
            return components[i];
    }

    return nullptr;
}

RigidBodyComponent* ComponentBase::getFirstRigidbodyComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<RigidBodyComponent> &components = manager->mRigidBodyComponents;
    size_t size = manager->mRigidBodyComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

MovementComponent* ComponentBase::getFirstMovementComponent()
{
    return nullptr;
}

ColliderComponent* ComponentBase::getFirstColliderComponent()
{
    return nullptr;
}

BoxColliderComponent* ComponentBase::getFirstBoxColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<BoxColliderComponent> &components = manager->mBoxColliderComponents;
    size_t size = manager->mBoxColliderComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

SphereColliderComponent* ComponentBase::getFirstSphereColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<SphereColliderComponent> &components = manager->mSphereColliderComponents;
    size_t size = manager->mSphereColliderComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

CyllinderColliderComponent* ComponentBase::getFirstCyllinderColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<CyllinderColliderComponent> &components = manager->mCyllinderColliderComponents;
    size_t size = manager->mCyllinderColliderComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

CapsuleColliderComponent* ComponentBase::getFirstCapsuleColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<CapsuleColliderComponent> &components = manager->mCapsuleColliderComponents;
    size_t size = manager->mCapsuleColliderComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

MeshColliderComponent* ComponentBase::getFirstMeshColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<MeshColliderComponent> &components = manager->mMeshColliderComponents;
    size_t size = manager->mMeshColliderComponents.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}

TrophyColliderComponent* ComponentBase::getFirstTrophyColliderComponent()
{
    ECSManager *manager = ECSManager::getInstance();
    std::vector<TrophyColliderComponent> &components = manager->mTrophyColliderComponents;
    size_t size = manager->mMeshColliderComponents.size();

    for(size_t i = 0; i < size; i++)
    {
        if(components[i].ownerEntityID == ownerEntityID)
        {
            return &components[i];
        }
    }
    return nullptr;
}

CameraComponent *ComponentBase::getFirstCameraComponent()
{
    ECSManager *manager = ECSManager::getInstance();

    std::vector<CameraComponent> &components = manager->mCameraComponents;
    size_t size = components.size();

    for (size_t i = 0; i < size; i++)
    {
        if (components[i].ownerEntityID == ownerEntityID)
            return &components[i];
    }

    return nullptr;
}
