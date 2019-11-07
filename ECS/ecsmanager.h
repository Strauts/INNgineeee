#ifndef ECSMANAGER_H
#define ECSMANAGER_H

#include "systembase.h"
#include "componentbase.h"
#include "constants.h"
#include <list>
#include <vector>

class RenderSystem;
class InputSystem;
class CollisionSystem;
class ScriptSystem;
class PhysicsSystem;
class CameraSystem;
class RenderWindow;
class MeshFactory;
class Entity;
class Shader;
class Texture;
class PointLight;
class DirectionalLight;
class NPC;
class BallSystem;

class ECSManager
{
    public:
        ECSManager(RenderWindow *renderWindow);
        static ECSManager* getInstance();

        void runRenderSystem();
        void runInputSystem();
        void runCollisionSystem();
        void runScriptSystem(GLfloat deltaTime);
        void runPhysicsSystem(GLfloat deltaTime);
        void runCameraSystem();
        void runNPCSystem(GLfloat deltaTime);
        void runBallSystem();
        void runApplyPhysics(GLfloat deltaTime);

        Entity* makeEntity(std::string entityName);
        Entity* makeEntity(std::string entityName, std::string meshName, gsl::ShaderTypes shader);

        DirectionalLight *getLight() const;

        std::list<Entity*> mEntities;            //All Entities in the program
        InputComponent* mInComp{nullptr};

        //All components in the program. Making own vectors for each Component
        std::vector<MeshComponent> mMeshComponents;
        std::vector<TransformComponent> mTransformComponents;
        std::vector<MaterialComponent> mMaterialComponents;
        std::vector<InputComponent> mInputComponents;
        std::vector<RigidBodyComponent> mRigidBodyComponents;
        std::vector<ScriptComponent*> mScriptComponents;
        std::vector<BoxColliderComponent> mBoxColliderComponents;
        std::vector<SphereColliderComponent> mSphereColliderComponents;
        std::vector<CyllinderColliderComponent> mCyllinderColliderComponents;
        std::vector<CapsuleColliderComponent> mCapsuleColliderComponents;
        std::vector<MeshColliderComponent> mMeshColliderComponents;
        std::vector<CameraComponent> mCameraComponents;
        std::vector<TrophyColliderComponent> mTrophyColliderComponents;
        std::vector<NPCComponent> mNPCComponents;

        RenderSystem *mRenderSystem{nullptr};
        InputSystem *mInputSystem{nullptr};
        MeshFactory *mMeshFactory{nullptr};
        RenderWindow *mRenderWindow{nullptr};
        CollisionSystem *mCollisionSystem{nullptr};
        ScriptSystem *mScriptSystem{nullptr};
        PhysicsSystem* mPhysicsSystem{nullptr};
        CameraSystem* mCameraSystem{nullptr};
        BallSystem* mBallSystem{nullptr};
        NPC* mNPCSystem{nullptr};

        //Other resources:
        Shader *mShaders[4]{nullptr};
        Texture *mTextures[4]{nullptr};
        Material *mMaterials[4]{nullptr};
        DirectionalLight *mLight{nullptr};

    private:
        ECSManager(ECSManager const&);
        void operator=(ECSManager const&);
        static ECSManager *mInstance;

        void makeShaders();
        void makeTextures();
        void makeMaterials();
};

#endif // ECSMANAGER_H
