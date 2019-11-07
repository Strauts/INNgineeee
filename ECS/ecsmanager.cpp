#include "ecsmanager.h"
#include "rendersystem.h"
#include "meshfactory.h"
#include "entity.h"
#include "constants.h"
#include "material.h"
#include "renderwindow.h"
#include "entity.h"
#include "inputsystem.h"
#include "meshbase.h"
#include "shader.h"
#include "texture.h"
#include "colormaterial.h"
#include "texturematerial.h"
#include "lightcolormaterial.h"
#include "pointlight.h"
#include "directionallight.h"
#include "collisionsystem.h"
#include "scriptsystem.h"
#include "physicssystem.h"
#include "camerasystem.h"
#include <cassert>
#include "npc.h"
#include "ballsystem.h"


ECSManager* ECSManager::mInstance = nullptr;

ECSManager::ECSManager(RenderWindow *renderWindow) : mRenderWindow{renderWindow}
{
    mInstance = this;

    mRenderSystem = new RenderSystem(&mMeshComponents, &mMaterialComponents, &mTransformComponents);
    mInComp = new InputComponent; //OEF: this should belong to an Entity
    mInputSystem = new InputSystem();
    mCollisionSystem = new CollisionSystem();
    mScriptSystem = new ScriptSystem();
    mPhysicsSystem = new PhysicsSystem();
    mCameraSystem = new CameraSystem(renderWindow);
    mMeshFactory = new MeshFactory;
    mNPCSystem = new NPC();
    mBallSystem = new BallSystem();

    //Compile shaders:
    makeShaders();

    //make textures
    makeTextures();

    //Make directional light
    mLight = new DirectionalLight(gsl::Quaternion(30, gsl::Vec3(1, 0, 0)), gsl::Vec3{0.1f, 0.1f, 0.1f}, gsl::Vec3{1.f, 1.f, 1.f}, gsl::Vec3{0.7f, 0.7f, 0.7f});

    //Make all materials
    makeMaterials();
}

ECSManager* ECSManager::getInstance()
{
    assert(mInstance != nullptr);   //If done in correct order, this should not happen

    return mInstance;
}

void ECSManager::runRenderSystem()
{
    mRenderSystem->update();    //DeltaTime not needed
    mRenderWindow->checkForGLerrors();
}

void ECSManager::runInputSystem()
{
    mInputSystem->update();
}

void ECSManager::runCollisionSystem()
{
    mCollisionSystem->checkCollisions();
}

void ECSManager::runScriptSystem(GLfloat deltaTime)
{
    mScriptSystem->update(deltaTime);
}

void ECSManager::runPhysicsSystem(GLfloat deltaTime)
{
    mPhysicsSystem->UpdateRigidBodies(deltaTime);
}

void ECSManager::runCameraSystem()
{
    mCameraSystem->update();
}

void ECSManager::runBallSystem()
{
    mBallSystem->checkCollision();
}

void ECSManager::runNPCSystem(GLfloat deltaTime)
{
    mNPCSystem->patrol(deltaTime);
}

void ECSManager::runApplyPhysics(GLfloat deltaTime)
{
    mBallSystem->applyPhysics(deltaTime);
}

Entity* ECSManager::makeEntity(std::string entityName)
{
    Entity *tempEntity = new Entity();
    mEntities.push_back(tempEntity);

    if (entityName.length() == 0)
        entityName = "UnNamed";

    tempEntity->setObjectName(entityName);

    return tempEntity;
}

Entity* ECSManager::makeEntity(std::string entityName, std::string meshName, gsl::ShaderTypes shader)
{
    if (meshName.length() == 0)
    {
        std::cout << "You tried to load a mesh with no meshName, returned nullptr." << std::endl;
        return nullptr;
    }

    Entity *tempEntity = new Entity();
    mEntities.push_back(tempEntity);
    tempEntity->setObjectName(entityName);

    //If a mesh, make the three components it has to have:
    mMeshComponents.push_back(mMeshFactory->getMeshComponent(meshName));
    mRenderWindow->checkForGLerrors();
    mMeshComponents.back().ownerEntityID = tempEntity->mEntityID;
    mMeshComponents.back().componentType = gsl::MESH;

    mMaterialComponents.push_back(MaterialComponent());
    mRenderWindow->checkForGLerrors();
    mMaterialComponents.back().ownerEntityID = tempEntity->mEntityID;
    mMaterialComponents.back().componentType = gsl::MATERIAL;
    mMaterialComponents.back().mMaterial = mMaterials[shader];
    mRenderWindow->checkForGLerrors();
    mMaterialComponents.back().mMaterial->setupShader(mMeshComponents.back().mVAO);
    mRenderWindow->checkForGLerrors();

    //Make Bounding Box
    mMeshFactory->getMesh(meshName)->makeBoundingBox();
    mRenderWindow->checkForGLerrors();
    mMeshComponents.back().mOBB_VAO = mMeshFactory->getMesh(meshName)->mOBB_VAO;

    tempEntity->mBoundingBoxLeftDownBack = mMeshFactory->getMesh(meshName)->mBoundingBoxLeftDownBack;
    tempEntity->mBoundingBoxRightUpFront = mMeshFactory->getMesh(meshName)->mBoundingBoxRightUpFront;
    tempEntity->mMeshComp = &mMeshComponents.back();

    TransformComponent t;
    t.ownerEntityID = tempEntity->mEntityID;
    mTransformComponents.push_back(t);

    tempEntity->mTransform = &mTransformComponents.back().mTransform;

    mRenderWindow->checkForGLerrors();

    return tempEntity;
}

void ECSManager::makeShaders()
{
    mShaders[gsl::COLOR] = new Shader("plainvertex.vert", "plainfragment.frag");
    mShaders[gsl::TEXTURE] = new Shader("textureshader.vert", "textureshader.frag");
    mShaders[gsl::LIGHTCOLOR] = new Shader("lightcolor.vert", "lightcolor.frag");
    mShaders[gsl::NORMAL] = new Shader("lightnormal.vert", "lightnormal.frag");
}

void ECSManager::makeTextures()
{
    mTextures[0] = new Texture("hund.bmp");
    mTextures[1] = new Texture("uvtemplate.bmp");
    //mTextures[2] = new Texture("normal_map.bmp");
}

void ECSManager::makeMaterials()
{
    mMaterials[gsl::COLOR] = new ColorMaterial(mShaders[gsl::COLOR]);
    mRenderWindow->checkForGLerrors();

    //Make Texture material
    TextureMaterial *texMat = new TextureMaterial(mShaders[gsl::TEXTURE]);
    mMaterials[gsl::TEXTURE] = texMat;
    texMat->setTexture1(mTextures[0]);
    texMat->setTexture2(mTextures[1]);
    mRenderWindow->checkForGLerrors();

    //Make LightColor material
    LightColorMaterial *lightColorMat = new LightColorMaterial(mShaders[gsl::LIGHTCOLOR], mLight);
    mMaterials[gsl::LIGHTCOLOR] = lightColorMat;
    lightColorMat->setColors(gsl::Vec3(0.2f, 0.2f, 0.2f), gsl::Vec3(0.1f, 0.8f, 1.f), gsl::Vec3(1.f, 0.f, 0.f), 64.f);
    mRenderWindow->checkForGLerrors();
}

DirectionalLight* ECSManager::getLight() const
{
    return mLight;
}
