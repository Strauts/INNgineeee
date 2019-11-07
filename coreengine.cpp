#include "coreengine.h"
#include "renderwindow.h"
#include "mainwindow.h"
#include "ecsmanager.h"
#include "entity.h"
#include "inputsystem.h"
#include "rendersystem.h"
#include "meshfactory.h"
#include "meshbase.h"
#include "controlledballscript.h"
#include "mousepicker.h"
#include "linedebug.h"
#include "camerasystem.h"

#include <QDebug>
#include <QStatusBar>

GLfloat CoreEngine::mDeltaTime = 0;

CoreEngine::CoreEngine(RenderWindow *renderWindow, MainWindow *mainWindow, QObject *parent)
    : QObject{parent}, mRenderWindow{renderWindow}, mMainWindow{mainWindow}
{
    mECSManager = new ECSManager(renderWindow);
    mLineDebugClass = new LineDebug();
    mMousePicker = new MousePicker(this);
    makeInitialEntities();
    startLoop();
}

GLfloat CoreEngine::getDeltaTime()
{
    return mDeltaTime;
}

void CoreEngine::makeInitialEntities()
{
    mLineDebugClass->createObject();
    mECSManager->makeEntity("Axis", "axis", gsl::COLOR);
    //mECSManager->makeEntity("Cube", "cube");
    Entity* temp = mECSManager->makeEntity("Sphere", "sphere", gsl::COLOR);

    InputComponent i;
    i.ownerEntityID = temp->mEntityID;
    mECSManager->mInputComponents.push_back(i);
    SphereColliderComponent sphere;
    sphere.radius = 1;
    sphere.ownerEntityID = temp->mEntityID;
    mECSManager->mSphereColliderComponents.push_back(sphere);

    // Add Camera Component
    CameraComponent c;
    c.ownerEntityID = temp->mEntityID;
    c.followEntity = true;
    c.orbitCamera = true;

    c.mDistance = 20;
    c.mParentTransformID = Entity::getFirstTransformComponent(c.ownerEntityID)->mComponentID;
    mECSManager->mCameraComponents.push_back(c);
    CameraSystem::getInstance()->setCurrentCamera(mECSManager->mCameraComponents.size()-1);

    mECSManager->mScriptComponents.push_back(new ControlledBallScriptComponent());
    mECSManager->mScriptComponents.back()->ownerEntityID = temp->mEntityID;

    RigidBodyComponent rigidbody;
    rigidbody.isKinematic = true;
    rigidbody.useGravity = true;
    rigidbody.detectCollisions = true;
    rigidbody.mass = 1;
    rigidbody.ownerEntityID = temp->mEntityID;
    mECSManager->mRigidBodyComponents.push_back(rigidbody);


    temp = mECSManager->makeEntity("Terrain", "koordinater.las", gsl::LIGHTCOLOR);
    MeshColliderComponent mcc;
    mcc.ownerEntityID = temp->mEntityID;
    mcc.mData = mECSManager->mMeshFactory->getMesh("koordinater.las");
    mcc.triangles = mcc.mData->mTriangles;
    mcc.numTriangles = mcc.mData->mNumberOfTriangles;
    mECSManager->mMeshColliderComponents.push_back(mcc);

    //Creating trophies with boxColliderComponent
    temp = mECSManager->makeEntity("1st_Throphy", "cube", gsl::COLOR);
    temp->getTransform().setPosition(35, 248, 90);
    BoxColliderComponent trophy1;
    trophy1.bSpline = true;
    trophy1.ownerEntityID = temp->mEntityID;
    mECSManager->mBoxColliderComponents.push_back(trophy1);

    temp = mECSManager->makeEntity("2nd_Throphy", "cube", gsl::COLOR);
    temp->getTransform().setPosition(50, 245, 90);
    BoxColliderComponent trophy2;
    trophy2.bSpline = true;
    trophy2.ownerEntityID = temp->mEntityID;
    mECSManager->mBoxColliderComponents.push_back(trophy2);

    temp = mECSManager->makeEntity("3rd_Throphy", "cube", gsl::COLOR);
    temp->getTransform().setPosition(50, 245, 65);
    BoxColliderComponent trophy3;
    trophy3.bSpline = true;
    trophy3.ownerEntityID = temp->mEntityID;
    mECSManager->mBoxColliderComponents.push_back(trophy3);

    temp = mECSManager->makeEntity("4th_Throphy", "cube", gsl::COLOR);
    temp->getTransform().setPosition(15, 255, 90);
    BoxColliderComponent trophy4;
    trophy4.bSpline = true;
    trophy4.ownerEntityID = temp->mEntityID;
    mECSManager->mBoxColliderComponents.push_back(trophy4);

    //Creating NPC and attaching NPC Component
    temp = mECSManager->makeEntity("NPC", "sphere", gsl::COLOR);
    NPCComponent npc;
    npc.ownerEntityID = temp->mEntityID;
    npc.mData = npc.getFirstMeshComponent()->mData;
    mECSManager->mNPCComponents.push_back(npc);

}

//The main game loop for the whole program
void CoreEngine::timerEvent(QTimerEvent *)
{
    GLint msSinceLastFrame = mTimeStart.restart();    //restart() returns ms since last restart.
    mDeltaTime = msSinceLastFrame / 1000.f;

    mRenderWindow->preRender(mDeltaTime);  //This just sets up the rendering

    mLineDebugClass->drawObject();  // Draw debug lines
    mECSManager->runCameraSystem(); // Update cameras before we draw...
    mECSManager->runRenderSystem(); //Render all meshes
    mECSManager->runInputSystem();
    mECSManager->runCollisionSystem();
    mECSManager->runScriptSystem(mDeltaTime);
    mECSManager->runPhysicsSystem(mDeltaTime);
    mECSManager->runBallSystem();
    mECSManager->runApplyPhysics(mDeltaTime);
    mECSManager->runNPCSystem(mDeltaTime);

    float intersection_distance;
    if (mRenderWindow->mMousePickerOn)
    {
        for(Entity* ent : mECSManager->mEntities)
        {
            TransformComponent* transformComp = Entity::getFirstTransformComponent(ent->mEntityID);

            if(transformComp)
            {
                if(mMousePicker->TestRayToObbIntersection(ent->mBoundingBoxRightUpFront, ent->mBoundingBoxLeftDownBack, transformComp->mTransform.getModelMatrix(), intersection_distance))
                {
                    std::cout << "Ray intersecting " << ent->getObjectName() << std::endl;;
                }
            }
        }
    }
    mRenderWindow->postRender();

    //The rest here is just to show the frame rate:
    calculateFramerate(msSinceLastFrame);
}

void CoreEngine::startLoop()
{
    mTimer.start(16, this);
    mTimeStart.start();
}

void CoreEngine::calculateFramerate(GLint msSinceLastFrame)
{
    static GLint frameCount{0};  //Counting actual frames for a quick "timer" for the statusbar

    if (mMainWindow)    //If no mainWindow, something is really wrong...
    {
        ++frameCount;
        if (frameCount > 60) //Once pr 60 frames =  update the message once pr second
        {
            //Showing some statistics in status bar
            mMainWindow->statusBar()->showMessage(" FrameDraw: " + QString::number(msSinceLastFrame) + " ms");
            frameCount = 0;     //Reset to show a new message in 60 frames
        }
    }
}
