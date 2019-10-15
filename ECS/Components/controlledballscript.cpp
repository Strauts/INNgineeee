#include "controlledballscript.h"
#include "quaternion.h"
#include "vec3.h"
#include "vec4.h"
#include "material.h"
#include "camerasystem.h"
#include "ecsmanager.h"
#include "renderwindow.h"
#include "coreengine.h"
//#include "mat4.h"
#include <iostream>


ControlledBallScriptComponent::ControlledBallScriptComponent()
{
    std::cout << "Constructor in controlledball called!" << std::endl;

    RenderWindow* glWindow = ECSManager::getInstance()->mRenderWindow;
}

ControlledBallScriptComponent::~ControlledBallScriptComponent()
{
}

void ControlledBallScriptComponent::update(GLfloat deltaTime)
{
    RigidBodyComponent *r = getFirstRigidbodyComponent();
    InputComponent *i = getFirstInputComponent();
    CameraComponent *c = getFirstCameraComponent();
    SphereColliderComponent* sc = getFirstSphereColliderComponent();

    gsl::Mat4 viewMatrix = c->mViewMatrix;

    gsl::Vec3 camera_right = gsl::Vec3(viewMatrix(0, 0), viewMatrix(0, 1), viewMatrix(0, 2));
    gsl::Vec3 camera_fwd = (camera_right ^ gsl::Vec3(0, 1, 0)).normalized();

    gsl::Vec3 acceleration = (camera_right * i->mRight + camera_fwd * i->mForward)*1000*deltaTime;

    if(sc->touching)
        r->addForce(acceleration*r->mass);

    // Rotate the camera

    gsl::Vec2 mousePos = gsl::Vec2(i->mMousePosX, i->mMousePosY);

    gsl::Vec2 mouseDelta = mousePos - lastMousePos;

    if(lastMouseSet && i->mRightMouseBtn)
    {
        theta += mouseSensitivity*mouseDelta.getX()*deltaTime;
        phi += mouseSensitivity*mouseDelta.getY()*deltaTime;
    }
    theta -= i->mYaw*deltaTime*controllerSensitivity;
    phi += i->mPitch*deltaTime*controllerSensitivity;

    if(phi > (gsl::PI / 2.))
        phi = gsl::PI / 2. - 0.001;
    if(phi < (-gsl::PI / 2.))
        phi = -gsl::PI / 2. + 0.001;

    c->mPositionOffset = gsl::Vec3(-std::cos(phi)*std::sin(theta),
                                -std::sin(phi),
                                -std::cos(theta)*std::cos(phi));

    bool state = i->mButtonA;

    if(lastButtonState && !state && sc->touching)
    {
        // Jump
        r->addForce(gsl::Vec3(0, gsl::GRAVITY / CoreEngine::getDeltaTime(), 0));
    }

    lastButtonState = state;

    lastMousePos = mousePos;

    c->mDistance += 100*i->mButtonL2*deltaTime;

    c->mDistance -= 100*i->mButtonR2*deltaTime;

    if(!lastMouseSet)
        lastMouseSet = true;
}
