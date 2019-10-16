#include "camerasystem.h"
#include "transform.h"
#include "renderwindow.h"
#include "ecsmanager.h"
#include "mat4.h"
#include "entity.h"

CameraSystem* CameraSystem::mInstance = nullptr;
CameraComponent CameraSystem::mActiveCamera;

CameraSystem::CameraSystem(RenderWindow *glWindow)
    : mGlWindow{glWindow}
{
    mInstance = this;

    mECSManager = ECSManager::getInstance();
    mECSManager->mCameraComponents.push_back(CameraComponent());

    updateAspectRatio();
}

CameraSystem::~CameraSystem()
{

}

CameraSystem *CameraSystem::getInstance()
{
    return mInstance;
}

void CameraSystem::update()
{
    updateAspectRatio();
    updateCameraComponent();
    updatePerspectiveMatrices();
    updateViewMatrices();

    // Update mainCamera
    mActiveCamera = mECSManager->mCameraComponents[mActiveIndex];
}

void CameraSystem::updateViewMatrices()
{
    std::vector<CameraComponent> &cameraComponents = mECSManager->mCameraComponents;
    size_t size = cameraComponents.size();
    for(size_t i = 0; i < size; i++)
    {
        if(cameraComponents[i].orbitCamera)
        {
            gsl::Mat4 model = cameraComponents[i].cameraTransform.getModelMatrix();

            // Rotate mDirection based on Quaternion
            cameraComponents[i].mViewMatrix.lookAt(cameraComponents[i].cameraTransform.getPosition() +
                                                   cameraComponents[i].mPositionOffset*cameraComponents[i].mDistance,
                                                   cameraComponents[i].mTarget,
                                                   gsl::Vec3(0, 1, 0));
        }
        else
        {
            cameraComponents[i].mViewMatrix.lookAt(cameraComponents[i].cameraTransform.getPosition(),
                     cameraComponents[i].cameraTransform.getPosition()+cameraComponents[i].mDirection,
                     gsl::Vec3(0, 1, 0));

            gsl::Mat4 matrix = cameraComponents[i].cameraTransform.getModelMatrix();

            if(cameraComponents[i].inheritRotationX)
            {
                gsl::Vec3 forward = gsl::Vec3(matrix(2, 0), matrix(2, 1), matrix(2, 2));

                cameraComponents[i].mViewMatrix(2, 0) = forward.getX();
                cameraComponents[i].mViewMatrix(2, 1) = forward.getY();
                cameraComponents[i].mViewMatrix(2, 2) = forward.getZ();
            }
            if(cameraComponents[i].inheritRotationY)
            {
                gsl::Vec3 forward = gsl::Vec3(matrix(1, 0), matrix(1, 1), matrix(1, 2));

                cameraComponents[i].mViewMatrix(1, 0) = forward.getX();
                cameraComponents[i].mViewMatrix(1, 1) = forward.getY();
                cameraComponents[i].mViewMatrix(1, 2) = forward.getZ();
            }
            if(cameraComponents[i].inheritRotationZ)
            {
                gsl::Vec3 forward = gsl::Vec3(matrix(0, 0), matrix(0, 1), matrix(0, 2));

                cameraComponents[i].mViewMatrix(0, 0) = forward.getX();
                cameraComponents[i].mViewMatrix(0, 1) = forward.getY();
                cameraComponents[i].mViewMatrix(0, 2) = forward.getZ();
            }
        }
    }
}

void CameraSystem::updatePerspectiveMatrices()
{
    std::vector<CameraComponent> &cameraComponents = mECSManager->mCameraComponents;
    size_t size = cameraComponents.size();

    for(size_t i = 0; i < size; i++)
    {
        if(cameraComponents[i].persValuesChanged)
        {
            cameraComponents[i].mPerspectiveMatrix.perspective(cameraComponents[i].mFieldOfView,
                                                               cameraComponents[i].mAspectRatio,
                                                               cameraComponents[i].mNearPlane,
                                                               cameraComponents[i].mFarPlane);
            cameraComponents[i].persValuesChanged = false;
        }
    }
}

void CameraSystem::updateCameraComponent()
{
    std::vector<CameraComponent> &cameraComponents = mECSManager->mCameraComponents;
    size_t size = cameraComponents.size();

    for(size_t i = 0; i < size; i++)
    {
        // If you are supposed to follow the entity
        if(cameraComponents[i].followEntity)
        {
            TransformComponent* entityTransformComponent = Entity::getFirstTransformComponent(cameraComponents[i].ownerEntityID);

            if(entityTransformComponent)
            {
                cameraComponents[i].cameraTransform = entityTransformComponent->mTransform; // Position is your center
                cameraComponents[i].mTarget = cameraComponents[i].cameraTransform.getPosition();
            }
        }
        else
        {
            cameraComponents[i].cameraTransform.setPosition(gsl::Vec3(0, 5, -5));
            gsl::Mat4 model = cameraComponents[i].cameraTransform.getModelMatrix();
            cameraComponents[i].mDirection = gsl::Vec3(model(2, 0), model(2, 1), model(2, 2));
        }
    }
}

void CameraSystem::updateAspectRatio()
{
    if(mGlWindow)
    {
        GLfloat aspectRatio = static_cast<float>((mGlWindow->width())/static_cast<float>(mGlWindow->height()));
        if(aspectRatio > 0)
            mAspectRatio = aspectRatio;
        else
            mAspectRatio = 1.7777f;
    }

    //Updates aspectRatio for all cameraComponents. (Update only current camera?)
    std::vector<CameraComponent> &cameraComponents = mECSManager->mCameraComponents;

    size_t size = cameraComponents.size();
    for(size_t i = 0; i < size; i++)
    {
        cameraComponents[i].mAspectRatio = mAspectRatio;
    }
}

void CameraSystem::setCurrentCamera(GLuint index)
{
    mActiveIndex = index;
}

GLfloat CameraSystem::getAspectRatio() const
{
    return mAspectRatio;
}

CameraComponent& CameraSystem::getCurrentCamera()
{
    return mActiveCamera;
}

gsl::Vec3 CameraSystem::getUpVector(const CameraComponent& comp)
{
    return gsl::Vec3(comp.mViewMatrix(1, 0), comp.mViewMatrix(1, 1), comp.mViewMatrix(1, 2));
}

gsl::Vec3 CameraSystem::getRightVector(const CameraComponent& comp)
{
    return gsl::Vec3(comp.mViewMatrix(0, 0), comp.mViewMatrix(0, 1), comp.mViewMatrix(0, 2));
}

gsl::Vec3 CameraSystem::getForwardVector(const CameraComponent& comp)
{
    return gsl::Vec3(comp.mViewMatrix(2, 0), comp.mViewMatrix(2, 1), comp.mViewMatrix(2, 2));
}
