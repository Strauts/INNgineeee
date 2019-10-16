#ifndef CAMERASYSTEM_H
#define CAMERASYSTEM_H

#include "systembase.h"
#include "componentbase.h"
#include "gsl_math.h"


class RenderWindow;
class ECSManager;

class CameraSystem : public SystemBase
{
public:
    CameraSystem(RenderWindow *glWindow);
    ~CameraSystem() override;

    static CameraSystem* getInstance();
    static CameraComponent &getCurrentCamera();

    void update() override;
    void setCurrentCamera(GLuint index);
    void follow(CameraComponent* camera, gsl::Vec3 target);

    GLfloat getAspectRatio() const;
private:
    GLfloat mAspectRatio;
    RenderWindow* mGlWindow{nullptr};
    ECSManager* mECSManager{nullptr};

    static CameraSystem* mInstance;

    GLuint mActiveIndex;

    static CameraComponent mActiveCamera;

    void updateAspectRatio();
    void updateViewMatrices();
    void updateCameraComponent();
    void updatePerspectiveMatrices();

    gsl::Vec3 getUpVector(const CameraComponent &comp);
    gsl::Vec3 getRightVector(const CameraComponent& comp);
    gsl::Vec3 getForwardVector(const CameraComponent& comp);
};


#endif // CAMERASYSTEM_H
