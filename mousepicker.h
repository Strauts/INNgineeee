#ifndef MOUSEPICKER_H
#define MOUSEPICKER_H

#include "vec4.h"
#include "vec3.h"
#include "vec2.h"
#include "mat4.h"
#include "gltypes.h"
#include <QOpenGLFunctions_4_1_Core>
#include <QMouseEvent>

#include "gsl_math.h"
#include "camera.h"
#include "coreengine.h"

class RenderWindow;
class Camera;

class MousePicker
{
public:
    MousePicker(CoreEngine *Owner);
    gsl::Vec3 getCurrentRay();
    void update();

    bool TestRayToObbIntersection(gsl::Vec3 aabb_min, gsl::Vec3 aabb_max, gsl::Mat4 ModelMatrix, float &intersection_distance);

    void NewScreenPostToWorld();

protected:
    CoreEngine* mOwner{nullptr};

private:
    LineDebug* mLineDebug{nullptr};
    gsl::Vec3 rayDirection;
    gsl::Vec3 rayOrigin;

    gsl::Mat4 *viewMatrix;
    gsl::Mat4 *projectionMatrix;

    gsl::Vec4 toEyeCoords(gsl::Vec4 clipCoords);
    gsl::Vec3 toWorldCoords(gsl::Vec4 eyeCoords);

    gsl::Vec3 calculateMouseRay();
    gsl::Vec2 getNormalizedDeviceCoords(float mouseX, float mouseY);

};

#endif // MOUSEPICKER_H
