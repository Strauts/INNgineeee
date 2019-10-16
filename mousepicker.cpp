#include "mousepicker.h"
#include "renderwindow.h"
#include "linedebug.h"
#include "material.h"
#include "camerasystem.h"
#include <QDebug>

MousePicker::MousePicker(CoreEngine* Owner)
{
    mOwner = Owner;
    mLineDebug = LineDebug::getInstance();
}

gsl::Vec3 MousePicker::getCurrentRay()
{
    return rayDirection;
}

void MousePicker::update()
{
    //Somehting happends to the projection and view-matrix from the camera, they do not send the right values.
    NewScreenPostToWorld();
}

bool MousePicker::TestRayToObbIntersection(Vec3 aabb_min, Vec3 aabb_max, Mat4 ModelMatrix, float& intersection_distance)
{

    float tMin = 0.0f;
    float tMax = 100000.0f;

    gsl::Vec3 OBBposition_worldspace(ModelMatrix(0, 3), ModelMatrix(1, 3), ModelMatrix(2, 3));
    gsl::Vec3  delta = OBBposition_worldspace - rayOrigin;
    qDebug() << delta.getX()<< delta.getY()<<delta.getZ();

    {
        gsl::Vec3 xaxis(ModelMatrix(0, 0),ModelMatrix(1, 0),ModelMatrix(2, 0));
        float e = xaxis * delta;
        float f = rayDirection * xaxis;

        float t1 = (e+aabb_min.x) /f;
        float t2 = (e+aabb_max.x) /f;

        if (t1 > t2)
        {
            float w = t1; t1=t2; t2 = w;
        }

        if (t2 <tMax)
            tMax = t2;
        if (t1 > tMin)
            tMin = t1;

        if (tMax < tMin)
            return false;
    }
    {
        gsl::Vec3 yaxis(ModelMatrix(0, 1),ModelMatrix(1, 1),ModelMatrix(3, 1));
        float e = yaxis * delta;
        float f = rayDirection * yaxis;

        float t1 = (e+aabb_min.y) /f;
        float t2 = (e+aabb_max.y) /f;

        if (t1 > t2)
        {
            float w = t1; t1=t2; t2 = w;
        }

        if (t2 <tMax)
            tMax = t2;
        if (t1 > tMin)
            tMin = t1;

        if (tMax < tMin)
            return false;
    }

    {
        gsl::Vec3 zaxis(ModelMatrix(0, 2),ModelMatrix(1, 2),ModelMatrix(2, 2));
        float e = zaxis * delta;
        float f = rayDirection * zaxis;

        float t1 = (e+aabb_min.z) /f;
        float t2 = (e+aabb_max.z) /f;

        if (t1 > t2)
        {
            float w = t1; t1=t2; t2 = w;
        }

        if (t2 <tMax)
            tMax = t2;
        if (t1 > tMin)
            tMin = t1;

        if (tMax < tMin)
            return false;
    }

    intersection_distance = tMin;
    return true;
}

void MousePicker::NewScreenPostToWorld()
{
    CameraComponent currentCamera = CameraSystem::getInstance()->getCurrentCamera();
    Mat4 mProjectInv = currentCamera.mPerspectiveMatrix;
    Mat4 mViewMatrix = currentCamera.mViewMatrix;
    Mat4 mViewInv = mViewMatrix;
    mViewInv.inverse();
    mProjectInv.inverse();

    float mouseX = mOwner->mRenderWindow->getMouseX();
    float mouseY = mOwner->mRenderWindow->getMouseY();

    int width = mOwner->mRenderWindow->width();
    int height = mOwner->mRenderWindow->height();

    gsl::Vec4 ray_orig(
                (2.f*mouseX / width) - 1.f,
                1.f - (2.f*mouseY / height),
                -1.f,
                1.f);
    gsl::Vec4 ray_clip (
                (2.f*mouseX / width) - 1.f,
                1.f - (2.f*mouseY / height),
                1.f,
                1.f);

    // Transform from NDC to eye coordinates
    gsl::Vec4 ray_eye = mProjectInv*ray_clip;
    gsl::Vec4 orig_eye = mProjectInv*ray_orig;
    // Normalize positions based on w component
    ray_eye = ray_eye * (1.f / ray_eye.getW());
    orig_eye = orig_eye * (1.f / orig_eye.getW());

    // Transform the ray into world space
    gsl::Vec4 ray_wor = mViewInv * ray_eye;

    // Transform the start position into world space
    gsl::Vec4 orig_wor = mViewInv * orig_eye;

    // Where the ray starts
    rayOrigin = orig_wor.toVector3D();

    // The direction of the ray
    rayDirection = (ray_wor - orig_wor).toVector3D().normalized();

    std::cout << "Ray World Vec: {" << ray_wor << "} " << std::endl;
    std::cout << "Ray Orig Pos: {" << orig_wor << "} " << std::endl;

    // Camera forward vector
    gsl::Vec3 offset = Vec3(-mViewMatrix(2, 0), -mViewMatrix(2, 1), -mViewMatrix(2, 2));
    offset.normalize();

    // Put a debug line right in front of the camera view
    mLineDebug->addLine(rayOrigin+offset*0.1f, rayOrigin + offset*0.1f + rayDirection*1000, Vec3(1, 0, 0));
}
