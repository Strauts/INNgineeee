#ifndef DIRECTIONALLIGHT_H
#define DIRECTIONALLIGHT_H

#include "vec3.h"
#include "quaternion.h"

class DirectionalLight
{
public:
    DirectionalLight(gsl::Quaternion rotation, gsl::Vec3 ambientColor = gsl::Vec3(0.2f, 0.2f, 0.2f), gsl::Vec3 lightColor = gsl::Vec3(1.f, 1.f, 1.f),
               gsl::Vec3 specularColor = gsl::Vec3(0.1f, 0.1f, 0.1f));

    gsl::Vec3 getNormal();
    void setRotation(const gsl::Quaternion &rotation);

    gsl::Vec3 mAmbientColor;
    gsl::Vec3 mLightColor;
    gsl::Vec3 mSpecularColor;

    float lightIntensity{0.8f};
private:
    gsl::Quaternion mRotation;
};

#endif // DIRECTIONALLIGHT_H
