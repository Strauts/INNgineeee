#include "directionallight.h"
#include "quaternion.h"
#include "mat4.h"
#include "vec4.h"

DirectionalLight::DirectionalLight(gsl::Quaternion rotation, gsl::Vec3 ambientColor, gsl::Vec3 lightColor, gsl::Vec3 specularColor)
    : mRotation{rotation}, mAmbientColor{ambientColor}, mLightColor{lightColor}, mSpecularColor{specularColor}
{
}

gsl::Vec3 DirectionalLight::getNormal()
{
    return (mRotation.toRotationMatrix4() * gsl::Vec4(0, 0, 1, 0)).toVector3D();
}

void DirectionalLight::setRotation(const gsl::Quaternion &rotation)
{
    mRotation = rotation;
}
