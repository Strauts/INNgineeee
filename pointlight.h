#ifndef POINTLIGHT_H
#define POINTLIGHT_H

#include "vec3.h"

class PointLight
{
    public:
        PointLight(gsl::Vec3 position, gsl::Vec3 ambientColor = gsl::Vec3(0.2f, 0.2f, 0.2f), gsl::Vec3 diffuseColor = gsl::Vec3(1.f, 1.f, 1.f),
                   gsl::Vec3 specularColor = gsl::Vec3(0.1f, 0.1f, 0.1f));
        gsl::Vec3 getPosition() const;
        void setPosition(const gsl::Vec3 &position);

        gsl::Vec3 mAmbientColor;
        gsl::Vec3 mDiffuseColor;
        gsl::Vec3 mSpecularColor;

    private:
        gsl::Vec3 mPosition;
};

#endif // POINTLIGHT_H
