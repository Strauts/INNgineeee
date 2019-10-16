#ifndef CONTROLLEDBALL_H
#define CONTROLLEDBALL_H

#include "entity.h"
#include "vec3.h"
#include "gltypes.h"
#include "componentbase.h"

class ControlledBall : public Entity
{
    public:
        ControlledBall();
        void update(GLfloat deltaTime);

    private:
        GLfloat mMovementSpeed;
        GLfloat mRadius;
        gsl::Vec3 mVelocity;
        InputComponent *mInputcomp{nullptr};
};

#endif // CONTROLLEDBALL_H
