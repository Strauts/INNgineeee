#ifndef MOVEMENTSYSTEM_H
#define MOVEMENTSYSTEM_H

#include "systembase.h"
#include <vector>

struct MovementComponent;
struct TransformComponent;

class MovementSystem : public SystemBase
{
    public:
        MovementSystem(std::vector<MovementComponent> MoveComp, std::vector<TransformComponent> TransformComp);
        virtual void init() override;
        virtual void update(GLfloat deltatime) override;

    private:
        std::vector<MovementComponent> mMovementComp;
        std::vector<TransformComponent> mTransformComp;
};

#endif // MOVEMENTSYSTEM_H
