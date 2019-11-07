#ifndef BALLSYSTEM_H
#define BALLSYSTEM_H

#include "systembase.h"
#include "componentbase.h"

class ECSManager;

class BallSystem : public SystemBase
{
public:
    BallSystem();

    static BallSystem* getInstance();

    void checkCollision();
    void applyPhysics(float deltaTime);
    size_t throphyCounter;

private:
    ECSManager* mManager{nullptr};
    static BallSystem* mInstance;
};

#endif // BALLSYSTEM_H
