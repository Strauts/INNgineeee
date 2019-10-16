#include "movementsystem.h"
#include "componentbase.h"


MovementSystem::MovementSystem(std::vector<MovementComponent> moveComp, std::vector<TransformComponent> transformComp)
    : mMovementComp{moveComp}, mTransformComp{transformComp}
{
}

void MovementSystem::init()
{
}

void MovementSystem::update(GLfloat deltatime)
{
    for (auto move : mMovementComp)
    {
        std::size_t searchCount{};
        //For loop which stops when it have found a component it is looking for and starts from the next component next time it is searching for new component.
        for (std::size_t i{searchCount}; i < mTransformComp.size(); i++)
        {
            if (move.ownerEntityID == mTransformComp[i].ownerEntityID)
            {
                //Checking if currentSpeed is higher then maxSpeed.
                if (move.currentSpeed > move.maxSpeed)
                {
                    move.currentSpeed = move.maxSpeed;
                }

                //Adding forwardVector to position with the length of the speed of the object.
                mTransformComp[i].mTransform.mLocalPosition += (move.forwardVector * move.currentSpeed * deltatime);

                //Reducing the speed over time.
                move.currentSpeed *= 0.98f;

                searchCount = i + 1;

                //Breaks the forloop once it found its needed component.
                break;
            }
        }
    }
}
