#include "physicssystem.h"
#include "ecsmanager.h"
#include "componentbase.h"
#include "math_constants.h"

PhysicsSystem* PhysicsSystem::mInstance = nullptr;

PhysicsSystem::PhysicsSystem()
{
}

void PhysicsSystem::UpdateRigidBodies(float deltaTime)
{
    ECSManager* mECSManager = ECSManager::getInstance();

    // Update gravity on each rigidbody
    std::vector<RigidBodyComponent>& rigidbodies = mECSManager->mRigidBodyComponents;
    size_t size = rigidbodies.size();
    for(size_t i = 0; i < size; i++)
    {
        TransformComponent* t = rigidbodies[i].getFirstTransformComponent();

        if(rigidbodies[i].isKinematic && !rigidbodies[i].sleepOneFrame)
        {
            // Add gravity to the component...
            if(rigidbodies[i].useGravity)
            {
                rigidbodies[i].totalForceApplied += gsl::Vec3(0, -gsl::GRAVITY, 0);
            }

            // Change the velocity based on acceleration F = m*a, a = F/m, each frame
            rigidbodies[i].velocity += (rigidbodies[i].totalForceApplied * (1.f / rigidbodies[i].mass))*deltaTime;

            rigidbodies[i].totalForceApplied = gsl::Vec3();

            // Change the position based on velocity
            rigidbodies[i].position += rigidbodies[i].velocity*deltaTime;

            //rigidbodies[i].angularVelocity += (rigidbodies[i].totalAngularForce * (1.f / rigidbodies[i].mass))*deltaTime;

            rigidbodies[i].angularVelocity = gsl::Vec3(rigidbodies[i].velocity.getX(), 0, rigidbodies[i].velocity.getZ());
            gsl::Vec3 rotationAxis = rigidbodies[i].angularVelocity.normalized()^gsl::Vec3(0, 1, 0);
            rotationAxis.normalize();

            double mRadius = 1.0;

            double theta = (rigidbodies[i].angularVelocity.length() / mRadius) * (180.0 / gsl::PI);

            gsl::Quaternion deltaRot = gsl::Quaternion(theta*deltaTime, rotationAxis);

            deltaRot.normalize();

            t->mTransform.rotate(deltaRot);
        }
        else
        {
            rigidbodies[i].sleepOneFrame = false;
        }

        // Update transform position here
        t->mTransform.setPosition(rigidbodies[i].position);
    }
}
