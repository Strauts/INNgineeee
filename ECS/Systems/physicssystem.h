#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H


class PhysicsSystem
{
public:
    PhysicsSystem();

    // Update the position of rigidbodies
    void UpdateRigidBodies(float deltaTime);

    static PhysicsSystem* getInstance();
private:
    static PhysicsSystem* mInstance;
};

#endif // PHYSICSSYSTEM_H
