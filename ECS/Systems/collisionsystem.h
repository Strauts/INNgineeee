#ifndef COLLISIONSYSTEM_H
#define COLLISIONSYSTEM_H

#include "systembase.h"
#include "componentbase.h"

class ECSManager;

class CollisionSystem : public SystemBase
{
public:
    CollisionSystem();

    static CollisionSystem* getInstance();

    void checkCollisions();

    // Objects colliding with box
    void checkBoxInsideBox(BoxColliderComponent b1, BoxColliderComponent b2);
    void checkSphereInsideBox(SphereColliderComponent s1, BoxColliderComponent b2);
    void checkCyllinderInsideBox(CyllinderColliderComponent c1, BoxColliderComponent b2);
    void checkCapsuleInsideBox(CapsuleColliderComponent c1, BoxColliderComponent b2);
    void checkBoxInsideMesh(BoxColliderComponent b1, MeshColliderComponent m2);

    // Objects colliding with Sphere
    void checkSphereInsideSphere(SphereColliderComponent s1, SphereColliderComponent s2);
    void checkCyllinderInsideSphere(CyllinderColliderComponent c1, SphereColliderComponent s2);
    void checkCapsuleInsideSphere(CapsuleColliderComponent c1, SphereColliderComponent s2);
    void checkSphereInsideMesh(SphereColliderComponent &s1, MeshColliderComponent m2);
    void checkSphereInsideTrophy(SphereColliderComponent &s1, TrophyColliderComponent t2);

    // Objects colliding with Cyllinder
    void checkCyllinderInsideCyllinder(CyllinderColliderComponent c1, CyllinderColliderComponent c2);
    void checkCyllinderInsideMesh(CyllinderColliderComponent c1, MeshColliderComponent m2);

    // Meshes colliding
    void checkMeshInsideMesh(MeshColliderComponent m1, MeshColliderComponent m2);
    
    int findTriangleIndexFromWorldPosition(int startIndex, gsl::Vec3 localPos, Vertex *vertexData, Triangle *triangleData);

private:
    ECSManager* mManager{nullptr};

    static CollisionSystem* mInstance;
};

#endif // COLLISIONSYSTEM_H
