#include "collisionsystem.h"
#include "ecsmanager.h"
#include "vec2.h"
#include "vertex.h"
#include "meshbase.h"
#include "coreengine.h"

CollisionSystem* CollisionSystem::mInstance = nullptr;

CollisionSystem::CollisionSystem()
{
    mInstance = this;
    mManager = ECSManager::getInstance();
}

void CollisionSystem::checkCollisions()
{
    size_t numBoxColliderComponents = mManager->mBoxColliderComponents.size();
    size_t numSphereColliderComponents = mManager->mSphereColliderComponents.size();
    size_t numCyllinderColliderComponents = mManager->mCyllinderColliderComponents.size();
    size_t numCapsuleColliderComponents = mManager->mCapsuleColliderComponents.size();
    size_t numMeshColliderComponents = mManager->mMeshColliderComponents.size();
    size_t numTrophyColliderComponents = mManager->mTrophyColliderComponents.size();

    std::vector<BoxColliderComponent>& mBoxColliderComponents = mManager->mBoxColliderComponents;
    std::vector<SphereColliderComponent>& mSphereColliderComponents = mManager->mSphereColliderComponents;
    std::vector<CyllinderColliderComponent>& mCyllinderColliderComponents = mManager->mCyllinderColliderComponents;
    std::vector<CapsuleColliderComponent>& mCapsuleColliderComponents = mManager->mCapsuleColliderComponents;
    std::vector<MeshColliderComponent>& mMeshColliderComponents = mManager->mMeshColliderComponents;
    std::vector<TrophyColliderComponent>& mTrophyColliderComponents = mManager->mTrophyColliderComponents;

    size_t i, j;
    // BoxColliderComponents
    for(i = 0; i < numBoxColliderComponents; i++)
    {
        for(j = 0; j < numBoxColliderComponents; j++)
        {
            // Check if mBoxColliderComponents[j] is inside mBoxColliderComponents[i]
            // checkBoxInsideBox();
        }
    }

    for(i = 0; i < numBoxColliderComponents; i++)
    {
        for(j = 0; j < numSphereColliderComponents; j++)
        {
            // Check if mSphereColliderComponents[j] is inside mBoxColliderComponents[i]
            // checkSphereInsideBox();
        }
    }

    for(i = 0; i < numBoxColliderComponents; i++)
    {
        for(j = 0; j < numCyllinderColliderComponents; j++)
        {
            // Check if mCyllinderColliderComponents[j] is inside mBoxColliderComponents[i]
            // checkCyllinderInsideBox();
        }
    }

    for(i = 0; i < numBoxColliderComponents; i++)
    {
        for(j = 0; j < numCapsuleColliderComponents; j++)
        {
            // Check if mCapsuleColliderComponents[j] is inside mBoxColliderComponents[i]
            // checkCapsuleInsideBox();
        }
    }

    for(i = 0; i < numBoxColliderComponents; i++)
    {
        for(j = 0; j < numMeshColliderComponents; j++)
        {
            // Check if mMeshColliderComponents[j] is inside mBoxColliderComponents[i]
            // checkMeshInsideBox();
        }
    }

    // sphereColliderComponents
    for(i = 0; i < numSphereColliderComponents; i++)
    {
        for(j = 0; j < numSphereColliderComponents; j++)
        {
            // Check if mSphereComponents[j] is inside mSphereComponents[i]
            // checkSphereInsideSphere();
        }
    }

    for(i = 0; i < numSphereColliderComponents; i++)
    {
        for(j = 0; j < numCyllinderColliderComponents; j++)
        {
            // Check if mCyllinderComponents[j] is inside mSphereComponents[i]
            // checkCyllinderInsideSphere();
        }
    }

    for(i = 0; i < numSphereColliderComponents; i++)
    {
        for(j = 0; j < numCapsuleColliderComponents; j++)
        {
            // Check if mCapsuleComponents[j] is inside mSphereComponents[i]
            // checkCapsuleInsideSphere();
        }
    }

    for(i = 0; i < numSphereColliderComponents; i++)
    {
        for(j = 0; j < numMeshColliderComponents; j++)
        {
            // Check if mMeshColliderComponents[j] is inside mSphereComponents[i]
            checkSphereInsideMesh(mSphereColliderComponents[i], mMeshColliderComponents[j]);
        }
    }

    // cyllinderColliderComponents
    for(i = 0; i < numCyllinderColliderComponents; i++)
    {
        for(j = 0; j < numCyllinderColliderComponents; j++)
        {
            // Check if mCyllinderComponents[j] is inside mCyllinderComponents[i]
            // checkCyllinderInsideCyllinder();
        }
    }

    for(i = 0; i < numCyllinderColliderComponents; i++)
    {
        for(j = 0; j < numCapsuleColliderComponents; j++)
        {
            // Check if mCapsuleComponents[j] is inside mCyllinderComponents[i]
            // checkCapsuleInsideCyllinder();
        }
    }

    for(i = 0; i < numCyllinderColliderComponents; i++)
    {
        for(j = 0; j < numMeshColliderComponents; j++)
        {
            // Check if mMeshColliderComponents[j] is inside mCyllinderComponents[i]
            // checkMeshInsideCyllinder();
        }
    }

    // capsuleColliderComponents
    for(i = 0; i < numCapsuleColliderComponents; i++)
    {
        for(j = 0; j < numCapsuleColliderComponents; j++)
        {
            // Check if mCapsuleColliderComponents[j] is inside mCapsuleColliderComponents[i]
            // checkCapsuleInsideCapsule();
        }
    }

    for(i = 0; i < numCapsuleColliderComponents; i++)
    {
        for(j = 0; j < numMeshColliderComponents; j++)
        {
            // Check if mMeshColliderComponents[j] is inside mCapsulCollidereComponents[i]
            // checkMeshInsideCapsule();
        }
    }

    // meshColliderComponents
    for(i = 0; i < numMeshColliderComponents; i++)
    {
        for(j = 0; j < numMeshColliderComponents; j++)
        {
            // Check if mMeshColliderComponents[j] is inside numMeshColliderComponents[i]
            // checkMeshInsideMesh();
        }
    }
}

void CollisionSystem::checkBoxInsideBox(BoxColliderComponent b1, BoxColliderComponent b2)
{
}

void CollisionSystem::checkSphereInsideBox(SphereColliderComponent s1, BoxColliderComponent b2)
{
}

void CollisionSystem::checkCyllinderInsideBox(CyllinderColliderComponent c1, BoxColliderComponent b2)
{
}

void CollisionSystem::checkCapsuleInsideBox(CapsuleColliderComponent c1, BoxColliderComponent b2)
{
}

void CollisionSystem::checkBoxInsideMesh(BoxColliderComponent b1, MeshColliderComponent m2)
{
}

void CollisionSystem::checkSphereInsideSphere(SphereColliderComponent s1, SphereColliderComponent s2)
{
}

void CollisionSystem::checkCyllinderInsideSphere(CyllinderColliderComponent c1, SphereColliderComponent s2)
{
}

void CollisionSystem::checkCapsuleInsideSphere(CapsuleColliderComponent c1, SphereColliderComponent s2)
{
}

// Use barycentric coordinates to check which triangle the sphere center is inside, or if the sphere is outside the mesh
int CollisionSystem::findTriangleIndexFromWorldPosition(int startIndex, gsl::Vec3 localPos, Vertex* vertexData, Triangle* triangleData)
{
    gsl::Vec2 p1, p2, p3, currentPoint;
    gsl::Vec3 v3_p1, v3_p2, v3_p3, area;

    int i = 0;

    Triangle t;

    gsl::Vec3 normal, triangleCenter;

    // Find out which triangle the position is
    for(; i != -1;)
    {
        t = triangleData[i];
        v3_p1 = vertexData[t.i].get_xyz();
        v3_p2 = vertexData[t.j].get_xyz();
        v3_p3 = vertexData[t.k].get_xyz();

        normal = MeshBase::normalFromTriangle(i, triangleData, vertexData);
        triangleCenter = MeshBase::centerFromTriangle(i, triangleData, vertexData);

        float dist = (localPos - triangleCenter)*normal;

        p1 = gsl::Vec2(v3_p1.getX(), v3_p1.getZ());
        p2 = gsl::Vec2(v3_p2.getX(), v3_p2.getZ());
        p3 = gsl::Vec2(v3_p3.getX(), v3_p3.getZ());

        currentPoint = gsl::Vec2(localPos.getX(), localPos.getZ());
        area = currentPoint.barycentricCoordinates(p1, p2, p3);

        if(area.getX() >= 0 && area.getY() >= 0 && area.getZ() >= 0)
        {
            // The point is inside a triangle...
            // We should check the distance to the triangle from the center
            return i;
        }

        // We are in the first triangle of the quad
        if(i % 2 == 0)
        {
            if(area.getX() >= 0 && area.getY() >= 0 && area.getZ() < 0)
                i = t.e3; // Go right
            else if(area.getY() < 0)
                i = t.e2; // Go down
            else if(area.getX() < 0)
                i = t.e1; // Go left
        }
        // We are in the second triangle in the quad
        else
        {
            if(area.getX() < 0 && area.getY() >= 0)
            {
                if(area.getX() < area.getZ())
                    i = t.e1; // Go right
                else
                    i = t.e2; // Go up
            }
            else if(area.getY() >= 0 && area.getZ() < 0)
            {
                if(area.getZ() < area.getX())
                    i = t.e2; // Go up
                else
                    i = t.e1; // Go right
            }
            else if(area.getY() < 0 && area.getZ() >= 0)
                i = t.e3; // Go left
        }
    }
}

void CollisionSystem::checkSphereInsideMesh(SphereColliderComponent& s1, MeshColliderComponent m2)
{
    Vertex* vertices = m2.mData->vertices();
    // Translate the world position of the sphere into the local-space of the terrain
    // TODO: start searching from different triangles based on the distance to each corner, and center
    gsl::Mat4 modelMatrix = m2.getFirstTransformComponent()->mTransform.getModelMatrix();
    gsl::Vec3 sphereCenter = s1.getFirstTransformComponent()->mTransform.getPosition();

    modelMatrix.inverse();
    gsl::Vec3 localPos = (modelMatrix * gsl::Vec4(sphereCenter.getX(), sphereCenter.getY(), sphereCenter.getZ(), 1)).toVector3D();

    int index = findTriangleIndexFromWorldPosition(0, sphereCenter, vertices, m2.triangles);

    // Here we know which triangle we are in
    if(index != -1)
    {
        //std::cout << "We found triangle: " << index << std::endl;
        // check if the distance to the triangle plane from the center
        // is less than the radius, the the distance is negative, the sphere
        // is on the other side of the plane

        gsl::Vec3 faceNormal = MeshBase::normalFromTriangle(index, m2.triangles, vertices);
        gsl::Vec3 facePosition = MeshBase::centerFromTriangle(index, m2.triangles, vertices);

        //std::cout << "FaceNormal: {" << faceNormal << "} " << std::endl;

        // Distance from center to plane along up axis is
        float pdistance = (localPos - facePosition)*gsl::Vec3(0, 1, 0);
        float distance = (localPos - facePosition)*faceNormal;

        float coef = s1.radius - distance;

        gsl::Vec3 sitPoint1 = sphereCenter - gsl::Vec3(0, 1, 0) * pdistance + faceNormal*s1.radius;
        gsl::Vec3 sitPoint2 = sphereCenter + faceNormal*coef;

        //std::cout << "Vertex: " << facePosition << std::endl;

        // Ball is on other side of triangle...
        if(pdistance < 0)
        {
            RigidBodyComponent* r1 = s1.getFirstRigidbodyComponent();
            // Place the sphere on top of the triangle

            r1->position = sitPoint1; // 1 = radius
            r1->velocity = gsl::Vec3();
        }
        // Ball is on the right side of the triangle...
        if(std::fabs(distance) < s1.radius)
        {
            RigidBodyComponent* r1 = s1.getFirstRigidbodyComponent();
            // Add a foce to the sphere that cancels out the current force downwards
            //r1->velocity = r1->velocity - faceNormal*(r1->velocity * faceNormal);
            gsl::Vec3 up(0, 1, 0);
            gsl::Vec3 normalForce = faceNormal*(-r1->velocity * faceNormal)*(r1->mass / CoreEngine::getDeltaTime());
            r1->addForce(normalForce);

            // Wheter you should add torque should depend on drag
            r1->angularVelocity = gsl::Vec3(r1->velocity.getX(), 0, r1->velocity.getZ());

            r1->position = sitPoint2; // 1 = radius

            s1.touching = true;
        }
        else
        {
            s1.touching = false;
        }
    }
    else
    {
        // If we get over here, the sphere center is not inside a triangle or we are at an edge...
        // We if we are here, we need to check the if one of the four corners of the sphere
        // is inside a triangle in the mesh...
    }
}

void CollisionSystem::checkCyllinderInsideCyllinder(CyllinderColliderComponent c1, CyllinderColliderComponent c2)
{
}

void CollisionSystem::checkCyllinderInsideMesh(CyllinderColliderComponent c1, MeshColliderComponent m2)
{
}

void CollisionSystem::checkMeshInsideMesh(MeshColliderComponent m1, MeshColliderComponent m2)
{
}
