#include "ballsystem.h"
#include "ecsmanager.h"
#include "vec2.h"
#include "vertex.h"
#include "meshbase.h"
#include "coreengine.h"

BallSystem* BallSystem::mInstance = nullptr;

BallSystem::BallSystem()
{
    mInstance = this;
    mManager = ECSManager::getInstance();

    throphyCounter = 4;

}

void BallSystem::checkCollision()
{
    std::vector<SphereColliderComponent>& spheres = mManager->mSphereColliderComponents;
    std::vector<MeshColliderComponent>& terrain = mManager->mMeshColliderComponents;

    //cycles through collisions between the balls and terrains
    for(size_t i = 0; i < spheres.size(); i++)
        for(size_t k = 0; k < terrain.size(); k++)
        {
            //fetches the vertices of the terrain and stores them localy
            Vertex* vertices = terrain[k].mData->vertices();

            //fetches the modelMatrix and get's the inverse matrix
            gsl::Mat4 modelMatrise = terrain[k].getFirstTransformComponent()->mTransform.getModelMatrix();
            modelMatrise.inverse();

            gsl::Vec3 center = spheres[i].getFirstTransformComponent()->mTransform.getPosition();
            gsl::Vec3 localPos = (modelMatrise * gsl::Vec4(center.getX(), center.getY(), center.getZ(), 1)).toVector3D();

            //gets the triangles data with indexes from the struct
            Triangle* triData = terrain[k].triangles;

            gsl::Vec2 points[3], currentPoint;
            gsl::Vec3 area;
            int j = 0;
            Triangle currentTri;

            for(; j != -1;)
            {
                currentPoint = gsl::Vec2(localPos.getX(), localPos.getZ());
                currentTri = triData[j];
                points[0] = gsl::Vec2(vertices[currentTri.i].get_x(), vertices[currentTri.i].get_z());
                points[1] = gsl::Vec2(vertices[currentTri.j].get_x(), vertices[currentTri.j].get_z());
                points[2] = gsl::Vec2(vertices[currentTri.k].get_x(), vertices[currentTri.k].get_z());

                area = currentPoint.barycentricCoordinates(points[0], points[1], points[2]);

                if(area.getX() >= 0 && area.getY() >= 0 && area.getZ() >= 0)
                {
                    break;
                }

                if(j % 2 == 0)
                {
                    if(area.getX() >= 0 && area.getY() >= 0 && area.getZ() < 0)
                        j = currentTri.e3;
                    else if(area.getY() < 0)
                        j = currentTri.e2;
                    else if(area.getX() < 0)
                        j = currentTri.e1;
                }
                else
                {
                    if(area.getX() < 0 && area.getY() >= 0)
                    {
                        if(area.getX() < area.getY())
                            j = currentTri.e1;
                        else
                            j = currentTri.e2;
                    }
                    else if(area.getY() >= 0 && area.getZ() < 0)
                    {
                        if(area.getZ() < area.getX())
                            j = currentTri.e2;
                        else
                            j = currentTri.e1;
                    }
                    else if(area.getY() < 0 && area.getZ() >= 0)
                        j = currentTri.e3;
                }
            }
            if(j != -1)
            {
                gsl::Vec3 triNormal = MeshBase::normalFromTriangle(j, terrain[k].triangles, vertices);
                gsl::Vec3 triPosition = MeshBase::centerFromTriangle(j, terrain[k].triangles, vertices);

                float planeDistance = (localPos - triPosition)*gsl::Vec3(0,1,0);
                float distance = (localPos - triPosition)*triNormal;

                float coef = spheres[i].radius - distance;

                gsl::Vec3 sitPoint1 = center - gsl::Vec3(0,1,0) * planeDistance + triNormal*spheres[i].radius;
                gsl::Vec3 sitPoint2 = center + triNormal*coef;

                if(planeDistance < 0)
                {
                    RigidBodyComponent* rigidSphere = spheres[i].getFirstRigidbodyComponent();

                    rigidSphere->position = sitPoint1;
                    rigidSphere->velocity = gsl::Vec3();
                }

                if(std::fabs(distance) < spheres[i].radius)
                {
                    RigidBodyComponent* rigidSphere = spheres[i].getFirstRigidbodyComponent();

                    gsl::Vec3 normalForce = triNormal*(-rigidSphere->velocity * triNormal)*(rigidSphere->mass / CoreEngine::getDeltaTime());
                    rigidSphere->addForce(normalForce);

                    //tried to add fricion
                    //gsl::Vec3 acceleration = triNormal * (-gsl::GRAVITY/rigidSphere->mass);
                    //rigidSphere->totalForceApplied -= rigidSphere->velocity + (acceleration*CoreEngine::getDeltaTime());

                    rigidSphere->angularVelocity = gsl::Vec3(rigidSphere->velocity.getX(), 0, rigidSphere->velocity.getZ());
                    rigidSphere->position = sitPoint2;

                    spheres[i].touching = true;
                }
                else
                    spheres[i].touching = false;
            }
        }

    //Ball/Trophy pickUp
    std::vector<BoxColliderComponent>& b2 = mManager->mBoxColliderComponents;
    for(size_t i = 0; i < spheres.size(); i++)
        for(size_t k = 0; k < b2.size(); k++)
        {
            GLfloat distance = (spheres[i].getFirstTransformComponent()->mTransform.getPosition() - b2[k].getFirstTransformComponent()->mTransform.getPosition()).length();
            if(distance < 5.f && b2[k].bSpline == true)
            {
                //Removes the entity from the bspline calculations, and sends it out of player vison (unable to destroy it)
                b2[k].bSpline = false;
                std::cout << "throphy " << k << " picked up" << std::endl;
                b2[k].getFirstTransformComponent()->mTransform.setPosition(0, 2000, 0);

                //Checks how many trophies is left and which one got picked up, and tells the player
                throphyCounter--;
                if(throphyCounter > 0)
                    std::cout << "Only " << throphyCounter << " throphies left" << std::endl;
                else
                    std::cout << "You collected all the trophies!" << std::endl;
            };
        }
}

void BallSystem::applyPhysics(float deltaTime)
{
    std::vector<RigidBodyComponent>& rigidbodies = mManager->mRigidBodyComponents;
    size_t size = rigidbodies.size();
    TransformComponent* ball{nullptr};

    for(size_t i = 0; i<size; i++)
    {
        if(rigidbodies[i].getFirstSphereColliderComponent())
        {
            //sets selected transformComponent to the sphere
            ball = rigidbodies[i].getFirstTransformComponent();
            //applies gravity
            rigidbodies[i].totalForceApplied += gsl::Vec3(0, -gsl::GRAVITY, 0);
            rigidbodies[i].velocity += (rigidbodies[i].totalForceApplied * (1.f / rigidbodies[i].mass))*deltaTime;

            rigidbodies[i].totalForceApplied = gsl::Vec3();

            rigidbodies[i].position += rigidbodies[i].velocity*deltaTime;

            rigidbodies[i].angularVelocity = gsl::Vec3(rigidbodies[i].velocity.getX(), 0, rigidbodies[i].velocity.getZ());

        }

        //applies the changes to the transform
        if(ball != nullptr)
            ball->mTransform.setPosition(rigidbodies[i].position);
    }
}
