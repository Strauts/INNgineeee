#include "npc.h"
#include "ecsmanager.h"
#include "vec2.h"
#include "vertex.h"
#include "meshbase.h"
#include "coreengine.h"
#include "gsl_math.h"
#include "bspline.h"

using namespace gsl;

NPC* NPC::mInstance = nullptr;


NPC::NPC()
{
    mInstance = this;
    mManager = ECSManager::getInstance();
}

void NPC::patrol(GLfloat deltatime)
{
    //Gather the trophie locations in the controlPoints vector
    std::vector<BoxColliderComponent>& boxCollider = mManager->mBoxColliderComponents;
    std::vector<gsl::Vec3> controllPoints;

    for (size_t i = 0; i < boxCollider.size(); i++)
    {
        if(boxCollider[i].bSpline == true)
        {
            controllPoints.push_back(boxCollider[i].getFirstTransformComponent()->mTransform.getPosition());
        }
    }

    //Use patrol() on the ai
    std::vector<NPCComponent>& npc = mManager->mNPCComponents;
    for (size_t i = 0; i < npc.size(); i++)
    {
        std::vector<float> knots;
        std::vector <gsl::Vec3> vs;
        BSpline spline;
        float sum = 0.f;

        //Setting up knots for the bSpline
        if(controllPoints.size() > 2)
        {
            for(uint j = 0; j < controllPoints.size()-2; j++)
            {
                vs.push_back(controllPoints[j+1] - controllPoints[j]);
                sum += vs.back().length();
            }

            knots.push_back(0.f);
            knots.push_back(0.f);
            knots.push_back(0.f);

            if (controllPoints.size() > 3)
            {
                knots.push_back(0.5f);
            }

            knots.push_back(1.f);
            knots.push_back(1.f);
            knots.push_back(1.f);
        }

        //Setting knots for the Bspline with 2 controllPoints
        else if(controllPoints.size() == 2)
        {
            Vec3 center = controllPoints[0];
            controllPoints.clear();

            Vec3 right = center + Vec3(20, 248, 0);
            Vec3 front = center + Vec3(0, 245, -20);
            Vec3 left = center + Vec3(-20, 245, 0);
            Vec3 back = center + Vec3(0, 255, 20);
            controllPoints.push_back(right);
            controllPoints.push_back(front);
            controllPoints.push_back(left);
            controllPoints.push_back(back);

            knots.push_back(0.f);
            knots.push_back(0.f);

            knots.push_back(0.f);
            knots.push_back(1.f/3.f);
            knots.push_back(2.f/3.f);
            knots.push_back(1.f);

            knots.push_back(1.f);
            knots.push_back(1.f);

        }

        //If we have 1 or less controllPoints remaining the ai doesn't move
        if(controllPoints.size() >= 2)
        {
            //running bSpline functions
            unsigned int interval = spline.getIntervalFromTime(npc[i].time, knots);
            Vec3 pos = spline.evaluate(2, 2, interval, npc[i].time, knots, controllPoints);

            //Gets the transform of the AI to make it more accecible
            TransformComponent* transform = npc[i].getFirstTransformComponent();
            std::vector<MeshColliderComponent>& t3 = mManager->mMeshColliderComponents;
            Mat4 terrainMat, invTerrainMat;

            for(size_t k = 0; k < t3.size(); k++)
            {
                terrainMat = t3[k].modelMatrix;
                invTerrainMat = terrainMat;
                invTerrainMat.inverse();

                //Barycentric Coordinates
                int triangle = t3[k].mData->findTriangleIndexFromWorldPosition(npc[i].triangleLastFrame, pos, invTerrainMat, t3[k].triangles, t3[k].mData->vertices());
                if(triangle != -1)
                {
                    npc[i].triangleLastFrame = triangle;

                    float y = (npc[i].mData->mBoundingBoxRightUpFront).length();

                    Vec3 triangleCenter = (terrainMat*Vec4(npc[i].mData->centerFromTriangle(triangle, t3[k].triangles, t3[k].mData->vertices()), 1)).toVector3D();
                    Vec3 triangleNormal = (terrainMat*Vec4(npc[i].mData->normalFromTriangle(triangle, t3[k].triangles, t3[k].mData->vertices()), 0)).toVector3D().normalized();
                    Vec3 centerToTriangleCenter = pos - triangleCenter;

                    float distance = Vec3::dot(centerToTriangleCenter, triangleNormal);

                    pos.setY(pos.getY() - distance + y);
                }

                //Gives speed and moves the AI
                transform->mTransform.setPosition(pos);

                Vec3 speed = transform->mTransform.getPosition() - npc[i].lastPosition;
                speed.setY(0.f);

                float speedMag = speed.length() / deltatime;

                if(speedMag > 12.f)
                {
                    npc[i].speed -= 0.005f;
                }
                else if(speedMag < 12.f)
                {
                    npc[i].speed += 0.005f;
                }

                speed.normalize();

                Vec3 rotAxis = speed^Vec3(0, 0, 1);

                double angle = gsl::rad2deg(static_cast<double>(std::acos(Vec3::dot(speed, Vec3(0, 0, -1)) / speed.length())));

                Quaternion rot(angle, rotAxis);
                transform->mTransform.setRotation(rot);

                if(npc[i].time >= knots[knots.size() - 1] - 0.001f)
                {
                    npc[i].time = 0.f;
                }
                else
                {
                    npc[i].time += deltatime*npc[i].speed;
                }
                npc[i].lastPosition = transform->mTransform.getPosition();
            }
        }
        else
        {
            break;
        }
    }
}
