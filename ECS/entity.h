#ifndef ENTITY_H
#define ENTITY_H

#include "componentbase.h"
#include "gltypes.h"
#include "vec3.h"
#include <vector>

class Transform;
class Material;

class Entity
{
public:
        Entity();
        ~Entity();

        void createObject();

        Entity* getParent() const;
        void setParent(Entity *parent);

        static TransformComponent* getFirstTransformComponent(GLuint entityID);
        Transform& getTransform();

        void setMaterial(Material *material);
        Material* getMaterial();

        void setObjectName(const std::string &objectName);
        std::string getObjectName() const;

        GLuint mEntityID;       //Unique ID for every entity

        Entity *mParent{nullptr};
        Transform *mTransform{nullptr};
        MeshComponent* mMeshComp{nullptr};
        MaterialComponent* mMaterialComp{nullptr};

        gsl::Vec3 mBoundingBoxRightUpFront;
        gsl::Vec3 mBoundingBoxLeftDownBack;

        std::vector<ComponentBase> mComponents;

    private:
        std::string mObjectName;
        static GLuint ID;
};

#endif // ENTITY_H
