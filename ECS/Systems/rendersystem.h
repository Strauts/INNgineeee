#ifndef RENDERSYSTEM_H
#define RENDERSYSTEM_H

#include "systembase.h"
#include <QOpenGLFunctions_4_1_Core>

struct MeshComponent;
struct MaterialComponent;
struct TransformComponent;

//A system that renders a MeshComponent using a MaterialComponent
class RenderSystem : public SystemBase,  protected QOpenGLFunctions_4_1_Core
{
    public:
        RenderSystem(std::vector<MeshComponent> *meshComponents, std::vector<MaterialComponent> *materialComponents,
                     std::vector<TransformComponent> *transformComponents);

        void update() override;

        std::vector<MeshComponent> *mMeshComponents;
        std::vector<MaterialComponent> *mMaterialComponents;
        std::vector<TransformComponent> *mTransformComponents;
};

#endif // RENDERSYSTEM_H
