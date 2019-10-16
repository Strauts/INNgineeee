#include "rendersystem.h"
#include "componentbase.h"
#include "material.h"
#include "shader.h"
#include "mat4.h"


RenderSystem::RenderSystem(std::vector<MeshComponent> *meshComponents, std::vector<MaterialComponent> *materialComponents, std::vector<TransformComponent> *transformComponents)
    : mMeshComponents{meshComponents}, mMaterialComponents{materialComponents}, mTransformComponents{transformComponents}
{
}

void RenderSystem::update()
{
    initializeOpenGLFunctions();

    GLuint meshNumber = mMeshComponents->size();

    for (GLuint x{0}; x < meshNumber; x++)
    {
        if (mMeshComponents->at(x).mDrawMesh)
        {
            mMaterialComponents->at(x).mMaterial->getShader()->Use();
            gsl::Mat4 tempMat4 =mTransformComponents->at(x).mTransform.getModelMatrix();

            mMaterialComponents->at(x).mMaterial->setMMatrix(tempMat4);

            glBindVertexArray(mMeshComponents->at(x).mVAO);

            if (mMeshComponents->at(x).mNumberOfIndices == 0)
            {
                glDrawArrays(mMeshComponents->at(x).mDrawType, 0, static_cast<GLsizei>(mMeshComponents->at(x).mNumberOfVertices));
            }
            else
            {
                glDrawElements(mMeshComponents->at(x).mDrawType, static_cast<GLsizei>(mMeshComponents->at(x).mNumberOfIndices), GL_UNSIGNED_INT, nullptr);
            }
        }

        //Bounding box:
        if (mMeshComponents->at(x).mOBB_VAO != 0 && mMeshComponents->at(x).mDrawBoundingBox)
        {
            glBindVertexArray(mMeshComponents->at(x).mOBB_VAO);
            glDrawElements(GL_LINE_LOOP, 16, GL_UNSIGNED_SHORT, nullptr);
        }

        glBindVertexArray(0);
    }

    glBindVertexArray(0);
}
