#ifndef GLMESH_H
#define GLMESH_H

#include <QOpenGLBuffer>
#include "../Core/OctreeSF.h"
#include "../Core/OctreeSDF.h"

class GLMesh
{
protected:
    std::shared_ptr<OctreeSF> m_Octree;
    GLuint m_VertexBuffer;
    GLuint m_IndexBuffer;
    unsigned int m_VertexBufferSize;
    unsigned int m_IndexBufferSize;

public:
    GLMesh(std::shared_ptr<OctreeSF> octree);
    ~GLMesh();

    void updateMesh();

    void render();
};

#endif // GLMESH_H
