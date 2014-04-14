#ifndef GLMESH_H
#define GLMESH_H

#include <QOpenGLBuffer>
#include "../Core/OctreeSF.h"
#include "../Core/OctreeSDF.h"

class GLMesh
{
protected:
    std::shared_ptr<OctreeSF> m_Octree;
    std::shared_ptr<Mesh> m_Mesh;
    GLuint m_VertexBuffer;
    GLuint m_IndexBuffer;
    unsigned int m_VertexBufferSize;
    unsigned int m_IndexBufferSize;

public:
    GLMesh(std::shared_ptr<OctreeSF> octree);
    ~GLMesh();

    void updateMesh();

    void render();

    std::shared_ptr<Mesh> getMesh() { return m_Mesh; }
    std::shared_ptr<OctreeSF> getOctree() { return m_Octree; }
};

#endif // GLMESH_H
