#include "GLMesh.h"
#include "GLManager.h"

GLMesh::GLMesh(std::shared_ptr<OctreeSF> octree) : m_Octree(octree)
{
    GLManager::getSingleton().getGLFunctions()->glGenBuffers(1, &m_VertexBuffer);
    GLManager::getSingleton().getGLFunctions()->glGenBuffers(1, &m_IndexBuffer);
    updateMesh();
}

GLMesh::~GLMesh()
{
    GLManager::getSingleton().getGLFunctions()->glDeleteBuffers(1, &m_VertexBuffer);
    GLManager::getSingleton().getGLFunctions()->glDeleteBuffers(1, &m_IndexBuffer);
}

void GLMesh::updateMesh()
{
    m_Mesh = m_Octree->generateMesh();
    /*mesh->vertexBuffer.push_back(Ogre::Vector3(0, 0, 0));
    mesh->vertexBuffer.push_back(Ogre::Vector3(0, 1, 0));
    mesh->vertexBuffer.push_back(Ogre::Vector3(0, 0, 1));
    mesh->indexBuffer.push_back(0);
    mesh->indexBuffer.push_back(1);
    mesh->indexBuffer.push_back(2);*/
    m_VertexBufferSize = (int)m_Mesh->vertexBuffer.size();
    m_IndexBufferSize = (int)m_Mesh->indexBuffer.size();

    GLManager::getSingleton().getGLFunctions()->glBindBuffer(GL_ARRAY_BUFFER, m_VertexBuffer);
    GLManager::getSingleton().getGLFunctions()->glBufferData(GL_ARRAY_BUFFER, m_VertexBufferSize * sizeof(Vertex), m_Mesh->vertexBuffer.data(), GL_DYNAMIC_DRAW);

    GLManager::getSingleton().getGLFunctions()->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IndexBuffer);
    GLManager::getSingleton().getGLFunctions()->glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_IndexBufferSize * sizeof(unsigned int), m_Mesh->indexBuffer.data(), GL_DYNAMIC_DRAW);
}

void GLMesh::render()
{
    GLManager::getSingleton().getGLFunctions()->glBindBuffer(GL_ARRAY_BUFFER, m_VertexBuffer);
    GLManager::getSingleton().getGLFunctions()->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IndexBuffer);
    GLManager::getSingleton().getGLFunctions()->glVertexAttribPointer(
       0,               // attribute index
       3,               // number of components
       GL_FLOAT,        // component type type
       GL_FALSE,        // normalized?
       sizeof(Vertex),  // stride
       (void*)0         // array buffer offset
    );
    GLManager::getSingleton().getGLFunctions()->glVertexAttribPointer(
       1,               // attribute index
       3,               // number of components
       GL_FLOAT,        // component type type
       GL_FALSE,        // normalized?
       sizeof(Vertex),  // stride
       (void*)sizeof(Ogre::Vector3)         // array buffer offset
    );
    GLManager::getSingleton().getGLFunctions()->glDrawElements(GL_TRIANGLES, m_IndexBufferSize, GL_UNSIGNED_INT, (void*)0);
}
