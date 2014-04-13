#ifndef CAMERA_H
#define CAMERA_H

#include "GLManager.h"
#include "../Core/OgreMath/OgreMatrix4.h"
#include "../Core/OgreMath/OgreVector3.h"

class Camera
{
protected:
    int m_Width;
    int m_Height;
    float m_Near;
    float m_Far;
    Ogre::Vector3 m_Origin;
    Ogre::Vector3 m_LookAt;
    Ogre::Matrix4 m_ProjectionMatrix;
    Ogre::Matrix4 m_ViewMatrix;
    Ogre::Matrix4 m_InverseViewMatrix;

    void computeViewMatrices();
    void computeProjectionMatrix();
public:
    Camera();

    void setViewportSize(int width, int height);

    void setNearFar(float nearClip, float farClip);

    void setTransform(const Ogre::Vector3& origin, const Ogre::Vector3& lookat);

    void updateUniforms(QOpenGLShaderProgram* program);

    const Ogre::Matrix4& getInverseViewMatrix() { return m_InverseViewMatrix; }
    const Ogre::Matrix4& getViewMatrix() { return m_ViewMatrix; }
    const Ogre::Matrix4& getProjectionMatrix() { return m_ProjectionMatrix; }
};

#endif // CAMERA_H
