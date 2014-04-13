#include "Camera.h"

Camera::Camera()
{
    m_Near = 0.1f;
    m_Far = 500.0f;
}

void Camera::computeViewMatrices()
{
    Ogre::Vector3 zDir = (m_LookAt - m_Origin).normalisedCopy();
    Ogre::Vector3 xDir = zDir.crossProduct(Ogre::Vector3::UNIT_Y).normalisedCopy();
    Ogre::Vector3 yDir = xDir.crossProduct(zDir).normalisedCopy();
    Ogre::Matrix3 mat;
    mat.FromAxes(xDir, yDir, zDir);
    m_InverseViewMatrix = Ogre::Matrix4(mat);
    m_InverseViewMatrix.setTrans(m_Origin);
    m_ViewMatrix = m_InverseViewMatrix.inverse();
}

void Camera::computeProjectionMatrix()
{
    float aspectRatio = (float)m_Width / m_Height;
    float fov = 1.2f;
    float top = Ogre::Math::Tan(fov*0.5) * m_Near;
    float right = aspectRatio * top;
    m_ProjectionMatrix = Ogre::Matrix4(
                m_Near / right, 0, 0, 0,
                0,  m_Near / top, 0, 0,
                0, 0, (m_Far + m_Near) / (m_Far - m_Near), -(2 * m_Far * m_Near) / (m_Far - m_Near),
                0, 0, 1, 0);
}

void Camera::setNearFar(float nearClip, float farClip)
{
    m_Near = nearClip;
    m_Far = farClip;
    computeProjectionMatrix();
}

void Camera::setViewportSize(int width, int height)
{
    m_Width = width;
    m_Height = height;

    computeProjectionMatrix();
}

void Camera::setTransform(const Ogre::Vector3& origin, const Ogre::Vector3& lookat)
{
    m_Origin = origin;
    m_LookAt = lookat;
    computeViewMatrices();
}

void Camera::updateUniforms(QOpenGLShaderProgram* program)
{
    GLfloat viewMatrixData[4][4];
    GLfloat projMatrixData[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            viewMatrixData[j][i] = m_ViewMatrix[i][j];
            projMatrixData[j][i] = m_ProjectionMatrix[i][j];
        }
    }
    program->setUniformValue("viewMatrix", viewMatrixData);
    program->setUniformValue("projMatrix", projMatrixData);
}
