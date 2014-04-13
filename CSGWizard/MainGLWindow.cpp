#include "MainGLWindow.h"
#include "GLManager.h"
#include "../Core/SDFManager.h"
#include "../Core/OctreeSF.h"

MainGLWindow::MainGLWindow() : GLWindow()
{
}

void MainGLWindow::initializeGL()
{
    GLManager::getSingleton().getGLFunctions()->glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    m_Camera.setViewportSize(width(), height());

    m_CamXRot = 0;
    m_CamYRot = 0;
    m_TrackballCenter = Ogre::Vector3(0, 0, 0);
    m_DistToCenter = 1.0f;
    updateCameraPos();

    GLManager::getSingleton().getGLFunctions()->glEnableVertexAttribArray(0);
    GLManager::getSingleton().getGLFunctions()->glEnableVertexAttribArray(1);

    GLManager::getSingleton().getGLFunctions()->glEnable(GL_DEPTH_TEST);

    std::shared_ptr<Mesh> mesh = SDFManager::loadObjMesh("../Tests/bunny_highres.obj");
    TriangleMeshSDF_Robust meshSDF(std::make_shared<TransformedMesh>(mesh));
    auto octree = OctreeSF::sampleSDF(&meshSDF, 8);
    m_Mesh = std::make_shared<GLMesh>(octree);
}

void MainGLWindow::wheelEvent(QWheelEvent* event)
{
    float delta = -event->delta() * 0.001f;
    if (m_DistToCenter + delta > 0.0f)
        m_DistToCenter += delta;
    updateCameraPos();
}

void MainGLWindow::mouseMoveEvent(QMouseEvent* event)
{
    if (event->buttons() & Qt::MiddleButton)
    {
        QPoint delta = event->pos() - m_MousePos;
        if (event->modifiers() & Qt::SHIFT)
        {

            Ogre::Vector4 xDir = (m_Camera.getInverseViewMatrix() *  Ogre::Vector4(1, 0, 0, 0));
            Ogre::Vector4 yDir = (m_Camera.getInverseViewMatrix() * Ogre::Vector4(0, 1, 0, 0));
            m_TrackballCenter -= Ogre::Vector3(xDir.x, xDir.y, xDir.z).normalisedCopy() * delta.x() * 0.001f;
            m_TrackballCenter += Ogre::Vector3(yDir.x, yDir.y, yDir.z).normalisedCopy() * delta.y() * 0.001f;
        }
        else
        {
            float xRotDelta = delta.y() * 0.01f;
            if (m_CamXRot + xRotDelta > -Ogre::Math::PI * 0.5f && m_CamXRot + xRotDelta < Ogre::Math::PI * 0.5f)
                m_CamXRot += xRotDelta;
            m_CamYRot -= delta.x() * 0.01f;
        }
        updateCameraPos();
    }
    m_MousePos = event->pos();
}

void MainGLWindow::updateCameraPos()
{
    Ogre::Matrix3 xRotMat(1, 0, 0,
                       0, Ogre::Math::Cos(m_CamXRot), -Ogre::Math::Sin(m_CamXRot),
                       0, Ogre::Math::Sin(m_CamXRot), Ogre::Math::Cos(m_CamXRot));
    Ogre::Matrix3 yRotMat(Ogre::Math::Cos(m_CamYRot), 0, Ogre::Math::Sin(m_CamYRot),
                       0, 1, 0,
                       -Ogre::Math::Sin(m_CamYRot), 0, Ogre::Math::Cos(m_CamYRot));
    Ogre::Matrix3 rotMat = yRotMat * xRotMat;
    Ogre::Vector3 camDir = rotMat * Ogre::Vector3(0, 0, 1);
    camDir.normalise();
    m_Camera.setTransform(m_TrackballCenter - camDir * m_DistToCenter, m_TrackballCenter);
    requestRedraw();
}

void MainGLWindow::render()
{
    GLManager::getSingleton().getGLFunctions()->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    GLManager::getSingleton().getSolidLightingProgram()->bind();
    m_Camera.updateUniforms(GLManager::getSingleton().getSolidLightingProgram());
    m_Mesh->render();
}

void MainGLWindow::resizeEvent(QResizeEvent* event)
{
    glViewport(0, 0, event->size().width(), event->size().height());
    m_Camera.setViewportSize(event->size().width(), event->size().height());
}
