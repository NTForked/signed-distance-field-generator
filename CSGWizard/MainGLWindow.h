#ifndef MAINGLWINDOW_H
#define MAINGLWINDOW_H

#include <QResizeEvent>
#include "GLWindow.h"
#include "Camera.h"
#include "GLMesh.h"

class MainGLWindow : public GLWindow
{
    Q_OBJECT
protected:
    Camera m_Camera;

    Ogre::Vector3 m_TrackballCenter;
    float m_DistToCenter;
    float m_CamXRot;
    float m_CamYRot;

    bool m_Cutting;
    Ogre::Vector2 m_CuttingStartPos;

    QPoint m_MousePos;
    Ogre::Vector3 m_LastIntersectionPos;

    void updateCameraPos();

    std::shared_ptr<GLMesh> m_Mesh;

    BVHScene m_CollisionGeometry;

    Ogre::Vector2 getNDCMouseCoords(QMouseEvent* event) const;

    virtual void initializeGL() override;
    virtual void render() override;

    virtual void resizeEvent(QResizeEvent* event) override;

    virtual void wheelEvent(QWheelEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void mouseReleaseEvent(QMouseEvent* event) override;
    virtual void mouseMoveEvent(QMouseEvent* event) override;

    bool raycast(std::shared_ptr<OctreeSF> octree, QMouseEvent* event, Ogre::Vector3& rayIntersection);

    enum Tools
    {
        SUBTRACT_SPHERE,
        MERGE_SPHERE,
        CUT_PLANE
    };
    Tools m_CurrentTool;

public:
    MainGLWindow();

public slots:
    void setSubtractSphereMode() { m_CurrentTool = SUBTRACT_SPHERE; }
    void setMergeSphereMode() { m_CurrentTool = MERGE_SPHERE; }
    void setCutPlaneMode() { m_CurrentTool = CUT_PLANE; }
};

#endif // MAINGLWINDOW_H
