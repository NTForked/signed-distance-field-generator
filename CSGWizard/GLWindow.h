#ifndef GLWINDOW_H
#define GLWINDOW_H

#include <QWindow>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLVertexArrayObject>

// Follows the example http://qt-project.org/doc/qt-5/qtgui-openglwindow-example.html#openglwindow-super-class

class GLWindow : public QWindow
{
    Q_OBJECT
protected:
    QOpenGLFunctions_4_3_Core* m_Funcs;
    QOpenGLContext* m_Context;
    QOpenGLVertexArrayObject* m_VAO;

    bool m_Initialized;
    bool m_UpdatePending;

    bool event(QEvent *event);

    void exposeEvent(QExposeEvent *event);

    void renderAndSwap();

    virtual void render() {}

    virtual void initializeGL() {}

public:
    explicit GLWindow(QOpenGLContext* shareContext = nullptr, QScreen* screen = nullptr);
    virtual ~GLWindow() {}

    void requestRedraw();

signals:

public slots:

};

#endif // GLWINDOW_H
