#include "GLWindow.h"
#include "GLManager.h"
#include <QOpenGLFunctions_4_3_Core>
#include <QCoreApplication>

GLWindow::GLWindow(QOpenGLContext* shareContext, QScreen* screen) :
    QWindow(screen), m_UpdatePending(false), m_Initialized(false)
{
    setSurfaceType(OpenGLSurface);

    // Specify the format and create platform-specific surface
    QSurfaceFormat format;
    format.setDepthBufferSize( 24 );
    format.setMajorVersion( 4 );
    format.setMinorVersion( 3 );
    format.setSamples( 4 );
    format.setProfile( QSurfaceFormat::CoreProfile );
    setFormat(format);
    create();

    // Create an OpenGL context
    m_Context = new QOpenGLContext;
    m_Context->setFormat( format );
    if (shareContext)
        m_Context->setShareContext(shareContext);
    m_Context->create();

    // Make the context current on this window
    m_Context->makeCurrent(this);

    QOpenGLFunctions_4_3_Core* funcs = dynamic_cast<QOpenGLFunctions_4_3_Core*>(m_Context->versionFunctions());
    if ( !funcs ) {
        qWarning("Could not obtain OpenGL versions object");
        exit( 1 );
    }
    funcs->initializeOpenGLFunctions();
    GLManager::getSingleton().setGLFunctions(funcs);

    m_VAO = new QOpenGLVertexArrayObject(this);
    m_VAO->create();
    m_VAO->bind();
}

void GLWindow::exposeEvent(QExposeEvent *event)
{
    Q_UNUSED(event);

    if (isExposed())
    {
        if (!m_Initialized)
        {
            m_Initialized = true;
            m_Context->makeCurrent(this);
            initializeGL();
        }
        renderAndSwap();
    }
}

bool GLWindow::event(QEvent *event)
{
    switch (event->type()) {
    case QEvent::UpdateRequest:
        m_UpdatePending = false;
        renderAndSwap();
        return true;
    default:
        return QWindow::event(event);
    }
}

void GLWindow::renderAndSwap()
{
    m_Context->makeCurrent(this);
    render();
    m_Context->swapBuffers(this);
}

void GLWindow::requestRedraw()
{
    if (!m_UpdatePending)
    {
        m_UpdatePending = true;
        QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
    }
}
