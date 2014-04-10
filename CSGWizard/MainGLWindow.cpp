#include "MainGLWindow.h"

MainGLWindow::MainGLWindow() : GLWindow()
{
}

void MainGLWindow::initializeGL()
{
    m_Funcs->glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
}
