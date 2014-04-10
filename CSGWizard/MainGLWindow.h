#ifndef MAINGLWINDOW_H
#define MAINGLWINDOW_H

#include "GLWindow.h"

class MainGLWindow : public GLWindow
{
    Q_OBJECT
protected:
    virtual void initializeGL();
public:
    MainGLWindow();
};

#endif // MAINGLWINDOW_H
