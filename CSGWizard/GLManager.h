#ifndef GLMANAGER_H
#define GLMANAGER_H

#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions_4_3_Core>

class GLManager
{
protected:
    QOpenGLShaderProgram* m_SolidLightingProgram;

    QOpenGLFunctions_4_3_Core* m_Functions;

public:
    GLManager();

    QOpenGLShaderProgram* getSolidLightingProgram() { return m_SolidLightingProgram; }

    void setGLFunctions(QOpenGLFunctions_4_3_Core* functions) { m_Functions = functions; }
    QOpenGLFunctions_4_3_Core* getGLFunctions() { return m_Functions; }

    static GLManager& getSingleton()
    {
        static GLManager eyeOfSauron;
        return eyeOfSauron;
    }
};

#endif // SHADERMANAGER_H
