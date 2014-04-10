#ifndef SHADERMANAGER_H
#define SHADERMANAGER_H

#include <QOpenGLShader>
#include <QOpenGLShaderProgram>

class ShaderManager
{
protected:
    QOpenGLShaderProgram* m_SolidLightingProgram;

public:
    ShaderManager();

    QOpenGLShaderProgram* getSolidLightingProgram() { return m_SolidLightingProgram; }

    static ShaderManager& getSingleton()
    {
        static ShaderManager eyeOfSauron;
        return eyeOfSauron;
    }
};

#endif // SHADERMANAGER_H
