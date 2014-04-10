#include "ShaderManager.h"
#include <QDebug>

ShaderManager::ShaderManager()
{
    m_SolidLightingProgram = new QOpenGLShaderProgram();
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, "");
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, "");
    if (!m_SolidLightingProgram->link())
    {
        qDebug() << "Could not link solid lighting shader program!\n";
    }
}
