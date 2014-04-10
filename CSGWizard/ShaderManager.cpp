#include "ShaderManager.h"
#include <QDebug>

ShaderManager::ShaderManager()
{
    m_SolidLightingProgram = new QOpenGLShaderProgram();
    const char* vertexShaderCode =
            "#version 330 core\n"
            "layout(location = 0) in vec3 vertexPosition;\n"
            "uniform mat4 modelViewProj;\n"
            "void main() {\n"
            "   gl_Position = modelViewProj * vec4(vertexPosition, 1);\n"
            "}";
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderCode);

    const char* fragmentShaderCode =
            "#version 330 core\n"
            "out vec4 colorOut;\n"
            "void main() {\n"
            "   colorOut = vec4(0.6, 0.6, 0.6, 1);\n"
            "}";
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderCode);
    if (!m_SolidLightingProgram->link())
    {
        qDebug() << "Could not link solid lighting shader program!\n";
    }
}
