#include "GLManager.h"
#include <QDebug>

GLManager::GLManager()
{
    m_SolidLightingProgram = new QOpenGLShaderProgram();
    const char* vertexShaderCode =
            "#version 330 core\n"
            "layout(location = 0) in vec3 vertexPosition;\n"
            "layout(location = 1) in vec3 vertexNormal;\n"
            "uniform mat4 viewMatrix;\n"
            "uniform mat4 projMatrix;\n"
            "out vec3 normalVS;\n"
            "void main() {\n"
            "   gl_Position = (projMatrix * viewMatrix) * vec4(vertexPosition, 1);\n"
            "   normalVS = (viewMatrix * vec4(vertexNormal, 0)).xyz;\n"
            "}";
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderCode);

    const char* fragmentShaderCode =
            "#version 330 core\n"
            "out vec4 colorOut;\n"
            "in vec3 normalVS;\n"
            "void main() {\n"
            "   vec3 normal = normalize(normalVS);\n"
            "   vec3 materialColor = vec3(0.9f, 0.9f, 0.9f);\n"
            "   float ambient = 0.2f;\n"
            "   float diffuse = clamp(-normal.z * 0.5f, 0, 1);\n"
            "   float lighting = clamp(ambient + diffuse, 0, 1);\n"
            "   \n"
            "   colorOut = vec4(materialColor * lighting, 1);\n"
            "}";
    m_SolidLightingProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderCode);
    if (!m_SolidLightingProgram->link())
    {
        qDebug() << "Could not link solid lighting shader program!\n";
    }
}
