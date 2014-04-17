#-------------------------------------------------
#
# Project created by QtCreator 2014-04-10T15:05:42
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CSGWizard
TEMPLATE = app


SOURCES += main.cpp \
        MainWindow.cpp \
    ../Core/OctreeSF2.cpp \
    ../Core/OctreeSF.cpp \
    ../Core/OctreeSDF.cpp \
    ../Core/Mesh.cpp \
    ../Core/OgreMath/OgreVector3.cpp \
    ../Core/OgreMath/OgreQuaternion.cpp \
    ../Core/OgreMath/OgreMatrix4.cpp \
    ../Core/OgreMath/OgreMatrix3.cpp \
    ../Core/OgreMath/OgreMath.cpp \
    GLMesh.cpp \
    GLWindow.cpp \
    MainGLWindow.cpp \
    GLManager.cpp \
    Camera.cpp

HEADERS  += MainWindow.h \
    ../Core/VoronoiFracture.h \
    ../Core/VertexMerger.h \
    ../Core/Vertex.h \
    ../Core/Vector3i.h \
    ../Core/UniformGridSDF.h \
    ../Core/TriangleSDF.h \
    ../Core/TriangleLookupTable.h \
    ../Core/Triangle.h \
    ../Core/TransformSDF.h \
    ../Core/Surfaces.h \
    ../Core/Sphere.h \
    ../Core/SignedDistanceField.h \
    ../Core/SDFManager.h \
    ../Core/Ray.h \
    ../Core/Profiler.h \
    ../Core/Prerequisites.h \
    ../Core/OpUnionSDF.h \
    ../Core/OpInvertSDF.h \
    ../Core/OpIntersectionSDF.h \
    ../Core/OctreeSF2.h \
    ../Core/OctreeSF.h \
    ../Core/OctreeSDF.h \
    ../Core/OBJReader.h \
    ../Core/Mesh.h \
    ../Core/MathMisc.h \
    ../Core/MarchingCubes.h \
    ../Core/FracturePattern.h \
    ../Core/FractalNoisePlaneSDF.h \
    ../Core/FractalNoiseGenerator.h \
    ../Core/ExportSTL.h \
    ../Core/ExportPLY.h \
    ../Core/ExportOBJ.h \
    ../Core/BVHScene.h \
    ../Core/BVH.h \
    ../Core/BlockBasedSparseArray.h \
    ../Core/Area.h \
    ../Core/AABBSDF.h \
    ../Core/AABB.h \
    ../Core/OgreMath/OgreVector4.h \
    ../Core/OgreMath/OgreVector3.h \
    ../Core/OgreMath/OgreVector2.h \
    ../Core/OgreMath/OgreQuaternion.h \
    ../Core/OgreMath/OgreMatrix4.h \
    ../Core/OgreMath/OgreMatrix3.h \
    ../Core/OgreMath/OgreMath.h \
    ../Core/OgreMath/asm_math.h \
    GLMesh.h \
    GLWindow.h \
    MainGLWindow.h \
    GLManager.h \
    Camera.h

FORMS    += MainWindow.ui
