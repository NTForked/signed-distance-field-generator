
#include <vector>
#include <fstream>
#include <ctime>
#include <limits>
#include "Profiler.h"
#include "OBJReader.h"
#include "UniformGridSDF.h"
#include "MarchingCubes.h"
#include "ExportOBJ.h"
#include "Mesh.h"
#include "BVH.h"
#include "Surfaces.h"
#include "AABB.h"
#include "TriangleSDF.h"
#include "OctreeSDF.h"
#include "OpUnionSDF.h"
#include "OpIntersectionSDF.h"
#include "OpInvertSDF.h"
#include "SDFManager.h"
#include "FractalNoisePlaneSDF.h"

using std::vector;
using Ogre::Vector3;

void buildSDFAndMarch(const std::string& fileName, int maxDepth)
{
	auto octreeSDF = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh(fileName + ".obj"), maxDepth);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_" + fileName, octreeSDF);
}

void splitBuddha()
{
	auto octreeSDF = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	std::cout << "Buddha SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0)));
	octreeSDF->subtract(fractalNoiseSDF);
	std::cout << "Modified buddha SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit", octreeSDF);
}

void splitBuddha2()
{
	auto octreeSDF = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	std::cout << "Buddha SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0)));
	auto fractalNoiseOctreeSDF = OctreeSDF::sampleSDF(fractalNoiseSDF, octreeSDF->getAABB(), 8);
	std::cout << "Fractal noise SDF has " << fractalNoiseOctreeSDF->countNodes() << " nodes." << std::endl;
	auto ts = Profiler::timestamp();
	octreeSDF->intersectAlignedOctree(fractalNoiseOctreeSDF);
	Profiler::printJobDuration("Buddha plane intersection", ts);
	std::cout << "Split buddha SDF has " << octreeSDF->countNodes() << " nodes." <<  std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit", octreeSDF);
}

void testFractalNoisePlane()
{
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0)));
	auto octreeSDF = SDFManager::sampleOctreeSDF(fractalNoiseSDF, 8);
	std::cout << "Fractal noise SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_FractalNoise", octreeSDF);
}

int main()
{
	// buildSDFAndMarch("bunny_highres", 7);
	// buildSDFAndMarch("sphere", 7);
	//testFractalNoisePlane();
	splitBuddha2();
	//splitBuddha();
	while (true) {}
	return 0;
}