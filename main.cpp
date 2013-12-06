
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
	octreeSDF->subtract(SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f));
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit", octreeSDF);
}

void testFractalNoisePlane()
{
	auto octreeSDF = SDFManager::sampleOctreeSDF(SDFManager::createFractalNoiseSDF(1.0f, 1.0f, 0.1f), 6);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_FractalNoise", octreeSDF);
}

int main()
{
	// buildSDFAndMarch("bunny_highres", 7);
	// buildSDFAndMarch("sphere", 7);
	// testFractalNoisePlane();
	splitBuddha();
	while (true) {}
	return 0;
}