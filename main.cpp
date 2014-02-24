
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
	std::shared_ptr<Mesh> mesh = SDFManager::loadObjMesh(fileName);
	auto ts = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(std::make_shared<TriangleMeshSDF_Robust>(std::make_shared<TransformedMesh>(mesh)), maxDepth);
	Profiler::printJobDuration("SDF import " + fileName, ts);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_" + fileName, octreeSDF);
}

void testMeshImport()
{
	buildSDFAndMarch("bunny.capped.obj", 8);		// 5.441 seconds
	buildSDFAndMarch("buddha2.obj", 8);				// 17.33 seconds
}

void splitBuddha()
{
	auto part1 = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0)));
	part1->subtract(fractalNoiseSDF);
	std::cout << "Buddha part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->intersect(fractalNoiseSDF);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit2", part2);
}

void splitBunny()
{
	auto part1 = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	auto part2 = part1->clone();
	std::cout << "Bunny SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(3.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.1f), Ogre::Vector3(1, 0, 0)));
	part1->subtract(fractalNoiseSDF);
	std::cout << "Bunny part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->intersect(fractalNoiseSDF);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BunnySplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BunnySplit2", part2);
}

void testBVHResampling()
{
	auto bunny = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	bunny->generateTriangleCache();
	auto ts = Profiler::timestamp();
	Ogre::Matrix4 transform = (Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0)));
	transform.setTrans(Ogre::Vector3(1.2f, 5.1f, 3.4f));
	auto bunnyRotated = SDFManager::sampleOctreeSDF(std::make_shared<TransformSDF>(bunny, transform), 8);
	Profiler::printJobDuration("Bunny resampling", ts);
	SDFManager::exportSampledSDFAsMesh("sdfOctree_BunnyResampled", bunnyRotated);
}

void splitBuddha2()
{
	auto part1 = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 9);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.15f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.1f), Ogre::Vector3(1, 0, 0)));
	auto fractalNoiseOctreeSDF = OctreeSDF::sampleSDF(fractalNoiseSDF, part1->getAABB(), 9);
	std::cout << "Fractal noise SDF has " << fractalNoiseOctreeSDF->countNodes() << " nodes." << std::endl;
	auto ts = Profiler::timestamp();
	part1->intersectAlignedOctree(fractalNoiseOctreeSDF);
	Profiler::printJobDuration("Buddha plane intersection", ts);
	std::cout << "Split buddha part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->subtractAlignedOctree(fractalNoiseOctreeSDF);
	std::cout << "Split buddha part2 has " << part2->countNodes() << " nodes." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit2", part2);
}

void testFractalNoisePlane()
{
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0)));
	auto octreeSDF = SDFManager::sampleOctreeSDF(std::static_pointer_cast<SignedDistanceField3D>(fractalNoiseSDF), 8);
	std::cout << "Fractal noise SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_FractalNoise", octreeSDF);
}

void exampleInsideOutsideTest()
{
	// input: Vertex and index buffer (here I just put some nonsense in it)
	std::vector<Vertex> vertexBuffer;
	vertexBuffer.push_back(Vertex(Ogre::Vector3(0, 0, 0)));
	vertexBuffer.push_back(Vertex(Ogre::Vector3(1, 0, 0)));
	vertexBuffer.push_back(Vertex(Ogre::Vector3(0, 1, 0)));
	vertexBuffer.push_back(Vertex(Ogre::Vector3(0, 0, 1)));
	std::vector<unsigned int> indexBuffer;
	indexBuffer.push_back(0);
	indexBuffer.push_back(1);
	indexBuffer.push_back(2);
	indexBuffer.push_back(1);
	indexBuffer.push_back(2);
	indexBuffer.push_back(3);
	auto mesh = std::make_shared<TransformedMesh>(std::make_shared<Mesh>(vertexBuffer, indexBuffer));
	TriangleMeshSDF_Robust tester(mesh);
	float cellSize = 0.1f;
	tester.prepareSampling(tester.getAABB(), cellSize);
	// now you can query any point
	bool inside = tester.isInside(Ogre::Vector3(0, 0, 0));
}

int main()
{
	// buildSDFAndMarch("bunny_highres", 7);
	// buildSDFAndMarch("sphere", 7);
	testMeshImport();
	// testFractalNoisePlane();
	// splitBuddha2();
	//splitBuddha();
	while (true) {}
	return 0;
}