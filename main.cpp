
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
#include "Sphere.h"
#include "FracturePattern.h"
#include "AABBSDF.h"
#include "OctreeSF.h"

using std::vector;
using Ogre::Vector3;

void testVectorPerformance()
{
	int size = 500000;
	auto ts = Profiler::timestamp();
	std::vector<Vertex> stdVec;
	stdVec.reserve(size);
	for (int i = 0; i < size; i++)
	{
		Vertex v;
		v.position = Ogre::Vector3(i, i, i);
		stdVec.push_back(v);
	}
	Profiler::printJobDuration("std::vector pushback benchmark", ts);

	std::vector<Vertex> stdVec2;
	ts = Profiler::timestamp();
	stdVec2.reserve(size);
	for (int i = 0; i < size; i++)
	{
		stdVec2.emplace_back();
		Vertex& vert = stdVec2.back();
		vert.position = Ogre::Vector3(i, i, i);
	}
	Profiler::printJobDuration("std::vector emplace benchmark", ts);

	ts = Profiler::timestamp();
	for (int i = 0; i < size; i++)
	{
		stdVec2[i] = Vertex();
	}
	Profiler::printJobDuration("std::vector [] benchmark", ts);
}

/*#include "boost/unordered_map.hpp"
void testVector3iHashGridPerformance()
{
	Vector3iHashGrid<Vertex> grid;
	int size = 30000;
	grid.rehash(size * 2);
	auto ts = Profiler::timestamp();
	for (int i = 0; i < size; i++)
	{
		// grid.lookupOrCreate(Vector3i(i, i, i));
		grid.emplace(Vector3i(i, i, i));
		// grid.insertUnsafe(std::make_pair(Vector3i(i, i, i), Vertex()));
	}
	Profiler::printJobDuration("Vector3iHashGrid emplace benchmark", ts);

	ts = Profiler::timestamp();
	for (int i = 0; i < size; i++)
	{
		grid[Vector3i(i, i, i)].position.x = 0;
	}
	Profiler::printJobDuration("Vector3iHashGrid [] benchmark", ts);

	boost::unordered_map<Vector3i, Vertex, std::hash<Vector3i> > boostMap;
	boostMap.rehash(size * 2);
	ts = Profiler::timestamp();
	for (int i = 0; i < size; i++)
	{
		boostMap.emplace(Vector3i(i,i,i), Vertex());
	}
	Profiler::printJobDuration("boost::unordered_map emplace benchmark", ts);

	ts = Profiler::timestamp();
	for (int i = 0; i < size; i++)
	{
		boostMap[Vector3i(i, i, i)].position.x = 0;
	}
	Profiler::printJobDuration("boost::unordered_map [] benchmark", ts);
}*/

template<class Sampler>
void buildSDFAndMarch(const std::string& fileName, int maxDepth)
{
	std::shared_ptr<Mesh> mesh = SDFManager::loadObjMesh(fileName);
	auto ts = Profiler::timestamp();
	TriangleMeshSDF_Robust meshSDF(std::make_shared<TransformedMesh>(mesh));
	auto octreeSDF = Sampler::sampleSDF(&meshSDF, maxDepth);
	Profiler::printJobDuration("SDF import " + fileName, ts);
	std::cout << fileName << " SDF has " << octreeSDF->countLeaves() << " leaves and occupies " << octreeSDF->countMemory() / 1000 << " kb." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_" + fileName, octreeSDF);
}

template<class Sampler>
void testMeshImport()
{
	// buildSDFAndMarch<Sampler>("bunny.capped.obj", 8);		// 5.441 seconds
	buildSDFAndMarch<Sampler>("buddha2.obj", 9);				// 17.33 seconds
}

void testSphere()
{
	SphereSDF sdf(Ogre::Vector3(0, 0, 0), 1.0f);
	auto ts = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(&sdf, 8);
	Profiler::printJobDuration("Sphere sampling", ts);
	std::cout << "Num leaves: " << octreeSDF->countLeaves() << std::endl;
	SDFManager::exportSampledSDFAsMesh("SphereSDF", octreeSDF);
}

void testCubeSplit()
{
	AABBSDF sdf(Ogre::Vector3(-0.5f, -0.5f, -0.5f), Ogre::Vector3(0.5f, 0.5f, 0.5f));
	auto timestamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(&sdf, 8);
	Profiler::printJobDuration("Cube creation", timestamp);
	timestamp = Profiler::timestamp();
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0)), Ogre::Vector3(0, 0, 0));
	auto planeOctree = OctreeSDF::sampleSDF(fractalNoiseSDF.get(), octreeSDF->getAABB(), 8);
	Profiler::printJobDuration("Plane creation", timestamp);
	timestamp = Profiler::timestamp();
	octreeSDF->intersectAlignedOctree(planeOctree.get());
	// octreeSDF->subtractAlignedOctree(planeOctree.get());
	Profiler::printJobDuration("Plane subtraction", timestamp);
	SDFManager::exportSampledSDFAsMesh("CubeSDF", octreeSDF);
}


void splitBuddha()
{
	/*Ogre::Quaternion rotation(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0));
	Ogre::Matrix4 transform(rotation);
	auto part1 = SDFManager::sampleOctreeSDF(std::make_shared<TransformSDF>(SDFManager::createSDFFromMesh("buddha2.obj"), rotation), 8);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BuddhaSplit1", part1);*/
	auto part1 = OctreeSF::sampleSDF(SDFManager::createSDFFromMesh("buddha2.obj").get(), 9);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0)));
	// std::cout << "Buddha part1 has " << part1->countNodes() << " nodes." << std::endl;
	part1->intersect(fractalNoiseSDF.get());
	part2->subtract(fractalNoiseSDF.get());
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
	part1->subtract(fractalNoiseSDF.get());
	std::cout << "Bunny part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->intersect(fractalNoiseSDF.get());
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BunnySplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_BunnySplit2", part2);
}

void mergeBuddhaBunny()
{
	auto bunny = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	auto buddha = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	bunny->mergeAlignedOctree(buddha.get());
	SDFManager::exportSampledSDFAsMesh("BunnyBuddhaMerged", bunny);
}

void testBVHResampling()
{
	auto bunny = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	bunny->generateTriangleCache();
	auto ts = Profiler::timestamp();
	Ogre::Matrix4 transform = (Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0)));
	transform.setTrans(Ogre::Vector3(1.2f, 5.1f, 3.4f));
	transform.setScale(Ogre::Vector3(10, 10, 10));
	auto bunnyRotated = SDFManager::sampleOctreeSDF(std::make_shared<TransformSDF>(bunny, transform), 8);
	Profiler::printJobDuration("Bunny resampling", ts);
	SDFManager::exportSampledSDFAsMesh("sdfOctree_BunnyResampled", bunnyRotated);
}

template<class Sampler>
void splitBuddha2()
{
	auto part1 = Sampler::sampleSDF(SDFManager::createSDFFromMesh("buddha2.obj").get(), 9);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countLeaves() << " leaves." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(1.5f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.1f), Ogre::Vector3(1, 0, 0)));
	auto fractalNoiseOctreeSDF = Sampler::sampleSDF(fractalNoiseSDF.get(), part1->getAABB(), 9);
	std::cout << "Fractal noise SDF has " << fractalNoiseOctreeSDF->countLeaves() << " leaves." << std::endl;
	auto ts = Profiler::timestamp();
	part1->intersectAlignedOctree(fractalNoiseOctreeSDF.get());
	Profiler::printJobDuration("Buddha plane intersection", ts);
	std::cout << "Split buddha part1 has " << part1->countLeaves() << " leaves." << std::endl;
	ts = Profiler::timestamp();
	part2->subtractAlignedOctree(fractalNoiseOctreeSDF.get());
	Profiler::printJobDuration("Buddha plane subtraction", ts);
	std::cout << "Split buddha part2 has " << part2->countLeaves() << " leaves." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit2", part2);
}

void testFractalNoisePlane()
{
	Ogre::Quaternion rotation(Ogre::Radian(Ogre::Math::PI*0.5f), Ogre::Vector3(1, 0, 0));
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.15f);
	auto ts = Profiler::timestamp();
	auto octreeSDF = OctreeSF::sampleSDF(std::static_pointer_cast<SignedDistanceField3D>(fractalNoiseSDF).get(), 6);
	Profiler::printJobDuration("Fractal noise sampling", ts);
	std::cout << "Fractal noise SDF has " << octreeSDF->countLeaves() << " leaves." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_FractalNoise", octreeSDF);
}

void testFractureBuddha()
{
	auto buddha = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	std::vector<std::shared_ptr<OctreeSDF> > pieces;
	FracturePattern::splitRecursiveRandom(2, buddha, pieces);
	SDFManager::exportSampledSDFAsMesh("BuddhaFractured0", buddha);
	int i = 1;
	for (auto iPiece = pieces.begin(); iPiece != pieces.end(); iPiece++)
	{
		std::stringstream ss;
		ss << "BuddhaFractured" << (i++);
		SDFManager::exportSampledSDFAsMesh(ss.str(), *iPiece);
	}
}
void testFractureBunny()
{
	auto bunny = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("bunny.capped.obj"), 8);
	std::vector<std::shared_ptr<OctreeSDF> > pieces;
	FracturePattern::splitRecursiveRandom(4, bunny, pieces);
	SDFManager::exportSampledSDFAsMesh("BunnyFractured0", bunny);
	int i = 1;
	for (auto iPiece = pieces.begin(); iPiece != pieces.end(); iPiece++)
	{
		std::stringstream ss;
		ss << "BunnyFractured" << (i++);
		SDFManager::exportSampledSDFAsMesh(ss.str(), *iPiece);
	}
}


void testSphericalFracturePattern()
{
	SphericalFracturePattern pattern(8, 2);
	auto buddha = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	Ogre::Matrix4 mat;
	mat.makeTransform(Ogre::Vector3(0, 0, 0), Ogre::Vector3(0.2f, 0.2f, 0.2f), Ogre::Quaternion::IDENTITY);
	auto pieces = pattern.fractureSDF(buddha.get(), mat);
	SDFManager::exportSampledSDFAsMesh("buddhaFractured0", buddha);
	int num = 1;
	for (auto i = pieces.begin(); i != pieces.end(); i++)
	{
		std::stringstream ss;
		ss << "buddhaFractured" << num++;
		SDFManager::exportSampledSDFAsMesh(ss.str(), *i);
	}
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
	bool inside = tester.getSign(Ogre::Vector3(0, 0, 0));
}

int main()
{
	// std::cout << sizeof(SignedDistanceField3D::Sample) << std::endl;
	// testVector3iHashGridPerformance();
	testMeshImport<OctreeSF>();
	// testCubeSplit();
	// testSphere();
	// testFractalNoisePlane();
	// splitBuddha2<OctreeSDF>();
	// splitBuddha();
	while (true) {}
	return 0;
}