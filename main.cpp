
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

using std::vector;
using Ogre::Vector3;

void buildSDFAndMarch(const std::string& fileName, int maxDepth)
{
	std::shared_ptr<Mesh> mesh = SDFManager::loadObjMesh(fileName);
	auto ts = Profiler::timestamp();
	TriangleMeshSDF_Robust meshSDF(std::make_shared<TransformedMesh>(mesh));
	auto octreeSDF = OctreeSDF::sampleSDF(&meshSDF, maxDepth);
	Profiler::printJobDuration("SDF import " + fileName, ts);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_" + fileName, octreeSDF);
}

void testMeshImport()
{
	buildSDFAndMarch("bunny.capped.obj", 8);		// 5.441 seconds
	buildSDFAndMarch("buddha2.obj", 8);				// 17.33 seconds
}

void testSphere()
{
	SphereSDF sdf(Ogre::Vector3(0, 0, 0), 1.0f);
	auto octreeSDF = OctreeSDF::sampleSDF(&sdf, 8);
	SDFManager::exportSampledSDFAsMesh("SphereSDF", octreeSDF);
}

void testCube()
{
	AABBSDF sdf(Ogre::Vector3(-0.5f, -0.5f, -0.5f), Ogre::Vector3(0.5f, 0.5f, 0.5f));
	auto octreeSDF = OctreeSDF::sampleSDF(&sdf, 8);
	SDFManager::exportSampledSDFAsMesh("CubeSDF", octreeSDF);
}


void splitBuddha()
{
	auto part1 = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 8);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0)));
	part1->subtract(fractalNoiseSDF.get());
	std::cout << "Buddha part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->intersect(fractalNoiseSDF.get());
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

void splitBuddha2()
{
	auto part1 = SDFManager::sampleOctreeSDF(SDFManager::createSDFFromMesh("buddha2.obj"), 9);
	auto part2 = part1->clone();
	std::cout << "Buddha SDF has " << part1->countNodes() << " nodes." << std::endl;
	// SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctree_Buddha", octreeSDF);
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.15f, Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI*0.1f), Ogre::Vector3(1, 0, 0)));
	auto fractalNoiseOctreeSDF = OctreeSDF::sampleSDF(fractalNoiseSDF.get(), part1->getAABB(), 9);
	std::cout << "Fractal noise SDF has " << fractalNoiseOctreeSDF->countNodes() << " nodes." << std::endl;
	auto ts = Profiler::timestamp();
	part1->intersectAlignedOctree(fractalNoiseOctreeSDF.get());
	Profiler::printJobDuration("Buddha plane intersection", ts);
	std::cout << "Split buddha part1 has " << part1->countNodes() << " nodes." << std::endl;
	part2->subtractAlignedOctree(fractalNoiseOctreeSDF.get());
	std::cout << "Split buddha part2 has " << part2->countNodes() << " nodes." << std::endl;
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit1", part1);
	SDFManager::exportSampledSDFAsMesh("signedDistanceTestOctreeAligned_BuddhaSplit2", part2);
}

void testFractalNoisePlane()
{
	Ogre::Quaternion rotation(Ogre::Radian(Ogre::Math::PI*0.25f), Ogre::Vector3(1, 0, 0));
	auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.15f);	// , rotation);
	auto octreeSDF = SDFManager::sampleOctreeSDF(std::static_pointer_cast<SignedDistanceField3D>(fractalNoiseSDF), 8);
	std::cout << "Fractal noise SDF has " << octreeSDF->countNodes() << " nodes." << std::endl;
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
	bool inside = tester.isInside(Ogre::Vector3(0, 0, 0));
}

int main()
{
	testCube();
	// testFractalNoisePlane();
	// splitBuddha2();
	//splitBuddha();
	while (true) {}
	return 0;
}