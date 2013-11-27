
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

using std::vector;
using Ogre::Vector3;

void buildSDFAndMarch(const std::string& fileName, int maxDepth)
{
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto sdf = SDFManager::sampleSignedDistanceFieldFromMesh(fileName + ".obj", maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	SDFManager::exportSignedDistanceFieldAsMesh(fileName + "_sdf", sdf);
	/*float cellSize = 1.0f / octreeSDF->getInverseCellSize();
	timeStamp = Profiler::timestamp();
	auto gridSDF = SignedDistanceField3DArray::sampleSDF(meshSDF, cellSize);
	Profiler::printJobDuration("SDF Grid construction", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestGrid_" + fileName, *gridSDF, gridSDF->getInverseCellSize(), 0, false);*/
}

void testOpUnion(const std::string& outFileName, SignedDistanceField3D& sdf1, SignedDistanceField3D& sdf2, int maxDepth)
{
	std::vector<SignedDistanceField3D*> sdfs;
	sdfs.push_back(&sdf1);
	sdfs.push_back(&sdf2);
	OpUnionSDF unionSDF(sdfs);
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(unionSDF, maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0);
}

void testOpDifference(const std::string& outFileName, SignedDistanceField3D& sdf1, SignedDistanceField3D& sdf2, int maxDepth)
{
	OpInvertSDF invertedSDF2(&sdf2);
	std::vector<SignedDistanceField3D*> sdfs;
	sdfs.push_back(&sdf1);
	sdfs.push_back(&invertedSDF2);
	OpIntersectionSDF differenceSDF(sdfs);
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(differenceSDF, maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0);
}

void testOpUnion2(const std::string& outFileName, std::shared_ptr<OctreeSDF> octreeSDF, SignedDistanceField3D& sdf2)
{
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	octreeSDF->merge(sdf2);
	Profiler::printJobDuration("SDF Octree union", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0);
}

void testOpDifference2(const std::string& outFileName, std::shared_ptr<OctreeSDF> octreeSDF, SignedDistanceField3D& sdf2)
{
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	octreeSDF->subtract(sdf2);
	Profiler::printJobDuration("SDF Octree subtract", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0);
}

void testOpIntersection(const std::string& outFileName, SignedDistanceField3D& sdf1, SignedDistanceField3D& sdf2, int maxDepth)
{
	std::vector<SignedDistanceField3D*> sdfs;
	sdfs.push_back(&sdf1);
	sdfs.push_back(&sdf2);
	OpIntersectionSDF unionSDF(sdfs);
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(unionSDF, maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	MarchingCubes::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0);
}

int main()
{
	// buildSDFAndMarch("bunny_highres", 7);
	// buildSDFAndMarch("sphere", 7);
	TriangleMeshSDF_Robust cubeSDF(std::make_shared<TransformedMesh>(SDFManager::loadMesh("buddha2.obj")));
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto buddhaSampled = OctreeSDF::sampleSDF(cubeSDF, 8);
	Profiler::printJobDuration("Cube SDF construction", timeStamp);
	testOpDifference2("Bunny_Buddha_Difference",
		buddhaSampled,
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("bunny_highres.obj"))));
	/*testOpDifference2("Cube_Sphere_Difference",
		buddhaSampled,
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("sphere.obj"))));*/
	/*testOpDifference("Bunny_Buddha_Difference2",
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("buddha2.obj"))),
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("bunny_highres.obj"))), 8);*/
	/*testOpIntersection("Buddha_Bunny_Intersection",
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("buddha2.obj"))),
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("bunny_highres.obj"))),
		8);
	testOpUnion("Buddha_Bunny_Union",
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("buddha2.obj"))),
		TriangleMeshSDF_Robust(std::make_shared<TransformedMesh>(SDFManager::loadMesh("bunny_highres.obj"))),
		8);*/
	// buildSDFAndMarch("bunny_highres", 7);
	// buildSDFAndMarch("buddha2", 8);
	// buildSDFAndMarch<TriangleMeshSDF_RC>("sponza2", 0.3f);
	while (true) {}
	return 0;
}