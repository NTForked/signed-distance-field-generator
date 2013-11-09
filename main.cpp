
#include <vector>
#include <fstream>
#include <ctime>
#include <limits>
#include "PPMWriter.h"
#include "Profiler.h"
#include "OBJReader.h"
#include "VoxelGrid3DArray.h"
#include "MarchingCubes.h"
#include "ExportOBJ.h"
#include "Mesh.h"
#include "BVH.h"
#include "Surfaces.h"
#include "AABB.h"
#include "TriangleSDF.h"
#include "OctreeSDF.h"
#include "OpUnionSDF.h"

using std::vector;
using Ogre::Vector3;

std::shared_ptr<Mesh> loadMesh(const std::string &filename) 
{
	std::cout << "Loading mesh " + filename + "..." << std::endl;
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	std::ifstream file(filename, std::ifstream::in);
	std::vector<Ogre::Vector3> vertexPositions, vertexNormals;
	OBJReader::readObjFile(file, vertexPositions, vertexNormals, mesh->indexBuffer);
	std::cout << filename << " has " << vertexPositions.size() << " vertices and " << mesh->indexBuffer.size() / 3 << " triangles." << std::endl;
	if (vertexPositions.size() == vertexNormals.size())
	{
		for (unsigned int i = 0; i < vertexPositions.size(); i++)
			mesh->vertexBuffer.push_back(Vertex(vertexPositions[i], vertexNormals[i]));
		mesh->computeTriangleNormals();
		mesh->computeVertexNormals();
	}
	else
	{
		for (unsigned int i = 0; i < vertexPositions.size(); i++)
			mesh->vertexBuffer.push_back(Vertex(vertexPositions[i], Ogre::Vector3(0,0,1)));
		// mesh->mergeVertices(0);
		mesh->removeDegeneratedTriangles();
		mesh->computeTriangleNormals();
		mesh->computeVertexNormals();
	}
	return mesh;
}

template<class ImplicitSDF>
void buildSDFAndMarch(const std::string& fileName, int maxDepth)
{
	std::shared_ptr<Mesh> mesh = loadMesh(fileName + ".obj");
	ImplicitSDF meshSDF(std::make_shared<TransformedMesh>(mesh));

	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(meshSDF, maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	MarchingCubes<MaterialID>::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + fileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0, false);

	/*float cellSize = 1.0f / octreeSDF->getInverseCellSize();
	timeStamp = Profiler::timestamp();
	auto gridSDF = SignedDistanceField3DArray::sampleSDF(meshSDF, cellSize);
	Profiler::printJobDuration("SDF Grid construction", timeStamp);
	MarchingCubes<MaterialID>::marchSDF<ExportOBJWithNormals>("signedDistanceTestGrid_" + fileName, *gridSDF, gridSDF->getInverseCellSize(), 0, false);*/
}

void testOpUnion(const std::string& outFileName, const SignedDistanceField3D& sdf1, const SignedDistanceField3D& sdf2, int maxDepth)
{
	std::vector<const SignedDistanceField3D*> sdfs;
	sdfs.push_back(&sdf1);
	sdfs.push_back(&sdf2);
	OpUnionSDF unionSDF(sdfs);
	Profiler::Timestamp timeStamp = Profiler::timestamp();
	auto octreeSDF = OctreeSDF::sampleSDF(unionSDF, maxDepth);
	Profiler::printJobDuration("SDF Octree construction", timeStamp);
	MarchingCubes<MaterialID>::marchSDF<ExportOBJWithNormals>("signedDistanceTestOctree_" + outFileName, *octreeSDF, octreeSDF->getInverseCellSize(), 0, false);
}

int main()
{
	// buildSDFAndMarch<TriangleMeshSDF_RC>("buddha2", 8);
	// buildSDFAndMarch<TriangleMeshSDF_RC>("main_gear_01", 0.02f);
	// buildSDFAndMarch<TriangleMeshSDF_AWP>("Armadillo", 8);
	/*testOpUnion("Buddha_Bunny_Union",
		TriangleMeshSDF_RC(std::make_shared<TransformedMesh>(loadMesh("buddha2.obj"))),
		TriangleMeshSDF_RC(std::make_shared<TransformedMesh>(loadMesh("bunny_highres.obj"))),
		8);*/
	buildSDFAndMarch<TriangleMeshSDF_AWP>("bunny_highres", 8);
	// buildSDFAndMarch<TriangleMeshSDF_RC>("buddha2", 0.01f);
	// buildSDFAndMarch<TriangleMeshSDF_RC>("sponza2", 0.3f);
	while (true) {}
	return 0;
}