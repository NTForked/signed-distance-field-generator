
#pragma once

#include <memory>
#include <fstream>
#include "OBJReader.h"
#include "SignedDistanceField.h"
#include "Mesh.h"
#include "OctreeSDF.h"
#include "TriangleSDF.h"
#include "MarchingCubes.h"
#include "ExportOBJ.h"

class SDFManager
{
public:
	static std::shared_ptr<Mesh> loadMesh(const std::string &filename)
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
				mesh->vertexBuffer.push_back(Vertex(vertexPositions[i], Ogre::Vector3(0, 0, 1)));
			// mesh->mergeVertices(0);
			mesh->removeDegeneratedTriangles();
			mesh->computeTriangleNormals();
			mesh->computeVertexNormals();
		}
		return mesh;
	}
	static std::shared_ptr<OctreeSDF> sampleSignedDistanceFieldFromMesh(const std::string& objFileName, int maxOctreeDepth = 7)
	{
		std::shared_ptr<Mesh> mesh = loadMesh(objFileName);
		TriangleMeshSDF_Robust meshSDF(std::make_shared<TransformedMesh>(mesh));
		return std::shared_ptr<OctreeSDF>(OctreeSDF::sampleSDF(meshSDF, maxOctreeDepth));
	}
	static void exportSignedDistanceFieldAsMesh(const std::string& objFileName, std::shared_ptr<OctreeSDF> sdf)
	{
		MarchingCubes::marchSDF<ExportOBJWithNormals>(objFileName, *sdf, sdf->getInverseCellSize(), 0);
	}
};