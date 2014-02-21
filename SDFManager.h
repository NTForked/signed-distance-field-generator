
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
#include "FractalNoisePlaneSDF.h"
#include "TransformSDF.h"

class SDFManager
{
protected:
	static std::shared_ptr<Mesh> loadObjMesh(const std::string &filename)
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
public:
	/// Creates an octree that approximates the given signed distance field.
	static std::shared_ptr<OctreeSDF> sampleOctreeSDF(std::shared_ptr<SignedDistanceField3D> sdf, int maxOctreeDepth = 7)
	{
		return OctreeSDF::sampleSDF(sdf, maxOctreeDepth);
	}

	/// Creates a signed distance field given the filename of an .obj file.
	static std::shared_ptr<SignedDistanceField3D> createSDFFromMesh(const std::string& objFileName)
	{
		std::shared_ptr<Mesh> mesh = loadObjMesh(objFileName);
		return std::make_shared<TriangleMeshSDF_Robust>(std::make_shared<TransformedMesh>(mesh));
	}

	/***
	* Creates a signed distance field given a vertex and index buffer.
	* In the vertex buffer, it is not necessary to provide normals.
	*/
	static std::shared_ptr<SignedDistanceField3D> createSDFFromMesh(
		std::vector<Vertex>& vertexBuffer,
		std::vector<unsigned int>& indexBuffer)
	{
		std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>(vertexBuffer, indexBuffer);
		return std::make_shared<TriangleMeshSDF_Robust>(std::make_shared<TransformedMesh>(mesh));
	}

	/// Creates a fractal noise sdf.
	static std::shared_ptr<SignedDistanceField3D> createFractalNoiseSDF(float size,
		float roughness,
		float zRange,
		const Ogre::Quaternion& rotation = Ogre::Quaternion::IDENTITY,
		const Ogre::Vector3& position = Ogre::Vector3(0,0,0))
	{
		auto fractalNoiseSDF = std::make_shared<FractalNoisePlaneSDF>(size, roughness, zRange);
		Ogre::Matrix4 transform(rotation);
		transform.setTrans(position);
		return std::make_shared<TransformSDF>(fractalNoiseSDF, transform);
	}

	/// Exports a sampled signed distance field as a triangle mesh in .obj format.
	static void exportSampledSDFAsMesh(const std::string& objFileName, std::shared_ptr<SampledSignedDistanceField3D> sdf)
	{
		auto marched = MarchingCubes::marchSDF(*sdf, sdf->getInverseCellSize());
		std::cout << "[Marching cubes] Computing normals ..." << std::endl;
		marched->computeTriangleNormals();
		marched->computeVertexNormals();
		std::cout << "[Marching cubes] Writing file \"" << objFileName << "\" ..." << std::endl;
		ExportOBJ::writeMesh(objFileName, marched->vertexBuffer, marched->indexBuffer);
		std::cout << "[Marching cubes] Finished!" << std::endl;
	}
};