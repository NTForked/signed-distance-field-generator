
#pragma once

#include <vector>
#include <memory>
#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreColourValue.h"
#include "OgreMath/OgreMatrix4.h"
#include "AABB.h"
#include "Triangle.h"
#include "VertexMerger.h"
#include "Vertex.h"

struct Mesh
{
	//vertices, referenced in index buffer
	std::vector<Vertex> vertexBuffer;

	//index buffer, three following vertex indices form a triangle
	std::vector<unsigned int> indexBuffer;

	//normal for each triangle for optimization purposes
	std::vector<Ogre::Vector3> triangleNormals;

	Mesh() {}
	Mesh(const std::vector<Vertex>& vertexBuffer, const std::vector<unsigned int>& indexBuffer)
	{
		this->vertexBuffer.insert(this->vertexBuffer.begin(), vertexBuffer.begin(), vertexBuffer.end());
		this->indexBuffer.insert(this->indexBuffer.begin(), indexBuffer.begin(), indexBuffer.end());
		removeDegeneratedTriangles();
		computeTriangleNormals();
	}

	void mergeVertices(float mergeRadius = std::numeric_limits<float>::epsilon())
	{
		std::vector<Ogre::Vector3> vertices;
		vertices.resize(vertexBuffer.size());
		for (unsigned int i = 0; i < vertexBuffer.size(); i++)
			vertices[i] = vertexBuffer[i].position;
		VertexMerger::mergeVertices(vertices, indexBuffer, mergeRadius);
		for (unsigned int i = 0; i < vertexBuffer.size(); i++)
			vertexBuffer[i].position = vertices[i];
	}

	/// Removes triangles with a surface area of 0.
	void removeDegeneratedTriangles()
	{
		std::vector<unsigned int> validTris;
		int numRemovedTriangles = 0;
		for (auto i = indexBuffer.begin(); i != indexBuffer.end(); i+=3)
		{
			Ogre::Vector3 v0 = vertexBuffer[*(i+1)].position-vertexBuffer[*i].position;
			Ogre::Vector3 v1 = vertexBuffer[*(i+2)].position-vertexBuffer[*i].position;
			// std::cout << v0.crossProduct(v1).squaredLength() << std::endl;
			if (v0.crossProduct(v1).length() > 0)
			{
				validTris.push_back(*i);
				validTris.push_back(*(i+1));
				validTris.push_back(*(i+2));
			}
			else numRemovedTriangles++;
		}
		indexBuffer = validTris;
		std::cout << "Removed " << numRemovedTriangles << " degenerated triangles." << std::endl;
		if (!triangleNormals.empty())
			computeTriangleNormals();
	}

	///Computes the normals for each triangle and stores them in triangleNormals.
	void computeTriangleNormals()
	{
		triangleNormals.resize(indexBuffer.size() / 3);
		unsigned int triIndex = 0;
		for (auto i = indexBuffer.begin(); i != indexBuffer.end(); i+=3)
		{
			Ogre::Vector3 v0 = vertexBuffer[*(i+1)].position-vertexBuffer[*i].position;
			Ogre::Vector3 v1 = vertexBuffer[*(i+2)].position-vertexBuffer[*i].position;
			triangleNormals[triIndex] = v0.crossProduct(v1);
			triangleNormals[triIndex].normalise();
			++triIndex;
		}
	}

	void computeVertexNormals()
	{
		assert(triangleNormals.size() == indexBuffer.size() / 3);

		for (auto i = vertexBuffer.begin(); i != vertexBuffer.end(); i++)
			i->normal = Ogre::Vector3(0,0,0);

		unsigned int triIndex = 0;
		for (auto i = indexBuffer.begin(); i != indexBuffer.end(); i+=3)
		{
			vertexBuffer[*i].normal += triangleNormals[triIndex];
			vertexBuffer[*(i+1)].normal += triangleNormals[triIndex];
			vertexBuffer[*(i+2)].normal += triangleNormals[triIndex];
			triIndex++;
		}
		for (auto i = vertexBuffer.begin(); i != vertexBuffer.end(); i++)
			if (i->normal.squaredLength() > 0) i->normal.normalise();
	}
};

class TriangleSurface;

class TransformedMesh
{
protected:
	std::shared_ptr<Mesh> mMesh;

	Ogre::Matrix3 mRotation;
	Ogre::Vector3 mPosition;
	Ogre::Vector3 mScale;

	/*
	TODO: Threaded mesh transformation.
	class TransformWorker
	{
		TransformedMesh *mMesh;
		unsigned int mTriStart, mTriEnd;
		TransformWorker(TransformedMesh *mesh, unsigned int triStart, unsigned int triEnd) : mMesh(mesh), mTriStart(triStart), mTriEnd(triEnd) {}
		void operator()();
	};*/

public:

	__forceinline std::shared_ptr<Mesh> getMesh() { return mMesh; }

	// aabb for each triangle
	mutable std::vector<TriangleSurface> triangleSurfaces;

	// vertices in view space
	mutable std::vector<Vertex> vertexBufferVS;

	// triangle data in view space
	mutable std::vector<TriangleCached> triangleDataVS;

	TransformedMesh(std::shared_ptr<Mesh> mesh);

	void computeCache() const;

	void setPosition(const Ogre::Vector3 &position) { mPosition = position; }

	void setRotation(const Ogre::Quaternion &rotation) { rotation.ToRotationMatrix(mRotation); }

	void setScale(const Ogre::Vector3 &scale) { mScale = scale; }
};