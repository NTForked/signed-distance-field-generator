
#pragma once

#include <vector>
#include <memory>
#include "OgreMath/OgreVector3.h"
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
	Mesh(const std::vector<Vertex>& vertexBuffer, const std::vector<unsigned int>& indexBuffer);

	void mergeVertices(float mergeRadius = std::numeric_limits<float>::epsilon());

	void smoothMesh(unsigned int numIterations);

	/// Removes triangles with a surface area of 0.
	void removeDegeneratedTriangles();

	///Computes the normals for each triangle and stores them in triangleNormals.
	void computeTriangleNormals();

	void computeVertexNormals();
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