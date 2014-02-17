
#pragma once

#include <unordered_map>
#include <memory>
#include <vector>
#include "SignedDistanceField.h"
#include "Vector3i.h"
#include "AABB.h"
#include "Prerequisites.h"
#include "OpInvertSDF.h"
#include "Area.h"

// #define USE_BOOST_POOL

#ifdef USE_BOOST_POOL
#include "boost/pool/object_pool.hpp"
#endif

using std::vector;

/*
Samples a signed distance field in an adaptive way.
For each node (includes inner nodes and leaves) signed distances are stores for the 8 corners. This allows to interpolate signed distances in the node cell using trilinear interpolation.
The actual signed distances are stored in a spatial hashmap because octree nodes share corners with other nodes.
*/
class OctreeSDF : public SampledSignedDistanceField3D
{
protected:
	struct Node
	{
		Node* m_Children[8];
		Node()
		{
			for (int i = 0; i < 8; i++)
				m_Children[i] = nullptr;
		}
	};

	// Node allocation and deallocation methods
	__forceinline Node* allocNode()
	{
#ifdef USE_BOOST_POOL
		Node* node = m_NodePool.malloc();
#else
		Node* node = new Node();
#endif
		return node;
	}
	__forceinline void deallocNode(Node* node)
	{
#ifdef USE_BOOST_POOL
		m_NodePool.destroy(node);
#else
		delete node;
#endif
	}

	typedef std::unordered_map<Vector3i, Sample> SignedDistanceGrid;

	SignedDistanceGrid m_SDFValues;
	Node* m_RootNode;

#ifdef USE_BOOST_POOL
	boost::object_pool<Node> m_NodePool;
#endif

	float m_CellSize;

	/// The octree covers an axis aligned cube.
	Area m_RootArea;

	Node* cloneNode(Node* node, const Area& area, SignedDistanceGrid& sdfValues, SignedDistanceGrid& clonedSDFValues);

	static Sample lookupOrComputeSample(int corner, const Area& area, const SignedDistanceField3D* implicitSDF, SignedDistanceGrid& sdfValues);

	static Sample lookupSample(int corner, const Area& area, const SignedDistanceGrid& sdfValues);

	Sample lookupSample(int corner, const Area& area) const;

	Node* createNode(const Area& area, const SignedDistanceField3D* implicitSDF, SignedDistanceGrid& sdfValues);

	/// Top down octree constructor given a SDF.
	Node* createNode(const Area& area, const SignedDistanceField3D* implicitSDF, SignedDistanceGrid& sdfValues, int& nodeTypeMask);

	// Computes a lower and upper bound inside the area given the 8 corner signed distances.
	void getLowerAndUpperBound(Node* node, const Area& area, float* signedDistances, float& lowerBound, float& upperBound) const;

	void countNodes(Node* node, const Area& area, int& counter);

	Sample getSample(Node* node, const Area& area, const Ogre::Vector3& point) const;

	/// Intersects aligned octree nodes.
	Node* intersect(Node* node, Node* otherNode, const Area& area, SignedDistanceGrid& otherSDF, SignedDistanceGrid& newSDF);

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	Node* intersect(Node* node, const Area& area, SignedDistanceField3D* otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	Node* merge(Node* node, const Area& area, SignedDistanceField3D* otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	/// Interpolates signed distance for the 3x3x3 subgrid of a leaf.
	static void interpolateLeaf(const Area& area, SignedDistanceGrid& grid);
	void interpolateLeaf(const Area& area) { interpolateLeaf(area, m_SDFValues); }

	void getCubesToMarch(Node* node, const Area& area, vector<Cube>& cubes);
public:
	static std::shared_ptr<OctreeSDF> sampleSDF(std::shared_ptr<SignedDistanceField3D> otherSDF, int maxDepth);

	static std::shared_ptr<OctreeSDF> sampleSDF(std::shared_ptr<SignedDistanceField3D> otherSDF, AABB& aabb, int maxDepth);

	float getInverseCellSize() override;

	AABB getAABB() const override;

	vector<Cube> getCubesToMarch();

	Sample getSample(const Ogre::Vector3& point) const override;

	// TODO!
	bool intersectsSurface(const AABB &) const override;

	void subtract(std::shared_ptr<SignedDistanceField3D> otherSDF);

	void intersect(std::shared_ptr<SignedDistanceField3D> otherSDF);

	void intersectAlignedOctree(std::shared_ptr<OctreeSDF> otherOctree);

	/// Resizes the octree so that it covers the given aabb.
	void resize(const AABB& aabb);

	void merge(std::shared_ptr<SignedDistanceField3D> otherSDF);

	int countNodes();
};