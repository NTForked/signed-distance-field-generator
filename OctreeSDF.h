
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
#include "BVHScene.h"

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
	__forceinline void deallocNode(Node* node);

	typedef Vector3iHashGrid<Sample> SignedDistanceGrid;

	SignedDistanceGrid m_SDFValues;
	Node* m_RootNode;

#ifdef USE_BOOST_POOL
	boost::object_pool<Node> m_NodePool;
#endif

	float m_CellSize;

	/// The octree covers an axis aligned cube.
	Area m_RootArea;

	Node* cloneNode(Node* node, const Area& area, const SignedDistanceGrid& sdfValues, SignedDistanceGrid& clonedSDFValues);

	void cloneSDF(Node* node, const Area& area, const SignedDistanceGrid& sdfValues, SignedDistanceGrid& clonedSDFValues);

	static Sample lookupOrComputeSample(int corner, const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	static Sample lookupOrComputeSample(const Vector3i& globalPos, const Ogre::Vector3& realPos, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	static Sample lookupSample(int corner, const Area& area, const SignedDistanceGrid& sdfValues);

	Sample lookupSample(int corner, const Area& area) const;

	Node* createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	/// Top down octree constructor given a SDF.
	Node* createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues, int& nodeTypeMask);

	// Computes a lower and upper bound inside the area given the 8 corner signed distances.
	void getLowerAndUpperBound(Node* node, const Area& area, float* signedDistances, float& lowerBound, float& upperBound) const;

	void countNodes(Node* node, const Area& area, int& counter);

	void sumPositionsAndMass(Node* node, const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass);

	void removeReferencedSDFEntries(const Node* node, const Area& area, std::unordered_set<Vector3i>* deletionCandidates) const;

	Node* simplifyNode(Node* node, const Area& area, int& nodeTypeMask);

	Sample getSample(Node* node, const Area& area, const Ogre::Vector3& point) const;

	/// Ensures that sdf values for the 4 cubes containing the given edge between corner1 and corner2 exist.
	void ensureSDFValuesExist(Node* node, const Area& area, const Vector3i& corner1, const Vector3i& corner2);

	/// Intersects aligned octree nodes.
	Node* intersect(Node* node, Node* otherNode, const Area& area, SignedDistanceGrid& otherSDF, SignedDistanceGrid& newSDF);

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	Node* intersect(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	Node* intersect2(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF);

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	Node* merge(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	/// Interpolates signed distance for the 3x3x3 subgrid of a leaf.
	static void interpolateLeaf(const Area& area, SignedDistanceGrid& grid);
	void interpolateLeaf(const Area& area) { interpolateLeaf(area, m_SDFValues); }

	/// Retrieves a pessimistic guess whether a leaf area could contain the surface.
	// bool couldContainSurface(const Area& area, SignedDistanceGrid& grid);

	std::vector<std::pair<Vector3i, float> > getControlPoints(const Area& area, float* cornerSignedDistances);

	bool approximatesWell(const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues, const std::vector<std::pair<Vector3i, float> >& controlPoints, float tolerance);

	void addCubesContainingEdge(const Vector3i& corner1, const Vector3i& corner2, Vector3iHashGrid<Cube>& cubes);
	void getCubesToMarch(Node* node, const Area& area, vector<Cube>& cubes);

	BVHScene m_TriangleCache;
public:
	~OctreeSDF();
	OctreeSDF() : m_RootNode(nullptr) {}
	OctreeSDF(const OctreeSDF& other);

	static std::shared_ptr<OctreeSDF> sampleSDF(SignedDistanceField3D* otherSDF, int maxDepth);

	static std::shared_ptr<OctreeSDF> sampleSDF(SignedDistanceField3D* otherSDF, const AABB& aabb, int maxDepth);

	float getInverseCellSize() override;

	AABB getAABB() const override;

	vector<Cube> getCubesToMarch() override;

	Sample getSample(const Ogre::Vector3& point) const override;

	/// Builds the triangle cache using marching cubes required for fast intersectsSurface queries.
	void generateTriangleCache();

	virtual bool intersectsSurface(const AABB &) const override;

	/// Subtracts the given signed distance field from this octree.
	void subtract(SignedDistanceField3D* otherSDF);

	/// Intersects the octree with a signed distance field. For intersections with other octrees, use intersectAlignedOctree if possible.
	void intersect(SignedDistanceField3D* otherSDF);

	void intersect2(SignedDistanceField3D* otherSDF);

	/// Intersects the octree with another aligned octree (underlying grids must match).
	void intersectAlignedOctree(OctreeSDF* otherOctree);

	/// Subtracts another aligned octree from this octree.
	void subtractAlignedOctree(OctreeSDF* otherOctree);

	/// Resizes the octree so that it covers the given aabb.
	void resize(const AABB& aabb);

	/// Merges the octree with another signed distance field.
	void merge(SignedDistanceField3D* otherSDF);

	/// Inverts the sdf represented by the octree.
	void invert();

	/// Clones the octree and returns the copy.
	std::shared_ptr<OctreeSDF> clone();

	/// Counts the number of nodes in the octree.
	int countNodes();

	/// Computes the center of mass, also returns the total mass which is computed along the way.
	Ogre::Vector3 getCenterOfMass(float& totalMass);

	/// Computes the center of mass.
	Ogre::Vector3 getCenterOfMass();

	/// Removes not referenced sdf values, call this when you have memory problems.
	void cleanupSDF();

	/// Removes nodes that are not required.
	void simplify();

	int getHeight() { return m_RootArea.m_SizeExpo;  }
};