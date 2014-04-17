
#pragma once

#include "Prerequisites.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <bitset>
#include "SignedDistanceField.h"
#include "Vector3i.h"
#include "AABB.h"
#include "OpInvertSDF.h"
#include "Area.h"
#include "BVHScene.h"
// #include "Vector3iHashGridRefCounted.h"

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
class OctreeSF2 : public SampledSignedDistanceField3D
{
protected:
	class GridNode;
public:
	static const int LEAF_EXPO = 0;

	struct SharedSurfaceVertex
	{
		Vertex vertex;
		int vertexIndex;
		int refCount;
		bool marked;
		SharedSurfaceVertex() : marked(false), refCount(0) {}
	};

protected:
	class Node
	{
	public:
		enum Type
		{
			INNER = 0,
			EMPTY = 1,
			GRID = 2
		};
	protected:
		Type m_NodeType;
	public:
		virtual ~Node() {}

        virtual void countNodes(int&) const = 0;

        virtual void countLeaves(int&) const {}

        virtual void countMemory(int&) const {}

        virtual void sumPositionsAndMass(const Area&, Ogre::Vector3&, float&) {}

        virtual void generateVertices(const Area&, vector<Vertex>&) {}
        virtual void generateIndices(const Area&, vector<unsigned int>&) const {}

        virtual void markSharedVertices(bool) {}

		virtual void invert() = 0;

		virtual Node* clone() const = 0;

		inline Type getNodeType() { return m_NodeType; }
	};

	class InnerNode : public Node
	{
	public:
		Node* m_Children[8];
		InnerNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices);
		~InnerNode();
		InnerNode(const InnerNode& rhs);

		virtual void countNodes(int& counter) const override;

		virtual void countLeaves(int& counter) const override;

		virtual void countMemory(int& memoryCounter) const override;

		virtual void markSharedVertices(bool marked) override;

		virtual void generateVertices(const Area& area, vector<Vertex>& vertices) override;
		virtual void generateIndices(const Area& area, vector<unsigned int>& indices) const override;

		virtual void invert();

		virtual Node* clone() const override { return new InnerNode(*this); }

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;
	};

	class EmptyNode : public Node
	{
	public:
		// EmptyNode() {}
		EmptyNode(const Area& area, const SignedDistanceField3D& implicitSDF);
		~EmptyNode();

		bool m_Sign;

		virtual void countNodes(int& counter) const override { counter++; }

		virtual void countMemory(int& memoryCounter) const override { memoryCounter += sizeof(*this); }

		virtual Node* clone() const override { return new EmptyNode(*this); }

		virtual void invert();

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;
	};

	class GridNode : public Node
	{
	public:
		GridNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices);
		~GridNode();

		unsigned char m_Signs;
		SharedSurfaceVertex* m_Edges[12];

		void unrefSharedVertex(SharedSurfaceVertex* sharedVertex)
		{
			sharedVertex->refCount--;
			if (!sharedVertex->refCount) delete sharedVertex;
		}
		virtual void countNodes(int& counter) const override { counter++; }

		virtual void countLeaves(int& counter) const override { counter++; }

		virtual void countMemory(int& memoryCounter) const override;

		virtual void markSharedVertices(bool marked) override;

		virtual void generateVertices(const Area& area, vector<Vertex>& vertices) override;
		virtual void generateIndices(const Area& area, vector<unsigned int>& indices) const override;

		void addUniqueEdgesAndVerticesWithSignChange(GridNode* otherGridNode);

		virtual Node* clone() const override;

		virtual void invert();

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;
	};

	Node* m_RootNode;

	float m_CellSize;

	/// The octree covers an axis aligned cube.
	Area m_RootArea;

	float m_GridLeafStepSize;

	Node* simplifyNode(Node* node, const Area& area, int& nodeTypeMask);

	/// Intersects aligned octree nodes.
	Node* intersectAlignedNode(Node* node, Node* otherNode, const Area& area);

	Node* subtractAlignedNode(Node* node, Node* otherNode, const Area& area);

	Node* mergeAlignedNode(Node* node, Node* otherNode, const Area& area);

	Node* intersect(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area);

	Node* subtract(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area);

	static Node* createNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices);

	BVHScene m_TriangleCache;
public:
	~OctreeSF2();
	OctreeSF2() : m_RootNode(nullptr) {}
	OctreeSF2(const OctreeSF2& other);

	static std::shared_ptr<OctreeSF2> sampleSDF(SignedDistanceField3D* otherSDF, int maxDepth);

	static std::shared_ptr<OctreeSF2> sampleSDF(SignedDistanceField3D* otherSDF, const AABB& aabb, int maxDepth);

	float getInverseCellSize() override;

	AABB getAABB() const override;

	std::shared_ptr<Mesh> generateMesh() override;

	/// Builds the triangle cache using marching cubes required for fast intersectsSurface queries.
	void generateTriangleCache();

	virtual bool intersectsSurface(const AABB &) const override;

	/// Subtracts the given signed distance field from this octree.
	void subtract(SignedDistanceField3D* otherSDF);

	/// Intersects the octree with a signed distance field. For intersections with other octrees, use intersectAlignedOctree if possible.
	void intersect(SignedDistanceField3D* otherSDF);

	/// Intersects the octree with another aligned octree (underlying grids must match).
	void intersectAlignedOctree(OctreeSF2* otherOctree);

	/// Subtracts another aligned octree from this octree.
	void subtractAlignedOctree(OctreeSF2* otherOctree);

	/// Merges another aligned octree into this octree.
	void mergeAlignedOctree(OctreeSF2* otherOctree);

	/// Resizes the octree so that it covers the given aabb.
	void resize(const AABB& aabb);

	/// Merges the octree with another signed distance field.
	void merge(SignedDistanceField3D* otherSDF);

	/// Inverts the sdf represented by the octree.
	void invert();

	/// Clones the octree and returns the copy.
	std::shared_ptr<OctreeSF2> clone();

	/// Counts the number of nodes in the octree.
	int countNodes();

	/// Counts the number of leaves in the octree.
	int countLeaves();

	/// Counts the number of bytes the octree occupies.
	int countMemory();

	/// Computes the center of mass, also returns the total mass which is computed along the way.
	Ogre::Vector3 getCenterOfMass(float& totalMass);

	/// Computes the center of mass.
	Ogre::Vector3 getCenterOfMass();

	/// Removes nodes that are not required.
	void simplify();

	int getHeight() { return m_RootArea.m_SizeExpo; }

	// NIY by this data structure...
    virtual void getSample(const Ogre::Vector3&, Sample&) const override {}
};
