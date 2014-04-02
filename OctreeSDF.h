
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
class OctreeSDF : public SampledSignedDistanceField3D
{
protected:
	class GridNode;
public:
	static const int LEAF_EXPO = 2;
	static const int LEAF_SIZE_1D = (1 << LEAF_EXPO) + 1;
	static const int LEAF_SIZE_2D = LEAF_SIZE_1D * LEAF_SIZE_1D;
	static const int LEAF_SIZE_3D = LEAF_SIZE_2D * LEAF_SIZE_1D;
	static const int LEAF_SIZE_1D_INNER = LEAF_SIZE_1D - 1;
	static const int LEAF_SIZE_2D_INNER = LEAF_SIZE_1D_INNER * LEAF_SIZE_1D_INNER;
	static const int LEAF_SIZE_3D_INNER = LEAF_SIZE_2D_INNER * LEAF_SIZE_1D_INNER;

	/*struct SharedLeafFace
	{
		SharedLeafFace(const Ogre::Vector3& pos, float stepSize, int dim1, int dim2, const SignedDistanceField3D& implicitSDF);
		SharedLeafFace() : useCount(0) {}
		Sample samples[LEAF_SIZE_2D];
		int useCount;

		Sample& at(int x, int y) { return samples[x * LEAF_SIZE_2D + y]; }
	};*/
	struct SharedSample
	{
		SharedSample(const Sample& s) : useCount(0), sample(s) {}
		// ~SharedSample() { std::cout << "MUUH" << std::endl; }
		Sample sample;
		int useCount;
	};
	struct SharedSamples
	{
		SharedSamples() : sample(nullptr), gridNode(nullptr) {}
		SharedSamples(SharedSample* sample) : sample(sample), gridNode(nullptr) {}
		SharedSample* sample;
		GridNode* gridNode;		// grid node with sample position as min pos
		/*SharedLeafFace* faceXY;
		SharedLeafFace* faceXZ;
		SharedLeafFace* faceYZ;*/
	};
	typedef Vector3iHashGrid<SharedSamples*> SignedDistanceGrid;
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

		virtual void countNodes(int& counter) const = 0;

		virtual void countLeaves(int& counter) const {}

		virtual void countMemory(int& memoryCounter) const {}

		virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) {}

		virtual Sample getSample(const Area& area, const Ogre::Vector3& point) const { return Sample(); }

		virtual void getCubesToMarch(const Area& area, SignedDistanceGrid& sdfValues, vector<Cube>& cubes) = 0;

		virtual void invert() = 0;

		virtual Node* clone() const = 0;

		inline Type getNodeType() { return m_NodeType; }
	};

	class InnerNode : public Node
	{
	public:
		Node* m_Children[8];
		InnerNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);
		~InnerNode();
		InnerNode(const InnerNode& rhs);

		virtual void countNodes(int& counter) const override;

		virtual void countLeaves(int& counter) const override;

		virtual void countMemory(int& memoryCounter) const override;

		void getCubesToMarch(const Area& area, SignedDistanceGrid& sdfValues, vector<Cube>& cubes) override;

		virtual void invert();

		virtual Node* clone() const override { return new InnerNode(*this); }

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;

		// virtual Sample getSample(const Area& area, const Ogre::Vector3& point) const override;
	};

	class EmptyNode : public Node
	{
	public:
		// EmptyNode() {}
		EmptyNode(Sample* cornerSamples);
		~EmptyNode();

		Sample m_CornerSamples[8];

		virtual void countNodes(int& counter) const override { counter++; }

		virtual void countMemory(int& memoryCounter) const override { memoryCounter += sizeof(*this); }

		void getCubesToMarch(const Area& area, SignedDistanceGrid& sdfValues, vector<Cube>& cubes) override;

		virtual Node* clone() const override { return new EmptyNode(*this); }

		virtual void invert();

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;

		// virtual Sample getSample(const Area& area, const Ogre::Vector3& point) const override;
	};

	class GridNode : public Node
	{
	public:
		GridNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);
		~GridNode();
		// SharedLeafFace* m_Faces[6];
		Sample m_Samples[LEAF_SIZE_3D];

		Sample& at(int x, int y, int z) { return m_Samples[x*LEAF_SIZE_2D + y * LEAF_SIZE_1D + z]; }

		virtual void countNodes(int& counter) const override { counter++; }

		virtual void countLeaves(int& counter) const override { counter++; }

		virtual void countMemory(int& memoryCounter) const override { memoryCounter += sizeof(*this); }

		void getCubesToMarch(const Area& area, SignedDistanceGrid& sdfValues, vector<Cube>& cubes) override;

		virtual Node* clone() const override { return new GridNode(*this); }

		virtual void invert();

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;

		// virtual Sample getSample(const Area& area, const Ogre::Vector3& point) const override;
	};

	SignedDistanceGrid m_SDFValues;
	Node* m_RootNode;

	float m_CellSize;

	/// The octree covers an axis aligned cube.
	Area m_RootArea;

	float m_GridLeafStepSize;

	Node* simplifyNode(Node* node, const Area& area, int& nodeTypeMask);

	/// Intersects aligned octree nodes.
	Node* intersect(Node* node, Node* otherNode, const Area& area);

	Node* subtract(Node* node, Node* otherNode, const Area& area);

	Node* merge(Node* node, Node* otherNode, const Area& area);

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	// Node* intersect(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	/// Merges an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	// Node* merge(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache);

	static SharedSamples* lookupOrComputeSample(int corner, const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	static SharedSamples* lookupOrComputeSample(const Vector3i& globalPos, const Ogre::Vector3& realPos, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	/*static SharedLeafFace* lookupOrComputeXYFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);
	static SharedLeafFace* lookupOrComputeXZFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);
	static SharedLeafFace* lookupOrComputeYZFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);*/

	static Node* createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues);

	/// Interpolates signed distance for the 3x3x3 subgrid of a leaf.
	static void interpolateLeaf(const Area& area, SignedDistanceGrid& grid);
	void interpolateLeaf(const Area& area) { interpolateLeaf(area, m_SDFValues); }
	static void insertIfMissing(const Vector3i& key, const Sample& sample, OctreeSDF::SignedDistanceGrid& grid);

	/// Retrieves a pessimistic guess whether a leaf area could contain the surface.
	// bool couldContainSurface(const Area& area, SignedDistanceGrid& grid);

	std::vector<std::pair<Vector3i, float> > getControlPoints(const Area& area, float* cornerSignedDistances);

	bool approximatesWell(const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues, const std::vector<std::pair<Vector3i, float> >& controlPoints, float tolerance);

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

	/// Intersects the octree with another aligned octree (underlying grids must match).
	void intersectAlignedOctree(OctreeSDF* otherOctree);

	/// Subtracts another aligned octree from this octree.
	void subtractAlignedOctree(OctreeSDF* otherOctree);

	/// Merges another aligned octree into this octree.
	void mergeAlignedOctree(OctreeSDF* otherOctree);

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

	int getHeight() { return m_RootArea.m_SizeExpo;  }
};