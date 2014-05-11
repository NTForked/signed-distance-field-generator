
#pragma once

#include "Prerequisites.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <bitset>
#include "SolidGeometry.h"
#include "Vector3i.h"
#include "AABB.h"
#include "OpInvertSDF.h"
#include "Area.h"
#include "BVHScene.h"
#include <functional>

// #define USE_BOOST_POOL

#ifdef USE_BOOST_POOL
#include "boost/pool/object_pool.hpp"
#endif

using std::vector;

/*
Dual Contouring design
Proposal 1:
Grid nodes store no redundant data.
*/

/*
Samples a signed distance field in an adaptive way.
For each node (includes inner nodes and leaves) signed distances are stores for the 8 corners. This allows to interpolate signed distances in the node cell using trilinear interpolation.
The actual signed distances are stored in a spatial hashmap because octree nodes share corners with other nodes.
*/
class OctreeSF : public SampledSolidGeometry
{
protected:
    class GridNode;

    typedef GridNode GridNodeImpl;
public:
    static const int LEAF_EXPO = 3;
	static const int LEAF_SIZE_1D = (1 << LEAF_EXPO) + 1;
	static const int LEAF_SIZE_2D = LEAF_SIZE_1D * LEAF_SIZE_1D;
	static const int LEAF_SIZE_3D = LEAF_SIZE_2D * LEAF_SIZE_1D;
	static const int LEAF_SIZE_1D_INNER = LEAF_SIZE_1D - 1;
	static const int LEAF_SIZE_2D_INNER = LEAF_SIZE_1D_INNER * LEAF_SIZE_1D_INNER;
	static const int LEAF_SIZE_3D_INNER = LEAF_SIZE_2D_INNER * LEAF_SIZE_1D_INNER;

    struct SurfaceVertex
	{
		Vertex vertex;
		int vertexIndex;
        bool shared;
        SurfaceVertex() : shared(true) {}
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

        struct Edge
        {
            GridNode* n1;
            GridNode* n2;
            unsigned char direction;

            Edge() {}
            Edge(GridNode* n1, GridNode* n2, unsigned char direction)
                : n1(n1), n2(n2), direction(direction) {}

        };

        struct Face
        {
            GridNode* n1;
            GridNode* n2;
            unsigned normalDirection;
            Face() {}
            Face(GridNode* n1, GridNode* n2, unsigned char normalDirection)
                : n1(n1), n2(n2), normalDirection(normalDirection) {}
        };

        //! Executes the given function for all surface nodes.
        virtual void forEachSurfaceNode(const Area&, const std::function<void(GridNode*, const Area&)>&) {}

        //! Executes the given function for all surface nodes.
        virtual void forEachSurfaceNode(const std::function<void(GridNode*)>&) {}

        //! Executes the given function for all pairs neighboring surface nodes that share a face in the octree.
        virtual void forEachSurfaceFaceAndEdge(const std::function<void(const Face&)>&, const std::function<void(const Edge&)>&) {}

		virtual void countNodes(int& counter) const = 0;

        //! Counts the total memory consumption of the octree.
        virtual void countMemory(int&) const {}

        virtual void sumPositionsAndMass(const Area&, Ogre::Vector3&, float&) {}

		virtual void invert() = 0;

		virtual Node* clone() const = 0;

        virtual bool rayIntersectUpdate(const Area&, const Ray&, Ray::Intersection&) { return false; }

		inline Type getNodeType() { return m_NodeType; }
	};

	class InnerNode : public Node
	{
	public:
		Node* m_Children[8];
        InnerNode(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);
		~InnerNode();
		InnerNode(const InnerNode& rhs);

        virtual void forEachSurfaceNode(const Area& area, const std::function<void(GridNode*, const Area&)>& function) override;
        virtual void forEachSurfaceNode(const std::function<void(GridNode*)>& function) override;
        virtual void forEachSurfaceFaceAndEdge(const std::function<void(const Face&)>&, const std::function<void(const Edge&)>&) override;

		virtual void countNodes(int& counter) const override;

		virtual void countMemory(int& memoryCounter) const override;

		virtual void invert();

		virtual Node* clone() const override { return new InnerNode(*this); }

        virtual bool rayIntersectUpdate(const Area& area, const Ray& ray, Ray::Intersection& intersection) override;

        static void forEachSurfaceFace(Node* n1, Node* n2, unsigned char normalDirection, const std::function<void(const Face&)>&, const std::function<void(const Edge&)>&);
        static void forEachSurfaceEdge(Node* n1, Node* n2, unsigned char direction, const std::function<void(const Edge&)>&);

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;
	};

	class EmptyNode : public Node
	{
	public:
		// EmptyNode() {}
        EmptyNode(const Area& area, const SolidGeometry& implicitSDF);
		~EmptyNode();

		bool m_Sign;

		virtual void countNodes(int& counter) const override { counter++; }

		virtual void countMemory(int& memoryCounter) const override { memoryCounter += sizeof(*this); }

		virtual Node* clone() const override { return new EmptyNode(*this); }

		virtual void invert();

		// virtual void sumPositionsAndMass(const Area& area, Ogre::Vector3& weightedPosSum, float& totalMass) override;
	};

    static inline int indexOf(int x, int y, int z) { return x*LEAF_SIZE_2D + y * LEAF_SIZE_1D + z; }

    static inline int indexOf(const Vector3i &v) { return indexOf(v.x, v.y, v.z); }

    static inline Vector3i fromIndex(int index) { return Vector3i(index / LEAF_SIZE_2D, (index % LEAF_SIZE_2D) / LEAF_SIZE_1D, index % LEAF_SIZE_1D); }

    struct SurfaceEdge
    {
        SurfaceEdge() {}
        inline void init(const Vector3i& localMinPos, unsigned char direction, Ogre::Vector3 globalPos, float edgeLength, const SolidGeometry& sdf)
        {
            this->direction = direction;
            edgeIndex1 = indexOf(localMinPos);
            static const int EDGE_OFFSETS[] = { LEAF_SIZE_2D, LEAF_SIZE_1D, 1 };
            edgeIndex2 = edgeIndex1 + EDGE_OFFSETS[direction];

            Sample s;
            globalPos[direction] += edgeLength * 0.5f;
            sdf.getSample(globalPos, s);
            // Ogre::Vector3 rayDir(0,0,0);
            // rayDir[direction] = 1.0f;
            // sdf.raycastClosest(Ray(globalPos, rayDir), s);
            vertex.position = s.closestSurfacePos;
            vertex.normal = s.normal;
        }

        SurfaceEdge clone() { SurfaceEdge copy(*this); return copy; }

        Vertex vertex;
        unsigned short edgeIndex1;
        unsigned short edgeIndex2;
        unsigned char direction;

        unsigned short getNeighborCube(unsigned char neighborIndex) const
        {
            static const int EDGE_OFFSETS[] = { LEAF_SIZE_2D, LEAF_SIZE_1D, 1 };
            unsigned short cubeIndex = edgeIndex1;
            cubeIndex -= (neighborIndex & 1) * EDGE_OFFSETS[(direction + 1) % 3];
            cubeIndex -= ((neighborIndex & 2) >> 1) * EDGE_OFFSETS[(direction + 2) % 3];
            return cubeIndex;
        }
    };

    struct SurfaceCube
    {
        SurfaceCube() {}
        SurfaceCube(unsigned short cubeIndex) : cubeIndex(cubeIndex) {}// memset(edgeSamples, 0, sizeof(Vertex*) * 3); }
        // SurfaceVertex surfaceVertex;
        unsigned short cubeIndex;
        unsigned int vertexIndex;
    };

    class GridNode : public Node
    {
    public:
        GridNode() { m_NodeType = GRID; }
        GridNode(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);
        ~GridNode();

        Area m_Area;        // remove me!

        bool m_Signs[LEAF_SIZE_3D];

        std::vector<SurfaceEdge> m_SurfaceEdges;

        std::vector<SurfaceCube> m_SurfaceCubes;

        // stores (offset, neighbor)
        std::vector<std::pair<Vector3i, GridNode*> > m_CachedNeighbors;

        virtual void forEachSurfaceNode(const Area& area, const std::function<void(GridNode*, const Area&)>& function) override;
        virtual void forEachSurfaceNode(const std::function<void(GridNode*)>& function) override;

        void computeSigns(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);
        void computeEdges(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);
        void computeEdges(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF, bool ignoreEdges[3][LEAF_SIZE_3D]);

        void cacheNeighbor(const Vector3i& offset, GridNode* other);

        virtual void countNodes(int& counter) const override { counter++; }

        virtual void countMemory(int& memoryCounter) const override;

        virtual void generateVertices(vector<Vertex>& vertices);
        virtual void generateIndices(const Area& area, vector<unsigned int>& indices, vector<Vertex>& vertices) const;

        virtual Node* clone() const override;

        virtual void invert();

        void merge(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);
        void intersect(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF);

        void intersect(GridNode* otherNode);
        void merge(GridNode* otherNode);

        static inline unsigned char getCubeBitMask(int index, const bool* signsArray);

        virtual bool rayIntersectUpdate(const Area& area, const Ray& ray, Ray::Intersection& intersection) override;

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

    Node* intersect(Node* node, const SolidGeometry& implicitSDF, const Area& area);

    Node* merge(Node* node, const SolidGeometry& implicitSDF, const Area& area);

    Node* createNode(const Area& area, const SolidGeometry& implicitSDF);

    inline Ogre::Vector3 getRealPos(const Vector3i& cellIndex) const;

	BVHScene m_TriangleCache;
public:
	~OctreeSF();
	OctreeSF() : m_RootNode(nullptr) {}
	OctreeSF(const OctreeSF& other);

    static std::shared_ptr<OctreeSF> sampleSDF(SolidGeometry* otherSDF, int maxDepth);

    static std::shared_ptr<OctreeSF> sampleSDF(SolidGeometry* otherSDF, const AABB& aabb, int maxDepth);

	float getInverseCellSize() override;

	AABB getAABB() const override;

	std::shared_ptr<Mesh> generateMesh() override;

	/// Builds the triangle cache using marching cubes required for fast intersectsSurface queries.
	void generateTriangleCache();

	virtual bool intersectsSurface(const AABB &) const override;

	/// Subtracts the given signed distance field from this octree.
    void subtract(SolidGeometry* otherSDF);

	/// Intersects the octree with a signed distance field. For intersections with other octrees, use intersectAlignedOctree if possible.
    void intersect(SolidGeometry* otherSDF);

	/// Intersects the octree with another aligned octree (underlying grids must match).
	void intersectAlignedOctree(OctreeSF* otherOctree);

	/// Subtracts another aligned octree from this octree.
	void subtractAlignedOctree(OctreeSF* otherOctree);

	/// Merges another aligned octree into this octree.
	void mergeAlignedOctree(OctreeSF* otherOctree);

	/// Resizes the octree so that it covers the given aabb.
	void resize(const AABB& aabb);

	/// Merges the octree with another signed distance field.
    void merge(SolidGeometry* otherSDF);

	/// Inverts the sdf represented by the octree.
	void invert();

	/// Clones the octree and returns the copy.
	std::shared_ptr<OctreeSF> clone();

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

    void generateVerticesAndIndices(vector<Vertex>& vertices, vector<unsigned int>& indices);

    bool rayIntersectClosest(const Ray& ray, Ray::Intersection& intersection);

	// NIY by this data structure...
    virtual void getSample(const Ogre::Vector3&, Sample&) const override {}
};
