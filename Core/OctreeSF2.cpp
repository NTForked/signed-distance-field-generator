
#include <stack>
#include "OctreeSF2.h"
#include "SignedDistanceField.h"
#include "MarchingCubes.h"
#include "Mesh.h"
#include "TriangleLookupTable.h"
#include <bitset>
#include <memory>

/******************************************************************************************
Node constructors
*******************************************************************************************/

OctreeSF2::InnerNode::InnerNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	m_NodeType = INNER;

	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
	{
		m_Children[i] = OctreeSF2::createNode(subAreas[i], octreeMaxSize, implicitSDF, sharedVertices);
	}
}

OctreeSF2::InnerNode::~InnerNode()
{
	for (int i = 0; i < 8; i++)
	{
		// m_CornerSamples[i]->sample->useCount--;
		delete m_Children[i];
	}
}

OctreeSF2::InnerNode::InnerNode(const InnerNode& rhs)
{
	m_NodeType = INNER;
	for (int i = 0; i < 8; i++)
	{
		m_Children[i] = rhs.m_Children[i]->clone();
	}
}

OctreeSF2::EmptyNode::EmptyNode(const Area& area, const SignedDistanceField3D& implicitSDF)
{
	m_NodeType = EMPTY;
	m_Sign = implicitSDF.getSign(area.toAABB().getCenter());
}

OctreeSF2::EmptyNode::~EmptyNode()
{
}

OctreeSF2::Node* OctreeSF2::GridNode::clone() const
{
	GridNode* copy = new GridNode(*this);
	// perform deep copy
	for (int i = 0; i < 12; i++)
	{
		if (copy->m_Edges[i])
			copy->m_Edges[i]->refCount++;
	}
	return copy;
}

OctreeSF2::GridNode::GridNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	m_NodeType = GRID;

	m_Signs = 0;
	for (int i = 0; i < 8; i++)
	{
		m_Signs |= (((unsigned char)implicitSDF.getSign(area.getCornerVecs(i).second)) << i);
	}

	static const int offsets[] = { 4, 2, 1 };

	for (int i = 0; i < 12; i++)
	{
		m_Edges[i] = nullptr;
		const TLT::DirectedEdge& edge = TLT::getSingleton().directedEdges[i];
		int cornerMask1 = 1 << edge.minCornerIndex;
		unsigned char sign1 = (m_Signs & cornerMask1) >> edge.minCornerIndex;
		int corner2 = edge.minCornerIndex + offsets[edge.direction];
		int cornerMask2 = 1 << corner2;
		unsigned char sign2 = (m_Signs & cornerMask2) >> corner2;
		if (sign1 != sign2)
		{
			bool created;
			SharedSurfaceVertex*& sharedVertex = sharedVertices[edge.direction].lookupOrCreate(area.getCorner(edge.minCornerIndex), created);
			if (created)
			{
				sharedVertex = new SharedSurfaceVertex();
				sharedVertex->vertex.position = implicitSDF.getSample((area.getCornerVecs(edge.minCornerIndex).second + area.getCornerVecs(corner2).second) * 0.5f).closestSurfacePos;
			}
			m_Edges[i] = sharedVertex;
			m_Edges[i]->refCount++;
		}
	}
}

OctreeSF2::GridNode::~GridNode()
{
}

void OctreeSF2::GridNode::countMemory(int& memoryCounter) const
{
	memoryCounter += sizeof(*this);

	for (int i = 0; i < 12; i++)
	{
		if (m_Edges[i] && !m_Edges[i]->marked)
		{
			memoryCounter += sizeof(SharedSurfaceVertex);
			m_Edges[i]->marked = true;
		}
	}
}

OctreeSF2::Node* OctreeSF2::createNode(const Area& area, int octreeMaxSize, const SignedDistanceField3D& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (area.m_SizeExpo <= LEAF_EXPO && needsSubdivision)
		return new GridNode(area, octreeMaxSize, implicitSDF, sharedVertices);

	if (needsSubdivision)
		return new InnerNode(area, octreeMaxSize, implicitSDF, sharedVertices);

	return new EmptyNode(area, implicitSDF);
}

void OctreeSF2::InnerNode::countNodes(int& counter) const
{
	counter++;
	for (int i = 0; i < 8; i++)
		m_Children[i]->countNodes(counter);
}

void OctreeSF2::InnerNode::countLeaves(int& counter) const
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->countLeaves(counter);
}

void OctreeSF2::InnerNode::countMemory(int& counter) const
{
	counter += sizeof(*this);
	for (int i = 0; i < 8; i++)
		m_Children[i]->countMemory(counter);
}

void OctreeSF2::InnerNode::markSharedVertices(bool marked)
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->markSharedVertices(marked);
}

void OctreeSF2::GridNode::markSharedVertices(bool marked)
{
	for (int i = 0; i < 12; i++)
	{
		if (m_Edges[i])
		{
			m_Edges[i]->marked = marked;
		}
	}
}

void OctreeSF2::GridNode::generateVertices(const Area& area, vector<Vertex>& vertices)
{
	for (int i = 0; i < 12; i++)
	{
		if (m_Edges[i] && !m_Edges[i]->marked)
		{
			m_Edges[i]->vertexIndex = vertices.size();
			m_Edges[i]->marked = true;
			vertices.push_back(m_Edges[i]->vertex);
		}
	}
}

#include "TriangleLookupTable.h"
void OctreeSF2::GridNode::generateIndices(const Area& area, vector<unsigned int>& indices) const
{
	const std::vector<Triangle<int> >& tris = TLT::getSingleton().indexTable[m_Signs];
	for (auto i2 = tris.begin(); i2 != tris.end(); ++i2)
	{
		indices.push_back((int)(m_Edges[i2->p1]->vertexIndex));
		indices.push_back((int)(m_Edges[i2->p2]->vertexIndex));
		indices.push_back((int)(m_Edges[i2->p3]->vertexIndex));
	}
}

void OctreeSF2::InnerNode::generateVertices(const Area& area, vector<Vertex>& vertices)
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
		m_Children[i]->generateVertices(subAreas[i], vertices);
}
void OctreeSF2::InnerNode::generateIndices(const Area& area, vector<unsigned int>& indices) const
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
		m_Children[i]->generateIndices(subAreas[i], indices);
}

void OctreeSF2::InnerNode::invert()
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->invert();
}

void OctreeSF2::EmptyNode::invert()
{
	m_Sign = !m_Sign;
}

void OctreeSF2::GridNode::invert()
{
	m_Signs = ~m_Signs;
}

void OctreeSF2::GridNode::addUniqueEdgesAndVerticesWithSignChange(GridNode* otherGridNode)
{
}

OctreeSF2::Node* OctreeSF2::intersect(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area)
{
	/*bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (node->getNodeType() == Node::INNER && needsSubdivision)
	{
	InnerNode* innerNode = (InnerNode*)node;
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
	innerNode->m_Children[i] = intersect(innerNode->m_Children[i], implicitSDF, subAreas[i]);
	return node;
	}
	if (!needsSubdivision)
	{
	if (implicitSDF.getSign(area.getCornerVecs(0).second))
	return node;
	delete node;
	return createNode(area, implicitSDF);
	}
	if (node->getNodeType() == Node::EMPTY)
	{
	EmptyNode* emptyNode = (EmptyNode*)node;
	if (!emptyNode->m_Sign)
	return node;
	delete node;
	return createNode(area, implicitSDF);

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode otherGridNode(area, implicitSDF);
	auto edgeCopy = gridNode->m_SurfaceEdges;
	auto cubesCopy = gridNode->m_SurfaceCubes;
	gridNode->m_SurfaceEdges.clear();
	gridNode->m_SurfaceCubes.clear();
	gridNode->m_Signs &= otherGridNode.m_Signs;
	gridNode->addEdgesAndVerticesWithSignChange(edgeCopy, cubesCopy);
	gridNode->addEdgesAndVerticesWithSignChange(otherGridNode.m_SurfaceEdges, otherGridNode.m_SurfaceCubes);*/
	return node;
}

OctreeSF2::Node* OctreeSF2::subtract(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area)
{
	/*bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (node->getNodeType() == Node::INNER && needsSubdivision)
	{
	InnerNode* innerNode = (InnerNode*)node;
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
	innerNode->m_Children[i] = subtract(innerNode->m_Children[i], implicitSDF, subAreas[i]);
	return node;
	}
	if (!needsSubdivision)
	{
	if (!implicitSDF.getSign(area.getCornerVecs(0).second))
	return node;
	delete node;
	Node* newNode = createNode(area, implicitSDF);
	newNode->invert();
	return newNode;
	}
	if (node->getNodeType() == Node::EMPTY)
	{
	EmptyNode* emptyNode = (EmptyNode*)node;
	if (!emptyNode->m_Sign)
	return node;
	delete node;
	Node* newNode = createNode(area, implicitSDF);
	newNode->invert();
	return newNode;

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode otherGridNode(area, implicitSDF);
	otherGridNode.invert();
	auto edgeCopy = gridNode->m_SurfaceEdges;
	auto cubesCopy = gridNode->m_SurfaceCubes;
	gridNode->m_SurfaceEdges.clear();
	gridNode->m_SurfaceCubes.clear();
	gridNode->m_Signs &= otherGridNode.m_Signs;
	gridNode->addEdgesAndVerticesWithSignChange(edgeCopy, cubesCopy);
	gridNode->addEdgesAndVerticesWithSignChange(otherGridNode.m_SurfaceEdges, otherGridNode.m_SurfaceCubes);*/
	return node;
}

OctreeSF2::Node* OctreeSF2::intersectAlignedNode(Node* node, Node* otherNode, const Area& area)
{
	if (node->getNodeType() == Node::INNER && otherNode->getNodeType() == Node::INNER)
	{
		InnerNode* innerNode = (InnerNode*)node;
		InnerNode* otherInnerNode = (InnerNode*)otherNode;
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			innerNode->m_Children[i] = intersectAlignedNode(innerNode->m_Children[i], otherInnerNode->m_Children[i], subAreas[i]);
		return node;
	}
	if (otherNode->getNodeType() == Node::EMPTY)
	{
		EmptyNode* otherEmptyNode = (EmptyNode*)otherNode;
		if (otherEmptyNode->m_Sign)
			return node;
		delete node;
		return otherNode->clone();
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (!emptyNode->m_Sign)
			return node;
		delete node;
		return otherNode->clone();

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	gridNode->m_Signs &= otherGridNode->m_Signs;
	gridNode->addUniqueEdgesAndVerticesWithSignChange(otherGridNode);
	return node;
}

OctreeSF2::Node* OctreeSF2::subtractAlignedNode(Node* node, Node* otherNode, const Area& area)
{
	if (node->getNodeType() == Node::INNER && otherNode->getNodeType() == Node::INNER)
	{
		InnerNode* innerNode = (InnerNode*)node;
		InnerNode* otherInnerNode = (InnerNode*)otherNode;
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			innerNode->m_Children[i] = subtractAlignedNode(innerNode->m_Children[i], otherInnerNode->m_Children[i], subAreas[i]);
		return node;
	}
	if (otherNode->getNodeType() == Node::EMPTY)
	{
		EmptyNode* otherEmptyNode = (EmptyNode*)otherNode;
		if (!otherEmptyNode->m_Sign)
			return node;
		delete node;
		Node* inverted = otherNode->clone();
		inverted->invert();
		return inverted;
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (!emptyNode->m_Sign)
			return node;
		delete node;
		Node* inverted = otherNode->clone();
		inverted->invert();
		return inverted;

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	otherGridNode->invert();
	gridNode->m_Signs &= otherGridNode->m_Signs;
	gridNode->addUniqueEdgesAndVerticesWithSignChange(otherGridNode);
	return node;
}

OctreeSF2::Node* OctreeSF2::mergeAlignedNode(Node* node, Node* otherNode, const Area& area)
{
	if (node->getNodeType() == Node::INNER && otherNode->getNodeType() == Node::INNER)
	{
		InnerNode* innerNode = (InnerNode*)node;
		InnerNode* otherInnerNode = (InnerNode*)otherNode;
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			innerNode->m_Children[i] = mergeAlignedNode(innerNode->m_Children[i], otherInnerNode->m_Children[i], subAreas[i]);
		return node;
	}
	if (otherNode->getNodeType() == Node::EMPTY)
	{
		EmptyNode* otherEmptyNode = (EmptyNode*)otherNode;
		if (!otherEmptyNode->m_Sign)
			return node;
		delete node;
		return otherNode->clone();
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_Sign)
			return node;
		delete node;
		return otherNode->clone();

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	gridNode->m_Signs |= otherGridNode->m_Signs;
	gridNode->addUniqueEdgesAndVerticesWithSignChange(otherGridNode);
	return node;
}

std::shared_ptr<OctreeSF2> OctreeSF2::sampleSDF(SignedDistanceField3D* otherSDF, int maxDepth)
{
	AABB aabb = otherSDF->getAABB();
	return sampleSDF(otherSDF, aabb, maxDepth);
}

std::shared_ptr<OctreeSF2> OctreeSF2::sampleSDF(SignedDistanceField3D* otherSDF, const AABB& aabb, int maxDepth)
{
	std::shared_ptr<OctreeSF2> octreeSF = std::make_shared<OctreeSF2>();
	Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
	float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
	octreeSF->m_CellSize = cubeSize / (1 << maxDepth);
	otherSDF->prepareSampling(aabb, octreeSF->m_CellSize);
	octreeSF->m_RootArea = Area(Vector3i(0, 0, 0), maxDepth, aabb.getMin(), cubeSize);
	Vector3iHashGrid<SharedSurfaceVertex*> sharedVertices[3];
	octreeSF->m_RootNode = octreeSF->createNode(octreeSF->m_RootArea, octreeSF->m_RootArea.m_MaxPos.x, *otherSDF, sharedVertices);
	return octreeSF;
}

float OctreeSF2::getInverseCellSize()
{
	return (float)(1 << m_RootArea.m_SizeExpo) / m_RootArea.m_RealSize;
}

AABB OctreeSF2::getAABB() const
{
	return m_RootArea.toAABB();
}

#include "Profiler.h"
std::shared_ptr<Mesh> OctreeSF2::generateMesh()
{
	auto tsTotal = Profiler::timestamp();
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	int numLeaves = countLeaves();
	mesh->vertexBuffer.reserve(numLeaves * 2);	// reasonable upper bound
	mesh->indexBuffer.reserve(numLeaves * 8);
	// auto ts = Profiler::timestamp();
	m_RootNode->generateVertices(m_RootArea, mesh->vertexBuffer);
	// Profiler::printJobDuration("generateVertices", ts);
	// std::cout << "Num vertices: " << mesh->vertexBuffer.size() << ", reserved was " << numLeaves * LEAF_SIZE_2D_INNER * 2 << std::endl;
	// ts = Profiler::timestamp();
	m_RootNode->generateIndices(m_RootArea, mesh->indexBuffer);
	// std::cout << "Num indices: " << mesh->indexBuffer.size() << ", reserved was " << numLeaves * LEAF_SIZE_2D_INNER * 8 << std::endl;
	// Profiler::printJobDuration("generateIndices", ts);
	// ts = Profiler::timestamp();
	m_RootNode->markSharedVertices(false);
	// Profiler::printJobDuration("markSharedVertices", ts);
	Profiler::printJobDuration("generateMesh", tsTotal);
	return mesh;
}

bool OctreeSF2::intersectsSurface(const AABB& aabb) const
{
	if (m_TriangleCache.getBVH())
		return m_TriangleCache.getBVH()->intersectsAABB(aabb);
	return true;
}

void OctreeSF2::subtract(SignedDistanceField3D* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	auto ts = Profiler::timestamp();
	m_RootNode = subtract(m_RootNode, *otherSDF, m_RootArea);
	Profiler::printJobDuration("Subtraction", ts);
}

void OctreeSF2::intersect(SignedDistanceField3D* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	auto ts = Profiler::timestamp();
	m_RootNode = intersect(m_RootNode, *otherSDF, m_RootArea);
	Profiler::printJobDuration("Intersection", ts);
}

void OctreeSF2::intersectAlignedOctree(OctreeSF2* otherOctree)
{
	m_RootNode = intersectAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF2::subtractAlignedOctree(OctreeSF2* otherOctree)
{
	m_RootNode = subtractAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF2::mergeAlignedOctree(OctreeSF2* otherOctree)
{
	m_RootNode = mergeAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF2::resize(const AABB& aabb)
{
	/*	while (!m_RootArea.toAABB().containsPoint(aabb.min))
	{	// need to resize octree
	m_RootArea.m_MinPos = m_RootArea.m_MinPos - Vector3i(1 << m_RootArea.m_SizeExpo, 1 << m_RootArea.m_SizeExpo, 1 << m_RootArea.m_SizeExpo);
	m_RootArea.m_MinRealPos -= Ogre::Vector3(m_RootArea.m_RealSize, m_RootArea.m_RealSize, m_RootArea.m_RealSize);
	m_RootArea.m_RealSize *= 2.0f;
	m_RootArea.m_SizeExpo++;
	Node* oldRoot = m_RootNode;
	m_RootNode = allocNode();
	m_RootNode->m_Children[7] = oldRoot;

	// need to fill in some signed distance values for the new area
	for (int i = 0; i < 7; i++)
	{
	Vector3i offset = Vector3i((i & 4) != 0, (i & 2) != 0, (i & 1) != 0) * (1 << m_RootArea.m_SizeExpo);
	m_SDFValues[m_RootArea.m_MinPos + offset] = -m_RootArea.m_RealSize;
	}
	interpolateLeaf(m_RootArea);
	}
	while (!m_RootArea.toAABB().containsPoint(aabb.max))
	{	// need to resize octree
	m_RootArea.m_RealSize *= 2.0f;
	m_RootArea.m_SizeExpo++;
	Node* oldRoot = m_RootNode;
	m_RootNode = allocNode();
	m_RootNode->m_Children[0] = oldRoot;

	// need to fill in some signed distance values for the new area
	for (int i = 1; i < 8; i++)
	{
	Vector3i offset = Vector3i((i & 4) != 0, (i & 2) != 0, (i & 1) != 0) * (1 << m_RootArea.m_SizeExpo);
	m_SDFValues[m_RootArea.m_MinPos + offset] = -m_RootArea.m_RealSize;
	}
	interpolateLeaf(m_RootArea);
	}*/
}

void OctreeSF2::merge(SignedDistanceField3D* otherSDF)
{
	// this is not an optimal resize policy but it should work
	// it is recommended to avoid resizes anyway
	/*	resize(otherSDF->getAABB());

	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	SignedDistanceGrid newSDF;
	m_RootNode = merge(m_RootNode, m_RootArea, *otherSDF, newSDF, SignedDistanceGrid());
	for (auto i = newSDF.begin(); i != newSDF.end(); i++)
	{
	for (auto i2 = i->begin(); i2 != i->end(); i2++)
	{
	m_SDFValues[i2->first] = i2->second;
	}
	}*/
}

OctreeSF2::OctreeSF2(const OctreeSF2& other)
{
	m_RootNode = other.m_RootNode->clone();
	m_RootArea = other.m_RootArea;
	m_CellSize = other.m_CellSize;
	m_TriangleCache = other.m_TriangleCache;
}

OctreeSF2::~OctreeSF2()
{
	if (m_RootNode)
		delete m_RootNode;
}

std::shared_ptr<OctreeSF2> OctreeSF2::clone()
{
	return std::make_shared<OctreeSF2>(*this);
}

int OctreeSF2::countNodes()
{
	int counter = 0;
	m_RootNode->countNodes(counter);
	return counter;
}

int OctreeSF2::countLeaves()
{
	int counter = 0;
	m_RootNode->countLeaves(counter);
	return counter;
}

int OctreeSF2::countMemory()
{
	int counter = 0;
	m_RootNode->countMemory(counter);
	m_RootNode->markSharedVertices(false);
	return counter;
}

Ogre::Vector3 OctreeSF2::getCenterOfMass(float& totalMass)
{
	Ogre::Vector3 centerOfMass(0, 0, 0);
	totalMass = 0;
	m_RootNode->sumPositionsAndMass(m_RootArea, centerOfMass, totalMass);
	if (totalMass > 0) centerOfMass /= totalMass;
	return centerOfMass;
}

Ogre::Vector3 OctreeSF2::getCenterOfMass()
{
	float mass = 0.0f;
	return getCenterOfMass(mass);
}

void OctreeSF2::simplify()
{
	// int nodeMask;
	// m_RootNode = simplifyNode(m_RootNode, m_RootArea, nodeMask);
}

void OctreeSF2::generateTriangleCache()
{
	auto mesh = generateMesh();
	auto transformedMesh = std::make_shared<TransformedMesh>(mesh);
	std::cout << "Computing cache" << std::endl;
	mesh->computeTriangleNormals();
	transformedMesh->computeCache();
	m_TriangleCache.clearMeshes();
	m_TriangleCache.addMesh(transformedMesh);
	std::cout << "Generating BVH" << std::endl;
	m_TriangleCache.generateBVH<AABB>();
	std::cout << "Finished generating BVH" << std::endl;
}