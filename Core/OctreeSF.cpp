
#include <stack>
#include "OctreeSF.h"
#include "SolidGeometry.h"
#include "MarchingCubes.h"
#include "Mesh.h"

/******************************************************************************************
InnerNode
*******************************************************************************************/

OctreeSF::InnerNode::InnerNode(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	m_NodeType = INNER;
	/*for (int i = 0; i < 8; i++)
	{
	m_CornerSamples[i] = cornerSamples[i];
	m_CornerSamples[i]->sample->useCount++;
	}*/

	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
	{
		m_Children[i] = OctreeSF::createNode(subAreas[i], implicitSDF, sharedVertices);
	}
}

OctreeSF::InnerNode::~InnerNode()
{
	for (int i = 0; i < 8; i++)
	{
		// m_CornerSamples[i]->sample->useCount--;
		delete m_Children[i];
	}
}

OctreeSF::InnerNode::InnerNode(const InnerNode& rhs)
{
	m_NodeType = INNER;
	for (int i = 0; i < 8; i++)
	{
		m_Children[i] = rhs.m_Children[i]->clone();
	}
}

void OctreeSF::InnerNode::countNodes(int& counter) const
{
	counter++;
	for (int i = 0; i < 8; i++)
		m_Children[i]->countNodes(counter);
}

void OctreeSF::InnerNode::countLeaves(int& counter) const
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->countLeaves(counter);
}

void OctreeSF::InnerNode::countMemory(int& counter) const
{
	counter += sizeof(*this);
	for (int i = 0; i < 8; i++)
		m_Children[i]->countMemory(counter);
}

void OctreeSF::InnerNode::markSharedVertices(bool marked)
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->markSharedVertices(marked);
}

bool OctreeSF::InnerNode::rayIntersectUpdate(const Area& area, const Ray& ray, Ray::Intersection& intersection)
{
	if (!ray.intersectAABB(&area.toAABB().min, 0, intersection.t)) return false;
	Area subAreas[8];
	area.getSubAreas(subAreas);
	bool foundSomething = false;
	for (int i = 0; i < 8; i++)
	{
		if (m_Children[i]->rayIntersectUpdate(subAreas[i], ray, intersection))
			foundSomething = true;
	}
	return foundSomething;
}

void OctreeSF::InnerNode::generateVertices(vector<Vertex>& vertices)
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->generateVertices(vertices);
}
void OctreeSF::InnerNode::generateIndices(const Area& area, vector<unsigned int>& indices, vector<Vertex>& vertices) const
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
		m_Children[i]->generateIndices(subAreas[i], indices, vertices);
}

void OctreeSF::InnerNode::invert()
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->invert();
}

/******************************************************************************************
EmptyNode
*******************************************************************************************/

OctreeSF::EmptyNode::EmptyNode(const Area& area, const SolidGeometry& implicitSDF)
{
	m_NodeType = EMPTY;
	m_Sign = implicitSDF.getSign(area.toAABB().getCenter());
}

OctreeSF::EmptyNode::~EmptyNode()
{
}

void OctreeSF::EmptyNode::invert()
{
	m_Sign = !m_Sign;
}

/******************************************************************************************
GridNode
*******************************************************************************************/

OctreeSF::Node* OctreeSF::GridNode::clone() const
{
	GridNode* copy = new GridNode(*this);
	// perform deep copy
	for (auto i = copy->m_SurfaceEdges.begin(); i != copy->m_SurfaceEdges.end(); ++i)
	{
		i->sharedVertex->refCount++;
	}
	return copy;
}

void OctreeSF::GridNode::computeSigns(const Area& area, const SolidGeometry& implicitSDF)
{
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;

	// compute inner grid
	Ogre::Vector3 currentPos = area.m_MinRealPos;
	int index = 0;
	for (unsigned int x = 0; x < LEAF_SIZE_1D; x++)
	{
		for (unsigned int y = 0; y < LEAF_SIZE_1D; y++)
		{
			for (unsigned int z = 0; z < LEAF_SIZE_1D; z++)
			{
				m_Signs.set(index++, implicitSDF.getSign(currentPos));
				currentPos.z += stepSize;
			}
			currentPos.z = area.m_MinRealPos.z;
			currentPos.y += stepSize;
		}
		currentPos.y = area.m_MinRealPos.y;
		currentPos.x += stepSize;
	}
}

void OctreeSF::GridNode::computeEdges(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices, const std::bitset<LEAF_SIZE_3D>* ignoreEdges)
{
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	Ogre::Vector3 currentPos = area.m_MinRealPos;
	int index = 0;
    // copy signs first to array because std::bitset::operator[] is not very fast
    static bool signsArray[LEAF_SIZE_3D];
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        signsArray[i] = m_Signs[i];
	for (int x = 0; x < LEAF_SIZE_1D; x++)
	{
		for (int y = 0; y < LEAF_SIZE_1D; y++)
		{
			for (int z = 0; z < LEAF_SIZE_1D; z++)
			{
				Vector3i iPos(x, y, z);
                if (!ignoreEdges[0][index] && x < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + LEAF_SIZE_2D])
				{
					m_SurfaceEdges.emplace_back();
					if (y > 0 && y < LEAF_SIZE_1D_INNER && z > 0 && z < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 0, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 0, currentPos, stepSize, implicitSDF, sharedVertices);
				}
                if (!ignoreEdges[1][index] && y < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + LEAF_SIZE_1D])
				{
					m_SurfaceEdges.emplace_back();
					if (x > 0 && x < LEAF_SIZE_1D_INNER && z > 0 && z < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 1, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 1, currentPos, stepSize, implicitSDF, sharedVertices);
				}
                if (!ignoreEdges[2][index] && z < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + 1])
				{
					m_SurfaceEdges.emplace_back();
					if (y > 0 && y < LEAF_SIZE_1D_INNER && x > 0 && x < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 2, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 2, currentPos, stepSize, implicitSDF, sharedVertices);
				}
				index++;
				currentPos.z += stepSize;
			}
			currentPos.z = area.m_MinRealPos.z;
			currentPos.y += stepSize;
		}
		currentPos.y = area.m_MinRealPos.y;
		currentPos.x += stepSize;
	}
}

void OctreeSF::GridNode::computeEdges(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices)
{
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	Ogre::Vector3 currentPos = area.m_MinRealPos;
	int index = 0;
    // copy signs first to array because std::bitset::operator[] is not very fast
    static bool signsArray[LEAF_SIZE_3D];
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        signsArray[i] = m_Signs[i];
	for (int x = 0; x < LEAF_SIZE_1D; x++)
	{
		for (int y = 0; y < LEAF_SIZE_1D; y++)
        {
			for (int z = 0; z < LEAF_SIZE_1D; z++)
			{
                Vector3i iPos(x, y, z);
                if (x < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + LEAF_SIZE_2D])
				{
					m_SurfaceEdges.emplace_back();
					if (y > 0 && y < LEAF_SIZE_1D_INNER && z > 0 && z < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 0, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 0, currentPos, stepSize, implicitSDF, sharedVertices);
				}
                if (y < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + LEAF_SIZE_1D])
				{
					m_SurfaceEdges.emplace_back();
					if (x > 0 && x < LEAF_SIZE_1D_INNER && z > 0 && z < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 1, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 1, currentPos, stepSize, implicitSDF, sharedVertices);
				}
                if (z < LEAF_SIZE_1D_INNER && signsArray[index] != signsArray[index + 1])
				{
					m_SurfaceEdges.emplace_back();
					if (y > 0 && y < LEAF_SIZE_1D_INNER && x > 0 && x < LEAF_SIZE_1D_INNER)
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 2, currentPos, stepSize, implicitSDF);
					else
						m_SurfaceEdges.back().init(area.m_MinPos, iPos, 2, currentPos, stepSize, implicitSDF, sharedVertices);
                }
				index++;
				currentPos.z += stepSize;
			}
			currentPos.z = area.m_MinRealPos.z;
			currentPos.y += stepSize;
		}
		currentPos.y = area.m_MinRealPos.y;
		currentPos.x += stepSize;
	}
}

OctreeSF::GridNode::GridNode(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	m_NodeType = GRID;

	computeSigns(area, implicitSDF);
	computeEdges(area, implicitSDF, sharedVertices);
}

OctreeSF::GridNode::~GridNode()
{
	for (auto i = m_SurfaceEdges.begin(); i != m_SurfaceEdges.end(); ++i)
	{
		i->deleteSharedVertex();
	}
}

void OctreeSF::GridNode::countMemory(int& memoryCounter) const
{
	memoryCounter += sizeof(*this) + (int)m_SurfaceEdges.capacity() * sizeof(SurfaceEdge);
	for (auto i = m_SurfaceEdges.begin(); i != m_SurfaceEdges.end(); ++i)
	{
		if (!i->sharedVertex->marked)
		{
			memoryCounter += sizeof(SharedSurfaceVertex);
			i->sharedVertex->marked = true;
		}
	}
}

void OctreeSF::GridNode::markSharedVertices(bool marked)
{
	for (auto i = m_SurfaceEdges.begin(); i != m_SurfaceEdges.end(); ++i)
	{
		i->sharedVertex->marked = marked;
	}
}

void OctreeSF::GridNode::generateVertices(vector<Vertex>& vertices)
{
	// vertices.reserve(vertices.size() + m_SurfaceEdges.size());
	for (auto i = m_SurfaceEdges.begin(); i != m_SurfaceEdges.end(); ++i)
	{
		if (!i->sharedVertex->marked && i->sharedVertex->shared)
		{
			i->sharedVertex->vertexIndex = (int)vertices.size();
			i->sharedVertex->marked = true;
			vertices.push_back(i->sharedVertex->vertex);
		}
	}
}

bool OctreeSF::GridNode::rayIntersectUpdate(const Area& area, const Ray& ray, Ray::Intersection& intersection)
{
	if (!ray.intersectAABB(&area.toAABB().min, 0, intersection.t)) return false;
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	generateVertices(mesh->vertexBuffer);
	generateIndices(area, mesh->indexBuffer, mesh->vertexBuffer);
	markSharedVertices(false);
	mesh->computeTriangleNormals();
	std::shared_ptr<TransformedMesh> transformedMesh = std::make_shared<TransformedMesh>(mesh);
	transformedMesh->computeCache();
	BVHScene scene;
	scene.addMesh(transformedMesh);
	scene.generateBVH<AABB>();
	return (scene.getBVH()->rayIntersectUpdate(intersection, ray) != nullptr);
}

#include "TriangleLookupTable.h"
void OctreeSF::GridNode::generateIndices(const Area&, vector<unsigned int>& indices, vector<Vertex>& vertices) const
{
	static const SurfaceEdge* surfaceEdgeMaps[3][LEAF_SIZE_3D];
	for (auto i = m_SurfaceEdges.begin(); i != m_SurfaceEdges.end(); ++i)
	{
		surfaceEdgeMaps[i->direction][i->edgeIndex1] = &(*i);
	}

    // copy signs first to array because std::bitset::operator[] is not very fast
    static bool signsArray[LEAF_SIZE_3D];
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        signsArray[i] = m_Signs[i];

	int index = 0;
	for (int x = 0; x < LEAF_SIZE_1D_INNER; x++)
	{
		for (int y = 0; y < LEAF_SIZE_1D_INNER; y++)
		{
			for (int z = 0; z < LEAF_SIZE_1D_INNER; z++)
			{
                unsigned char corners = getCubeBitMask(index, signsArray);
				if (corners && corners != 255)
				{
					const std::vector<Triangle<int> >& tris = TLT::getSingleton().indexTable[corners];
					for (auto i2 = tris.begin(); i2 != tris.end(); ++i2)
					{
						const TLT::DirectedEdge& p1 = TLT::getSingleton().directedEdges[i2->p1];
						const TLT::DirectedEdge& p2 = TLT::getSingleton().directedEdges[i2->p2];
						const TLT::DirectedEdge& p3 = TLT::getSingleton().directedEdges[i2->p3];
						SharedSurfaceVertex* vert = surfaceEdgeMaps[p1.direction][index
							+ (p1.minCornerIndex & 1)
							+ ((p1.minCornerIndex & 2) >> 1) * LEAF_SIZE_1D
							+ ((p1.minCornerIndex & 4) >> 2) * LEAF_SIZE_2D]->sharedVertex;
						// if (vert->shared)
						{
							indices.push_back((int)(vert->vertexIndex));
						}
						/*else
						{
						indices.push_back((int)vertices.size());
						vertices.push_back(vert->vertex);
						}*/
						vert = surfaceEdgeMaps[p2.direction][index
							+ (p2.minCornerIndex & 1)
							+ ((p2.minCornerIndex & 2) >> 1) * LEAF_SIZE_1D
							+ ((p2.minCornerIndex & 4) >> 2) * LEAF_SIZE_2D]->sharedVertex;
						// if (vert->shared)
						{
							indices.push_back((int)(vert->vertexIndex));
						}
						/*else
						{
						indices.push_back((int)vertices.size());
						vertices.push_back(vert->vertex);
						}*/
						vert = surfaceEdgeMaps[p3.direction][index
							+ (p3.minCornerIndex & 1)
							+ ((p3.minCornerIndex & 2) >> 1) * LEAF_SIZE_1D
							+ ((p3.minCornerIndex & 4) >> 2) * LEAF_SIZE_2D]->sharedVertex;
						//if (vert->shared)
						{
							indices.push_back((int)(vert->vertexIndex));
						}
						/*else
						{
						indices.push_back((int)vertices.size());
						vertices.push_back(vert->vertex);
						}*/
					}
				}
				index++;
			}
			index++;
		}
		index += LEAF_SIZE_1D;
	}
}


void OctreeSF::GridNode::invert()
{
	m_Signs.flip();
}

unsigned char OctreeSF::GridNode::getCubeBitMask(int index, const bool* signsArray)
{
	unsigned char corners = 0;
    corners |= (unsigned char)signsArray[index];
    corners |= ((unsigned char)signsArray[index + 1] << 1);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_1D] << 2);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_1D + 1] << 3);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_2D] << 4);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_2D + 1] << 5);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_2D + LEAF_SIZE_1D] << 6);
    corners |= ((unsigned char)signsArray[index + LEAF_SIZE_2D + LEAF_SIZE_1D + 1] << 7);
	return corners;
}

void OctreeSF::GridNode::merge(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices)
{
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	GridNode otherNode;
	otherNode.computeSigns(area, implicitSDF);
	m_Signs |= otherNode.m_Signs;
	auto thisEdgesCopy = m_SurfaceEdges;
	m_SurfaceEdges.clear();
	std::bitset<LEAF_SIZE_3D> addedEdges[3];
	for (auto i = thisEdgesCopy.begin(); i != thisEdgesCopy.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			m_SurfaceEdges.push_back(*i);
			addedEdges[i->direction][i->edgeIndex1] = true;

			// sign changes in both nodes
			if (otherNode.m_Signs[i->edgeIndex1] != otherNode.m_Signs[i->edgeIndex2])
			{
				Vector3i minPos = fromIndex(i->edgeIndex1);
				Ogre::Vector3 globalPos = area.m_MinRealPos + minPos.toOgreVec() * stepSize;
				globalPos[i->direction] += stepSize * 0.5f;
				Sample s;
				implicitSDF.getSample(globalPos, s);
				if (s.signedDistance > i->sharedVertex->signedDistance)
				{
					i->sharedVertex->signedDistance = s.signedDistance;
					i->sharedVertex->vertex.position = s.closestSurfacePos;
					i->sharedVertex->vertex.normal = s.normal;
				}
			}
		}
		else i->deleteSharedVertex();
	}
	computeEdges(area, implicitSDF, sharedVertices, addedEdges);
}

void OctreeSF::GridNode::intersect(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*> *sharedVertices)
{
	// auto ts = Profiler::timestamp();
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	GridNode otherNode;
	otherNode.computeSigns(area, implicitSDF);
	m_Signs &= otherNode.m_Signs;
	auto thisEdgesCopy = m_SurfaceEdges;
	m_SurfaceEdges.clear();
	std::bitset<LEAF_SIZE_3D> addedEdges[3];
	for (auto i = thisEdgesCopy.begin(); i != thisEdgesCopy.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			m_SurfaceEdges.push_back(*i);
			addedEdges[i->direction][i->edgeIndex1] = true;

			// sign changes in both nodes
			if (otherNode.m_Signs[i->edgeIndex1] != otherNode.m_Signs[i->edgeIndex2])
			{
				Vector3i minPos = fromIndex(i->edgeIndex1);
				Ogre::Vector3 globalPos = area.m_MinRealPos + minPos.toOgreVec() * stepSize;
				Sample s;
				// Ogre::Vector3 rayDir(0,0,0);
				// rayDir[i->direction] = 1.0f;
				// implicitSDF.raycastClosest(Ray(globalPos, rayDir), s);
				globalPos[i->direction] += stepSize * 0.5f;
				implicitSDF.getSample(globalPos, s);
				if (s.signedDistance < i->sharedVertex->signedDistance)
				{
					i->sharedVertex->signedDistance = s.signedDistance;
					i->sharedVertex->vertex.position = s.closestSurfacePos;
					i->sharedVertex->vertex.normal = s.normal;
				}
			}
		}
		else i->deleteSharedVertex();
	}
	computeEdges(area, implicitSDF, sharedVertices, addedEdges);
	// Profiler::getSingleton().accumulateJobDuration("GridNode::intersect", ts);
}

void OctreeSF::GridNode::intersect(GridNode* otherNode)
{
	auto thisEdgesCopy = m_SurfaceEdges;
	m_SurfaceEdges.clear();
	std::bitset<LEAF_SIZE_3D> addedEdges[3];
	static int addedEdgesIndices[3][LEAF_SIZE_3D];
	for (auto i = thisEdgesCopy.begin(); i != thisEdgesCopy.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			m_SurfaceEdges.push_back(*i);
			addedEdges[i->direction][i->edgeIndex1] = true;
			addedEdgesIndices[i->direction][i->edgeIndex1] = (int)m_SurfaceEdges.size() - 1;
		}
		else i->deleteSharedVertex();
	}
	for (auto i = otherNode->m_SurfaceEdges.begin(); i != otherNode->m_SurfaceEdges.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			if (addedEdges[i->direction][i->edgeIndex1])
			{
				if (i->sharedVertex->signedDistance < m_SurfaceEdges[addedEdgesIndices[i->direction][i->edgeIndex1]].sharedVertex->signedDistance)
				{
					m_SurfaceEdges[addedEdgesIndices[i->direction][i->edgeIndex1]] = i->clone();
				}
			}
			else m_SurfaceEdges.push_back(i->clone());
		}
	}
}

void OctreeSF::GridNode::merge(GridNode* otherNode)
{
	auto thisEdgesCopy = m_SurfaceEdges;
	m_SurfaceEdges.clear();
	std::bitset<LEAF_SIZE_3D> addedEdges[3];
	static int addedEdgesIndices[3][LEAF_SIZE_3D];
	for (auto i = thisEdgesCopy.begin(); i != thisEdgesCopy.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			m_SurfaceEdges.push_back(*i);
			addedEdges[i->direction][i->edgeIndex1] = true;
			addedEdgesIndices[i->direction][i->edgeIndex1] = (int)m_SurfaceEdges.size() - 1;
		}
		else i->deleteSharedVertex();
	}
	for (auto i = otherNode->m_SurfaceEdges.begin(); i != otherNode->m_SurfaceEdges.end(); i++)
	{
		if (m_Signs[i->edgeIndex1] != m_Signs[i->edgeIndex2])
		{
			if (addedEdges[i->direction][i->edgeIndex1])
			{
				if (i->sharedVertex->signedDistance > m_SurfaceEdges[addedEdgesIndices[i->direction][i->edgeIndex1]].sharedVertex->signedDistance)
				{
					m_SurfaceEdges[addedEdgesIndices[i->direction][i->edgeIndex1]] = i->clone();
				}
			}
			else m_SurfaceEdges.push_back(i->clone());
		}
	}
}

/******************************************************************************************
OctreeSF
*******************************************************************************************/

OctreeSF::Node* OctreeSF::createNode(const Area& area, const SolidGeometry& implicitSDF, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (area.m_SizeExpo <= LEAF_EXPO && needsSubdivision)
		return new GridNode(area, implicitSDF, sharedVertices);

	if (needsSubdivision)
		return new InnerNode(area, implicitSDF, sharedVertices);

	return new EmptyNode(area, implicitSDF);
}

OctreeSF::Node* OctreeSF::intersect(Node* node, const SolidGeometry& implicitSDF, const Area& area, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (node->getNodeType() == Node::INNER && needsSubdivision)
	{
		InnerNode* innerNode = (InnerNode*)node;
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			innerNode->m_Children[i] = intersect(innerNode->m_Children[i], implicitSDF, subAreas[i], sharedVertices);
		return node;
	}
	if (!needsSubdivision)
	{
		if (implicitSDF.getSign(area.getCornerVecs(0).second))
			return node;
		delete node;
		return new EmptyNode(area, implicitSDF);    // createNode(area, implicitSDF, sharedVertices);
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (!emptyNode->m_Sign)
			return node;
		delete node;
		return createNode(area, implicitSDF, sharedVertices);

	}

	GridNode* gridNode = (GridNode*)node;
	gridNode->intersect(area, implicitSDF, sharedVertices);
	return node;
}

OctreeSF::Node* OctreeSF::merge(Node* node, const SolidGeometry& implicitSDF, const Area& area, Vector3iHashGrid<SharedSurfaceVertex*>* sharedVertices)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (node->getNodeType() == Node::INNER && needsSubdivision)
	{
		InnerNode* innerNode = (InnerNode*)node;
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			innerNode->m_Children[i] = merge(innerNode->m_Children[i], implicitSDF, subAreas[i], sharedVertices);
		return node;
	}
	if (!needsSubdivision)
	{
		if (!implicitSDF.getSign(area.getCornerVecs(0).second))
			return node;
		delete node;
		return createNode(area, implicitSDF, sharedVertices);
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_Sign)
			return node;
		delete node;
		return createNode(area, implicitSDF, sharedVertices);
	}

	GridNode* gridNode = (GridNode*)node;
	gridNode->merge(area, implicitSDF, sharedVertices);
	return node;
}

OctreeSF::Node* OctreeSF::intersectAlignedNode(Node* node, Node* otherNode, const Area& area)
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
	gridNode->intersect(otherGridNode);
	return node;
}

OctreeSF::Node* OctreeSF::subtractAlignedNode(Node* node, Node* otherNode, const Area& area)
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
	gridNode->intersect(otherGridNode);
	return node;
}

OctreeSF::Node* OctreeSF::mergeAlignedNode(Node* node, Node* otherNode, const Area& area)
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
	gridNode->merge(otherGridNode);
	return node;
}

std::shared_ptr<OctreeSF> OctreeSF::sampleSDF(SolidGeometry* otherSDF, int maxDepth)
{
	AABB aabb = otherSDF->getAABB();
	return sampleSDF(otherSDF, aabb, maxDepth);
}

std::shared_ptr<OctreeSF> OctreeSF::sampleSDF(SolidGeometry* otherSDF, const AABB& aabb, int maxDepth)
{
	auto ts = Profiler::timestamp();
	std::shared_ptr<OctreeSF> octreeSF = std::make_shared<OctreeSF>();
	Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
	float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
	octreeSF->m_CellSize = cubeSize / (1 << maxDepth);
	otherSDF->prepareSampling(aabb, octreeSF->m_CellSize);
	octreeSF->m_RootArea = Area(Vector3i(0, 0, 0), maxDepth, aabb.getMin(), cubeSize);
	Vector3iHashGrid<SharedSurfaceVertex*> sharedVertices[3];
	octreeSF->m_RootNode = octreeSF->createNode(octreeSF->m_RootArea, *otherSDF, sharedVertices);
	Profiler::printJobDuration("sampleSDF", ts);
	return octreeSF;
}

float OctreeSF::getInverseCellSize()
{
	return (float)(1 << m_RootArea.m_SizeExpo) / m_RootArea.m_RealSize;
}

AABB OctreeSF::getAABB() const
{
	return m_RootArea.toAABB();
}

void OctreeSF::generateVerticesAndIndices(vector<Vertex>& vertices, vector<unsigned int>& indices)
{
	auto tsTotal = Profiler::timestamp();
	int numLeaves = countLeaves();
	vertices.reserve(numLeaves * LEAF_SIZE_2D_INNER * 2);	// reasonable upper bound
	m_RootNode->generateVertices(vertices);
    // Profiler::printJobDuration("generateVertices", tsTotal);

    // auto tsIndices = Profiler::timestamp();
	indices.reserve(numLeaves * LEAF_SIZE_2D_INNER * 8);
	m_RootNode->generateIndices(m_RootArea, indices, vertices);
    // Profiler::printJobDuration("generateIndices", tsIndices);

	m_RootNode->markSharedVertices(false);
	Profiler::printJobDuration("generateVerticesAndIndices", tsTotal);
}

std::shared_ptr<Mesh> OctreeSF::generateMesh()
{
	auto ts = Profiler::timestamp();
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	generateVerticesAndIndices(mesh->vertexBuffer, mesh->indexBuffer);
	mesh->computeTriangleNormals();
	mesh->computeVertexNormals();
	Profiler::printJobDuration("generateMesh", ts);
	return mesh;
}

bool OctreeSF::rayIntersectClosest(const Ray& ray, Ray::Intersection& intersection)
{
	intersection.t = std::numeric_limits<float>::max();
	return m_RootNode->rayIntersectUpdate(m_RootArea, ray, intersection);
}

bool OctreeSF::intersectsSurface(const AABB& aabb) const
{
	if (m_TriangleCache.getBVH())
		return m_TriangleCache.getBVH()->intersectsAABB(aabb);
	return true;
}

void OctreeSF::subtract(SolidGeometry* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	Vector3iHashGrid<SharedSurfaceVertex*> sharedVertices[3];
	auto ts = Profiler::timestamp();
	// Profiler::getSingleton().createJob("computeSigns");
	m_RootNode = intersect(m_RootNode, OpInvertSDF(otherSDF), m_RootArea, sharedVertices);
	// Profiler::getSingleton().printJobDuration("computeSigns");
	Profiler::printJobDuration("Subtraction", ts);
}

void OctreeSF::intersect(SolidGeometry* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	auto ts = Profiler::timestamp();
	Vector3iHashGrid<SharedSurfaceVertex*> sharedVertices[3];
	m_RootNode = intersect(m_RootNode, *otherSDF, m_RootArea, sharedVertices);
	Profiler::printJobDuration("Intersection", ts);
}

void OctreeSF::merge(SolidGeometry* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	// auto ts = Profiler::timestamp();
	Vector3iHashGrid<SharedSurfaceVertex*> sharedVertices[3];
	m_RootNode = merge(m_RootNode, *otherSDF, m_RootArea, sharedVertices);
	// Profiler::printJobDuration("Merge", ts);
}

void OctreeSF::intersectAlignedOctree(OctreeSF* otherOctree)
{
	m_RootNode = intersectAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF::subtractAlignedOctree(OctreeSF* otherOctree)
{
	m_RootNode = subtractAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF::mergeAlignedOctree(OctreeSF* otherOctree)
{
	m_RootNode = mergeAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSF::resize(const AABB&)
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

OctreeSF::OctreeSF(const OctreeSF& other)
{
	m_RootNode = other.m_RootNode->clone();
	m_RootArea = other.m_RootArea;
	m_CellSize = other.m_CellSize;
	m_TriangleCache = other.m_TriangleCache;
}

OctreeSF::~OctreeSF()
{
	if (m_RootNode)
		delete m_RootNode;
}

std::shared_ptr<OctreeSF> OctreeSF::clone()
{
	return std::make_shared<OctreeSF>(*this);
}

int OctreeSF::countNodes()
{
	int counter = 0;
	m_RootNode->countNodes(counter);
	return counter;
}

int OctreeSF::countLeaves()
{
	int counter = 0;
	m_RootNode->countLeaves(counter);
	return counter;
}

int OctreeSF::countMemory()
{
	int counter = 0;
	m_RootNode->countMemory(counter);
	m_RootNode->markSharedVertices(false);
	return counter;
}

Ogre::Vector3 OctreeSF::getCenterOfMass(float& totalMass)
{
	Ogre::Vector3 centerOfMass(0, 0, 0);
	totalMass = 0;
	m_RootNode->sumPositionsAndMass(m_RootArea, centerOfMass, totalMass);
	if (totalMass > 0) centerOfMass /= totalMass;
	return centerOfMass;
}

Ogre::Vector3 OctreeSF::getCenterOfMass()
{
	float mass = 0.0f;
	return getCenterOfMass(mass);
}

void OctreeSF::simplify()
{
	// int nodeMask;
	// m_RootNode = simplifyNode(m_RootNode, m_RootArea, nodeMask);
}

void OctreeSF::generateTriangleCache()
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
