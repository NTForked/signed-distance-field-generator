
#include <stack>
#include "OctreeSF.h"
#include "SolidGeometry.h"
#include "MarchingCubes.h"
#include "Mesh.h"

/******************************************************************************************
InnerNode
*******************************************************************************************/

OctreeSF::InnerNode::InnerNode(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    m_NodeType = INNER;

    Area subAreas[8];
    area.getSubAreas(subAreas);
    for (int i = 0; i < 8; i++)
    {
        m_Children[i] = tree->createNode(subAreas[i], implicitSDF);
    }
}

OctreeSF::InnerNode::~InnerNode()
{
    for (int i = 0; i < 8; i++)
    {
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

void OctreeSF::InnerNode::forEachSurfaceNode(const Area& area, const std::function<void(GridNode*, const Area&)>& function)
{
    Area subAreas[8];
    area.getSubAreas(subAreas);
    for (int i = 0; i < 8; i++)
        m_Children[i]->forEachSurfaceNode(subAreas[i], function);
}

void OctreeSF::InnerNode::forEachSurfaceNode(const std::function<void(GridNode*)>& function)
{
    for (int i = 0; i < 8; i++)
        m_Children[i]->forEachSurfaceNode(function);
}

void OctreeSF::InnerNode::forEachSurfaceFace(const Area& area, const std::function<void(const Face&)>& function)
{
    Area subAreas[8];
    area.getSubAreas(subAreas);
    for (int i = 0; i < 8; i++)
    {
        for (unsigned char d = 0; d < 3; d++)
        {
            int neighborIndex = i + (1 << d);
            if (neighborIndex < 8)
            {
                forEachSurfaceFace(m_Children[i], m_Children[neighborIndex], subAreas[i], d, function);
            }
        }
        m_Children[i]->forEachSurfaceFace(area, function);
    }
}

void OctreeSF::InnerNode::forEachSurfaceFace(Node* n1, Node* n2, const Area& minArea, unsigned char normalDirection, const std::function<void(const Face&)>& function)
{
    if (n1->getNodeType() == EMPTY || n2->getNodeType() == EMPTY)
        return;
    if (n1->getNodeType() == GRID && n2->getNodeType() == GRID)
    {
        function(Face((GridNode*)n1, (GridNode*)n2, minArea, normalDirection));
        return;
    }
    if (n1->getNodeType() == INNER && n2->getNodeType() == INNER)
    {
        Area subAreas[8];
        minArea.getSubAreas(subAreas);
        InnerNode* innerNode1 = (InnerNode*)n1;
        InnerNode* innerNode2 = (InnerNode*)n2;
        unsigned char dim1 = (normalDirection + 1) % 3;
        unsigned char dim2 = (normalDirection + 2) % 3;
        unsigned char dimensionMask1 = 1 << normalDirection;
        // the children of the two inner nodes share 4 faces
        // for instance, for neighborDir = 0 this processes the pairs (100, 000), (101, 001), (110, 010), (111, 011)
        for (unsigned char i = 0; i < 2; i++)
        {
            for (unsigned char j = 0; j < 2; j++)
            {
                unsigned char index2 = (i << dim1) | (j << dim2);
                unsigned char index1 = dimensionMask1 | index2;
                forEachSurfaceFace(innerNode1->m_Children[index1], innerNode2->m_Children[index2], subAreas[index1], normalDirection, function);
            }
        }
    }
}

void OctreeSF::InnerNode::forEachSurfaceEdge(const Area& area, const std::function<void(const Edge&)>& function)
{
    Area subAreas[8];
    area.getSubAreas(subAreas);
    // there are six edges inside the node
    forEachSurfaceEdge(m_Children[0], m_Children[1], m_Children[2], m_Children[3], subAreas[0], 0, function);
    forEachSurfaceEdge(m_Children[0], m_Children[1], m_Children[4], m_Children[5], subAreas[0], 1, function);
    forEachSurfaceEdge(m_Children[0], m_Children[2], m_Children[4], m_Children[6], subAreas[0], 2, function);

    forEachSurfaceEdge(m_Children[1], m_Children[3], m_Children[5], m_Children[7], subAreas[0], 2, function);

    forEachSurfaceEdge(m_Children[2], m_Children[3], m_Children[6], m_Children[7], subAreas[0], 1, function);

    forEachSurfaceEdge(m_Children[4], m_Children[5], m_Children[6], m_Children[7], subAreas[0], 0, function);

    for (int i = 0; i < 8; i++)
    {
        m_Children[i]->forEachSurfaceEdge(area, function);
    }
}

void OctreeSF::InnerNode::forEachSurfaceEdge(Node* n1, Node* n2, Node* n3, Node* n4, const Area& minArea, unsigned char direction, const std::function<void(const Edge&)>& function)
{
    if (n1->getNodeType() == EMPTY || n2->getNodeType() == EMPTY || n3->getNodeType() == EMPTY || n4->getNodeType() == EMPTY)
        return;
    if (n1->getNodeType() == GRID && n2->getNodeType() == GRID && n3->getNodeType() == GRID && n4->getNodeType() == GRID)
    {
        function(Edge((GridNode*)n1, (GridNode*)n2, (GridNode*)n2, (GridNode*)n3, minArea, direction));
        return;
    }
    if (n1->getNodeType() == INNER && n2->getNodeType() == INNER)
    {
        Area subAreas[8];
        minArea.getSubAreas(subAreas);
        InnerNode* innerNode1 = (InnerNode*)n1;
        InnerNode* innerNode2 = (InnerNode*)n2;
        InnerNode* innerNode3 = (InnerNode*)n3;
        InnerNode* innerNode4 = (InnerNode*)n4;
        // the children of the inner nodes share two edges
        unsigned char dim1 = (direction + 1) % 3;
        unsigned char dim2 = (direction + 2) % 3;
        if (dim1 > dim2) std::swap(dim1, dim2);
        unsigned char dim1Mask = 1 << dim1;
        unsigned char dim2Mask = 1 << dim2;
        int n1Mask = dim1Mask | dim2Mask;
        int n2Mask = dim1Mask;
        int n3Mask = dim2Mask;
        for (int i = 0; i < 2; i++)
        {
            unsigned char commonMask = i << direction;
            forEachSurfaceEdge(innerNode1->m_Children[n1Mask | commonMask],
                    innerNode2->m_Children[n2Mask | commonMask],
                    innerNode3->m_Children[n3Mask | commonMask],
                    innerNode4->m_Children[commonMask], minArea, direction, function);
        }
    }
}

void OctreeSF::InnerNode::countNodes(int& counter) const
{
    counter++;
    for (int i = 0; i < 8; i++)
        m_Children[i]->countNodes(counter);
}

void OctreeSF::InnerNode::countMemory(int& counter) const
{
    counter += sizeof(*this);
    for (int i = 0; i < 8; i++)
        m_Children[i]->countMemory(counter);
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
    return copy;
}

void OctreeSF::GridNode::forEachSurfaceNode(const Area& area, const std::function<void(GridNode*, const Area&)>& function)
{
    function(this, area);
}

void OctreeSF::GridNode::forEachSurfaceNode(const std::function<void(GridNode*)>& function)
{
    function(this);
}

void OctreeSF::GridNode::computeSigns(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    // compute inner grid
    int index = 0;
    for (unsigned int x = 0; x < LEAF_SIZE_1D; x++)
    {
        for (unsigned int y = 0; y < LEAF_SIZE_1D; y++)
        {
            for (unsigned int z = 0; z < LEAF_SIZE_1D; z++)
            {
                Ogre::Vector3 currentPos = tree->getRealPos(area.m_MinPos + Vector3i(x, y, z));
                m_Signs[index++] = implicitSDF.getSign(currentPos);
            }
        }
    }
}

void OctreeSF::GridNode::computeEdges(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF, const bool ignoreEdges[3][LEAF_SIZE_3D])
{
}

void OctreeSF::GridNode::computeEdges(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    float stepSize = tree->m_CellSize;
    static const int SIGN_OFFSETS[] = { LEAF_SIZE_2D, LEAF_SIZE_1D, 1 };
    for (unsigned char d = 0; d < 3; d++)
    {
        // inner edges
        for (int x = (d != 0); x < LEAF_SIZE_1D_INNER; x++)
        {
            for (int y = (d != 1); y < LEAF_SIZE_1D_INNER; y++)
            {
                for (int z = (d != 2); z < LEAF_SIZE_1D_INNER; z++)
                {
                    int index = indexOf(x, y, z);
                    if (m_Signs[index] != m_Signs[index + SIGN_OFFSETS[d]])
                    {
                        Vector3i iPos(x, y, z);
                        Ogre::Vector3 currentPos = tree->getRealPos(area.m_MinPos + iPos);
                        m_InnerSurfaceEdges.emplace_back();
                        m_InnerSurfaceEdges.back().init(iPos, d, currentPos, stepSize, implicitSDF);
                    }
                }
            }
        }

        // face edges
        unsigned char fDir1 = (d + 1) % 3;
        unsigned char fDir2 = (d + 2) % 3;
        Vector3i iPos;
        iPos[d] = 0;
        for (iPos[fDir1] = 1; iPos[fDir1] < LEAF_SIZE_1D_INNER; iPos[fDir1]++)
        {
            for (iPos[fDir2] = 1; iPos[fDir2] < LEAF_SIZE_1D_INNER; iPos[fDir2]++)
            {
                int index = indexOf(iPos);
                if (m_Signs[index] != m_Signs[index + SIGN_OFFSETS[fDir1]])
                {
                    Ogre::Vector3 currentPos = tree->getRealPos(area.m_MinPos + iPos);
                    m_FaceSurfaceEdges[d].emplace_back();
                    m_FaceSurfaceEdges[d].back().init(iPos, fDir1, currentPos, stepSize, implicitSDF);
                }
                if (m_Signs[index] != m_Signs[index + SIGN_OFFSETS[fDir2]])
                {
                    Ogre::Vector3 currentPos = tree->getRealPos(area.m_MinPos + iPos);
                    m_FaceSurfaceEdges[d].emplace_back();
                    m_FaceSurfaceEdges[d].back().init(iPos, fDir2, currentPos, stepSize, implicitSDF);
                }
            }
        }

        // edges along octree edges
        iPos = Vector3i(0, 0, 0);
        for (iPos[d] = 0; iPos[d] < LEAF_SIZE_1D_INNER; iPos[d]++)
        {
            int index = indexOf(iPos);
            if (m_Signs[index] != m_Signs[index + SIGN_OFFSETS[d]])
            {
                Ogre::Vector3 currentPos = tree->getRealPos(area.m_MinPos + iPos);
                m_EdgeSurfaceEdges[d].emplace_back();
                m_EdgeSurfaceEdges[d].back().init(iPos, d, currentPos, stepSize, implicitSDF);
            }
        }
    }
}

void OctreeSF::GridNode::computeCubes()
{
    const SurfaceCube* surfaceEdgeMaps[3][LEAF_SIZE_3D_INNER];
    /*for (unsigned char d = 0; d < 3; d++)
    {
        memset(surfaceEdgeMaps[d], 0, LEAF_SIZE_3D * sizeof(SurfaceEdge*));
    }*/
    for (auto i = m_InnerSurfaceEdges.begin(); i != m_InnerSurfaceEdges.end(); ++i)
    {
        // vAssert(surfaceEdgeMaps[i->direction][i->edgeIndex1] == nullptr);
        surfaceEdgeMaps[i->direction][i->edgeIndex1] = &(*i);
    }
    for (unsigned char d = 0; d < 3; d++)
    {
        for (auto i = m_FaceSurfaceEdges[d].begin(); i != m_FaceSurfaceEdges[d].end(); ++i)
        {
            // vAssert(surfaceEdgeMaps[i->direction][i->edgeIndex1] == nullptr);
            surfaceEdgeMaps[i->direction][i->edgeIndex1] = &(*i);
        }
        for (auto i = m_EdgeSurfaceEdges[d].begin(); i != m_EdgeSurfaceEdges[d].end(); ++i)
        {
            // vAssert(surfaceEdgeMaps[i->direction][i->edgeIndex1] == nullptr);
            surfaceEdgeMaps[i->direction][i->edgeIndex1] = &(*i);
        }
    }

    for (unsigned int x = 0; x < LEAF_SIZE_1D_INNER; x++)
    {
        for (unsigned int y = 0; y < LEAF_SIZE_1D_INNER; y++)
        {
            for (unsigned int z = 0; z < LEAF_SIZE_1D_INNER; z++)
            {
                int index = indexOf(x, y, z);
                unsigned char mask = getCubeBitMask(indexOf(x, y, z), m_Signs);
                if (mask && mask != 255)
                {
                    m_SurfaceCubes.emplace_back((unsigned short)index);
                    m_SurfaceCubes.back().
                }
            }
        }
    }
}

OctreeSF::GridNode::GridNode(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    m_NodeType = GRID;

    computeSigns(tree, area, implicitSDF);
    computeEdges(tree, area, implicitSDF);
    computeCubes();
}

OctreeSF::GridNode::~GridNode()
{
}

void OctreeSF::GridNode::countMemory(int& memoryCounter) const
{
    memoryCounter += sizeof(*this);
    memoryCounter += (int)m_SurfaceCubes.capacity() * sizeof(SurfaceEdge);
    memoryCounter += (int)m_InnerSurfaceEdges.capacity() * sizeof(SurfaceEdge);
    for (int i = 0; i < 3; i++)
    {
        memoryCounter += (int)m_FaceSurfaceEdges[i].capacity() * sizeof(SurfaceEdge);
        memoryCounter += (int)m_EdgeSurfaceEdges[i].capacity() * sizeof(SurfaceEdge);
    }
}

void OctreeSF::GridNode::generateVertices(vector<Vertex>& vertices)
{
}

bool OctreeSF::GridNode::rayIntersectUpdate(const Area& area, const Ray& ray, Ray::Intersection& intersection)
{
    if (!ray.intersectAABB(&area.toAABB().min, 0, intersection.t)) return false;
    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    generateVertices(mesh->vertexBuffer);
    generateIndices(area, mesh->indexBuffer, mesh->vertexBuffer);
    mesh->computeTriangleNormals();
    std::shared_ptr<TransformedMesh> transformedMesh = std::make_shared<TransformedMesh>(mesh);
    transformedMesh->computeCache();
    BVHScene scene;
    scene.addMesh(transformedMesh);
    scene.generateBVH<AABB>();
    return (scene.getBVH()->rayIntersectUpdate(intersection, ray) != nullptr);
}

#include "TriangleLookupTable.h"
void OctreeSF::GridNode::generateIndices(const Area&, vector<unsigned int>& indices, vector<Vertex>&) const
{
}


void OctreeSF::GridNode::invert()
{
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        m_Signs[i] = !m_Signs[i];
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

void OctreeSF::GridNode::merge(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    /*float cellSize = tree->m_CellSize;
    GridNode otherNode;
    otherNode.computeSigns(tree, area, implicitSDF);
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        m_Signs[i] = m_Signs[i] || otherNode.m_Signs[i];
    auto thisEdgesCopy = m_SurfaceEdges;
    m_SurfaceEdges.clear();
    bool addedEdges[3][LEAF_SIZE_3D];
    memset(addedEdges[0], 0, LEAF_SIZE_3D);
    memset(addedEdges[1], 0, LEAF_SIZE_3D);
    memset(addedEdges[2], 0, LEAF_SIZE_3D);
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
                Ogre::Vector3 globalPos = tree->getRealPos(area.m_MinPos + minPos);
                Ogre::Vector3 insidePos = globalPos;
                if (otherNode.m_Signs[i->edgeIndex2])
                    insidePos = tree->getRealPos(area.m_MinPos + fromIndex(i->edgeIndex2));

                globalPos[i->direction] += cellSize * 0.5f;
                Sample s;
                implicitSDF.getSample(globalPos, s);
                Ogre::Vector3 newDiff = s.closestSurfacePos - insidePos;
                Ogre::Vector3 oldDiff = i->surfaceVertex->vertex.position - insidePos;
                for (int j = 0; j < 3; j++)
                {
                    if (newDiff[j] * newDiff[j] > oldDiff[j] * oldDiff[j])
                    {
                        i->surfaceVertex->vertex.position[j] = s.closestSurfacePos[j];
                        i->surfaceVertex->vertex.normal[j] = s.normal[j];
                    }
                }
                i->surfaceVertex->vertex.normal.normalise();
            }
        }
    }
    computeEdges(tree, area, implicitSDF, addedEdges);*/
}

void OctreeSF::GridNode::intersect(OctreeSF* tree, const Area& area, const SolidGeometry& implicitSDF)
{
    // auto ts = Profiler::timestamp();
    /*float cellSize = tree->m_CellSize;
    GridNode otherNode;
    otherNode.computeSigns(tree, area, implicitSDF);
    for (int i = 0; i < LEAF_SIZE_3D; i++)
        m_Signs[i] = m_Signs[i] && otherNode.m_Signs[i];
    auto thisEdgesCopy = m_SurfaceEdges;
    m_SurfaceEdges.clear();
    bool addedEdges[3][LEAF_SIZE_3D];
    memset(addedEdges[0], 0, LEAF_SIZE_3D);
    memset(addedEdges[1], 0, LEAF_SIZE_3D);
    memset(addedEdges[2], 0, LEAF_SIZE_3D);
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
                Ogre::Vector3 globalPos = tree->getRealPos(area.m_MinPos + minPos);
                Ogre::Vector3 insidePos = globalPos;
                if (otherNode.m_Signs[i->edgeIndex2])
                    insidePos = tree->getRealPos(area.m_MinPos + fromIndex(i->edgeIndex2));

                Sample s;
                // Ogre::Vector3 rayDir(0,0,0);
                // rayDir[i->direction] = 1.0f;
                // implicitSDF.raycastClosest(Ray(globalPos, rayDir), s);
                globalPos[i->direction] += cellSize * 0.5f;
                implicitSDF.getSample(globalPos, s);
                Ogre::Vector3 newDiff = s.closestSurfacePos - insidePos;
                Ogre::Vector3 oldDiff = i->surfaceVertex->vertex.position - insidePos;
                for (int j = 0; j < 3; j++)
                {
                    if (newDiff[j] * newDiff[j] < oldDiff[j] * oldDiff[j])
                    {
                        i->surfaceVertex->vertex.position[j] = s.closestSurfacePos[j];
                        i->surfaceVertex->vertex.normal[j] = s.normal[j];
                    }
                }
                i->surfaceVertex->vertex.normal.normalise();
            }
        }
    }
    computeEdges(tree, area, implicitSDF, addedEdges);*/
    // Profiler::getSingleton().accumulateJobDuration("GridNode::intersect", ts);
}

void OctreeSF::GridNode::intersect(GridNode* otherNode)
{
    // TODO
    /*auto thisEdgesCopy = m_SurfaceEdges;
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
    }*/
}

void OctreeSF::GridNode::merge(GridNode* otherNode)
{
    // TODO
    /*auto thisEdgesCopy = m_SurfaceEdges;
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
    }*/
}

/******************************************************************************************
OctreeSF
*******************************************************************************************/

Ogre::Vector3 OctreeSF::getRealPos(const Vector3i& cellIndex) const
{
    return m_RootArea.m_MinRealPos + cellIndex.toOgreVec() * m_CellSize;
}

OctreeSF::Node* OctreeSF::createNode(const Area& area, const SolidGeometry& implicitSDF)
{
    bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
    if (area.m_SizeExpo <= LEAF_EXPO && needsSubdivision)
        return new GridNodeImpl(this, area, implicitSDF);

    if (needsSubdivision)
        return new InnerNode(this, area, implicitSDF);

    return new EmptyNode(area, implicitSDF);
}

OctreeSF::Node* OctreeSF::intersect(Node* node, const SolidGeometry& implicitSDF, const Area& area)
{
    bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
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
        return new EmptyNode(area, implicitSDF);
    }
    if (node->getNodeType() == Node::EMPTY)
    {
        EmptyNode* emptyNode = (EmptyNode*)node;
        if (!emptyNode->m_Sign)
            return node;
        delete node;
        return createNode(area, implicitSDF);

    }

    GridNodeImpl* gridNode = (GridNodeImpl*)node;
    gridNode->intersect(this, area, implicitSDF);
    return node;
}

OctreeSF::Node* OctreeSF::merge(Node* node, const SolidGeometry& implicitSDF, const Area& area)
{
    bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
    if (node->getNodeType() == Node::INNER && needsSubdivision)
    {
        InnerNode* innerNode = (InnerNode*)node;
        Area subAreas[8];
        area.getSubAreas(subAreas);
        for (int i = 0; i < 8; i++)
            innerNode->m_Children[i] = merge(innerNode->m_Children[i], implicitSDF, subAreas[i]);
        return node;
    }
    if (!needsSubdivision)
    {
        if (!implicitSDF.getSign(area.getCornerVecs(0).second))
            return node;
        delete node;
        return createNode(area, implicitSDF);
    }
    if (node->getNodeType() == Node::EMPTY)
    {
        EmptyNode* emptyNode = (EmptyNode*)node;
        if (emptyNode->m_Sign)
            return node;
        delete node;
        return createNode(area, implicitSDF);
    }

    GridNodeImpl* gridNode = (GridNodeImpl*)node;
    gridNode->merge(this, area, implicitSDF);
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

    GridNodeImpl* gridNode = (GridNodeImpl*)node;
    GridNodeImpl* otherGridNode = (GridNodeImpl*)otherNode;
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

    GridNodeImpl* gridNode = (GridNodeImpl*)node;
    GridNodeImpl* otherGridNode = (GridNodeImpl*)otherNode;
    otherGridNode->invert();
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

    GridNodeImpl* gridNode = (GridNodeImpl*)node;
    GridNodeImpl* otherGridNode = (GridNodeImpl*)otherNode;
    gridNode->merge(otherGridNode);
    return node;
}

std::shared_ptr<OctreeSF> OctreeSF::sampleSDF(SolidGeometry* otherSDF, int maxDepth)
{
    AABB aabb = otherSDF->getAABB();
    aabb.addEpsilon(0.00001f);
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
    octreeSF->m_RootNode = octreeSF->createNode(octreeSF->m_RootArea, *otherSDF);
    Profiler::printJobDuration("OctreeSF::sampleSDF", ts);
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
    m_RootNode->forEachSurfaceNode([&vertices](GridNode* node) { node->generateVertices(vertices); });
    // Profiler::printJobDuration("generateVertices", tsTotal);

    // auto tsIndices = Profiler::timestamp();
    indices.reserve(numLeaves * LEAF_SIZE_2D_INNER * 8);
    m_RootNode->forEachSurfaceNode(m_RootArea,
                                   [&indices, &vertices](GridNode* node, const Area& area) { node->generateIndices(area, indices, vertices); });
    // Profiler::printJobDuration("generateIndices", tsIndices);
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
    auto ts = Profiler::timestamp();
    // Profiler::getSingleton().createJob("computeSigns");
    m_RootNode = intersect(m_RootNode, OpInvertSDF(otherSDF), m_RootArea);
    // Profiler::getSingleton().printJobDuration("computeSigns");
    Profiler::printJobDuration("Subtraction", ts);
}

void OctreeSF::intersect(SolidGeometry* otherSDF)
{
    otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
    auto ts = Profiler::timestamp();
    m_RootNode = intersect(m_RootNode, *otherSDF, m_RootArea);
    Profiler::printJobDuration("Intersection", ts);
}

void OctreeSF::merge(SolidGeometry* otherSDF)
{
    otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
    // auto ts = Profiler::timestamp();
    m_RootNode = merge(m_RootNode, *otherSDF, m_RootArea);
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
    m_RootNode->forEachSurfaceNode([&counter](GridNode*) { counter++;});
    return counter;
}

int OctreeSF::countMemory()
{
    int counter = 0;
    m_RootNode->countMemory(counter);
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
