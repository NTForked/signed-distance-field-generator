
#include <stack>
#include "OctreeSDF.h"
#include "SignedDistanceField.h"
#include "MarchingCubes.h"
#include "Mesh.h"
#include <bitset>

/*OctreeSDF::SharedLeafFace::SharedLeafFace(const Ogre::Vector3& pos, float stepSize, int dim1, int dim2, const SignedDistanceField3D& implicitSDF)
{
	Ogre::Vector3 currentPos = pos;
	for (int x = 0; x < LEAF_SIZE_1D; x++)
	{
		for (int y = 0; y < LEAF_SIZE_1D; y++)
		{
			at(x, y) = implicitSDF.getSample(currentPos);
			currentPos[dim2] += stepSize;
		}
		currentPos[dim2] = pos[dim2];
		currentPos[dim1] += stepSize;
	}
}

OctreeSDF::SharedLeafFace* OctreeSDF::lookupOrComputeXYFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
{
	bool created;
	SharedSamples& sample = sdfValues.lookupOrCreate(globalPos, created);
	if (created || !sample.faceXY)
	{
		sample.faceXY = new SharedLeafFace(realPos, stepSize, 0, 1, implicitSDF);
	}
	return sample.faceXY;
}

OctreeSDF::SharedLeafFace* OctreeSDF::lookupOrComputeXZFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
{
	bool created;
	SharedSamples& sample = sdfValues.lookupOrCreate(globalPos, created);
	if (created || !sample.faceXZ)
	{
		sample.faceXZ = new SharedLeafFace(realPos, stepSize, 0, 3, implicitSDF);
	}
	return sample.faceXY;
}

OctreeSDF::SharedLeafFace* OctreeSDF::lookupOrComputeYZFace(const Vector3i& globalPos, const Ogre::Vector3& realPos, float stepSize, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
{
	bool created;
	SharedSamples& sample = sdfValues.lookupOrCreate(globalPos, created);
	if (created || !sample.faceYZ)
	{
		sample.faceYZ = new SharedLeafFace(realPos, stepSize, 1, 2, implicitSDF);
	}
	return sample.faceXY;
}*/

/******************************************************************************************
Node constructors
*******************************************************************************************/

OctreeSDF::InnerNode::InnerNode(const Area& area, const SignedDistanceField3D& implicitSDF)
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
		m_Children[i] = OctreeSDF::createNode(subAreas[i], implicitSDF);
	}
}

OctreeSDF::InnerNode::~InnerNode()
{
	for (int i = 0; i < 8; i++)
	{
		// m_CornerSamples[i]->sample->useCount--;
		delete m_Children[i];
	}
}

OctreeSDF::InnerNode::InnerNode(const InnerNode& rhs)
{
	m_NodeType = INNER;
	for (int i = 0; i < 8; i++)
	{
		m_Children[i] = rhs.m_Children[i]->clone();
	}
}

OctreeSDF::EmptyNode::EmptyNode(Sample* cornerSamples)
{
	m_NodeType = EMPTY;
	for (int i = 0; i < 8; i++)
	{
		m_CornerSamples[i] = cornerSamples[i];
		// m_CornerSamples[i]->sample->useCount++;
	}
}

OctreeSDF::EmptyNode::~EmptyNode()
{
	/*for (int i = 0; i < 8; i++)
	{
		m_CornerSamples[i]->sample->useCount--;
	}*/
}

OctreeSDF::GridNode::GridNode(const Area& area, const SignedDistanceField3D& implicitSDF)
{
	m_NodeType = GRID;
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	/*m_Faces[0] = lookupOrComputeYZFace(area.m_MinPos, area.m_MinRealPos, faceStepSize, implicitSDF, sdfValues);
	m_Faces[1] = lookupOrComputeXZFace(area.m_MinPos, area.m_MinRealPos, faceStepSize, implicitSDF, sdfValues);
	m_Faces[2] = lookupOrComputeXYFace(area.m_MinPos, area.m_MinRealPos, faceStepSize, implicitSDF, sdfValues);
	m_Faces[3] = lookupOrComputeXYFace(area.m_MinPos + Vector3i(0, 0, area.m_SizeExpo), area.m_MinRealPos + Ogre::Vector3(0, 0, area.m_RealSize), faceStepSize, implicitSDF, sdfValues);
	m_Faces[4] = lookupOrComputeXZFace(area.m_MinPos + Vector3i(0, area.m_SizeExpo, 0), area.m_MinRealPos + Ogre::Vector3(0, area.m_RealSize, 0), faceStepSize, implicitSDF, sdfValues);
	m_Faces[5] = lookupOrComputeYZFace(area.m_MinPos + Vector3i(area.m_SizeExpo, 0, 0), area.m_MinRealPos + Ogre::Vector3(area.m_RealSize, 0, 0), faceStepSize, implicitSDF, sdfValues);
	for (int i = 0; i < 6; i++)
		m_Faces[i]->useCount++;*/

	// compute inner grid
	implicitSDF.getSamples(area, m_Samples);
	Ogre::Vector3 currentPos = area.m_MinRealPos;
	for (unsigned int x = 0; x < LEAF_SIZE_1D; x++)
	{
		for (unsigned int y = 0; y < LEAF_SIZE_1D; y++)
		{
			for (unsigned int z = 0; z < LEAF_SIZE_1D; z++)
			{
				implicitSDF.getSample(currentPos, at(x, y, z));
				// at(x, y, z) = implicitSDF.getSample(currentPos);
				currentPos.z += stepSize;
			}
			currentPos.z = area.m_MinRealPos.z;
			currentPos.y += stepSize;
		}
		currentPos.y = area.m_MinRealPos.y;
		currentPos.x += stepSize;
	}
}

OctreeSDF::GridNode::~GridNode()
{
	/*for (int i = 0; i < 6; i++)
		m_Faces[i]->useCount--;*/
}

OctreeSDF::Node* OctreeSDF::createNode(const Area& area, const SignedDistanceField3D& implicitSDF)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
	if (area.m_SizeExpo <= LEAF_EXPO && needsSubdivision)
		return new GridNode(area, implicitSDF);

	Sample cornerSamples[8];
	for (int i = 0; i < 8; i++)
		implicitSDF.getSample(area.getCornerVecs(i).second, cornerSamples[i]);

	if (needsSubdivision)	
		return new InnerNode(area, implicitSDF);

	return new EmptyNode(cornerSamples);
}

void OctreeSDF::InnerNode::countNodes(int& counter) const
{
	counter++;
	for (int i = 0; i < 8; i++)
		m_Children[i]->countNodes(counter);
}

void OctreeSDF::InnerNode::countLeaves(int& counter) const
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->countLeaves(counter);
}

void OctreeSDF::InnerNode::countMemory(int& counter) const
{
	counter += sizeof(*this);
	for (int i = 0; i < 8; i++)
		m_Children[i]->countMemory(counter);
}

void OctreeSDF::GridNode::getCubesToMarch(const Area& area, vector<Cube>& cubes) const
{
	/*std::bitset<LEAF_SIZE_3D_INNER> cubesWithSignChange;
	for (unsigned int x = 0; x < LEAF_SIZE_1D - 1; x++)
	{
		for (unsigned int y = 0; y < LEAF_SIZE_1D - 1; y++)
		{
			for (unsigned int z = 0; z < LEAF_SIZE_1D - 1; z++)
			{
			}
		}
	}*/
	for (unsigned int x = 0; x < LEAF_SIZE_1D_INNER; x++)
	{
		for (unsigned int y = 0; y < LEAF_SIZE_1D_INNER; y++)
		{
			for (unsigned int z = 0; z < LEAF_SIZE_1D_INNER; z++)
			{
				Cube cube;
				cube.cornerSamples[0] = &at(x, y, z);
				cube.cornerSamples[1] = &at(x, y, z + 1);
				cube.cornerSamples[2] = &at(x, y + 1, z);
				cube.cornerSamples[3] = &at(x, y + 1, z + 1);
				cube.cornerSamples[4] = &at(x + 1, y, z);
				cube.cornerSamples[5] = &at(x + 1, y, z + 1);
				cube.cornerSamples[6] = &at(x + 1, y + 1, z);
				cube.cornerSamples[7] = &at(x + 1, y + 1, z + 1);
				if (!OctreeSDF::allSignsAreEqual((const Sample**)cube.cornerSamples))
				{
					cube.posMin = area.m_MinPos + Vector3i(x, y, z);
					cubes.push_back(cube);
				}
			}
		}
	}
}

// dist1 * w + dist2 * (1-w) = 0
// => w  = dist2 / (dist2 - dist1)
static float getInterpolationWeight(float dist1, float dist2)
{
	return dist2 / (dist2 - dist1);
}

void OctreeSDF::GridNode::getSharedVertices(const Area& area, std::vector<Vertex>& vertices, Vector3iHashGrid<unsigned int>& indexMap) const
{
	Vector3i minKey = area.m_MinPos.doubleVec();
	float stepSize = area.m_RealSize / LEAF_SIZE_1D_INNER;
	for (unsigned int y = 0; y < LEAF_SIZE_1D_INNER; y++)
	{
		for (unsigned int z = 0; z < LEAF_SIZE_1D_INNER; z++)
		{
			const Sample& s1 = at(0, y, z);
			const Sample& s2 = at(0, y + 1, z);
			if (!signsAreEqual(s1.signedDistance, s2.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(0, y * stepSize, z * stepSize);
				v.position.y = MathMisc::linearInterpolation(v.position.y, v.position.y + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(0, 2 * y + 1, 2 * z)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(0, 2 * y + 1, 2 * z), vertices.size()));
			}
			const Sample& s3 = at(0, y, z + 1);
			if (!signsAreEqual(s1.signedDistance, s3.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(0, y * stepSize, z * stepSize);
				v.position.z = MathMisc::linearInterpolation(v.position.z, v.position.z + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(0, 2 * y, 2 * z + 1)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(0, 2 * y, 2 * z + 1), vertices.size()));
			}
		}
	}

	for (unsigned int x = 1; x < LEAF_SIZE_1D_INNER; x++)
	{
		for (unsigned int z = 0; z < LEAF_SIZE_1D_INNER; z++)
		{
			const Sample& s1 = at(x, 0, z);
			const Sample& s2 = at(x + 1, 0, z);
			if (!signsAreEqual(s1.signedDistance, s2.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				// vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(x * stepSize, 0, z * stepSize);
				v.position.x = MathMisc::linearInterpolation(v.position.x, v.position.x + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(2 * x + 1, 0, 2 * z)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(2 * x + 1, 0, 2 * z), vertices.size()));
			}
			const Sample& s3 = at(x, 0, z + 1);
			if (!signsAreEqual(s1.signedDistance, s3.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				// vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(x * stepSize, 0, z * stepSize);
				v.position.z = MathMisc::linearInterpolation(v.position.z, v.position.z + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(2 * x, 0, 2 * z + 1)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(2 * x, 0, 2 * z + 1), vertices.size()));
			}
		}
	}
	for (unsigned int x = 1; x < LEAF_SIZE_1D_INNER; x++)
	{
		for (unsigned int y = 1; y < LEAF_SIZE_1D_INNER; y++)
		{
			const Sample& s1 = at(x, y, 0);
			const Sample& s2 = at(x + 1, y, 0);
			if (!signsAreEqual(s1.signedDistance, s2.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				// vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(x * stepSize, y * stepSize, 0);
				v.position.x = MathMisc::linearInterpolation(v.position.x, v.position.x + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(2 * x + 1, 2 * y, 0)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(2 * x + 1, 2 * y, 0), vertices.size()));
			}
			const Sample& s3 = at(x, y + 1, 0);
			if (!signsAreEqual(s1.signedDistance, s3.signedDistance))
			{
				float w = getInterpolationWeight(s1.signedDistance, s2.signedDistance);
				// vertices.emplace_back();
				Vertex& v = vertices.back();
				v.position = area.m_MinRealPos + Ogre::Vector3(x * stepSize, y * stepSize, 0);
				v.position.y = MathMisc::linearInterpolation(v.position.y, v.position.y + stepSize, w);
				// vAssert(!indexMap.hasKey(minKey + Vector3i(2 * x, 2 * y + 1, 0)));
				// indexMap.insertUnsafe(std::make_pair(minKey + Vector3i(2 * x, 2 * y + 1, 0), vertices.size()));
			}
		}
	}
}

void OctreeSDF::InnerNode::getCubesToMarch(const Area& area, vector<Cube>& cubes) const
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
		m_Children[i]->getCubesToMarch(subAreas[i], cubes);
}

void OctreeSDF::InnerNode::getSharedVertices(const Area& area, std::vector<Vertex>& vertices, Vector3iHashGrid<unsigned int>& indexMap) const
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
		m_Children[i]->getSharedVertices(subAreas[i], vertices, indexMap);
}

void OctreeSDF::InnerNode::invert()
{
	for (int i = 0; i < 8; i++)
		m_Children[i]->invert();
}

void OctreeSDF::EmptyNode::invert()
{
	for (int i = 0; i < 8; i++)
		m_CornerSamples[i].signedDistance *= -1.0f;
}

void OctreeSDF::GridNode::invert()
{
	for (int i = 0; i < LEAF_SIZE_3D; i++)
		m_Samples[i].signedDistance *= -1.0f;
}

OctreeSDF::Node* OctreeSDF::intersect(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area)
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
		if (implicitSDF.getSample(area.getCornerVecs(0).second).signedDistance >= 0)
			return node;
		delete node;
		return createNode(area, implicitSDF);
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		return createNode(area, implicitSDF);

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode otherGridNode(area, implicitSDF);
	for (int i = 0; i < LEAF_SIZE_3D; i++)
	{
		if (otherGridNode.m_Samples[i].signedDistance < gridNode->m_Samples[i].signedDistance)
			gridNode->m_Samples[i] = otherGridNode.m_Samples[i];
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::subtract(Node* node, const SignedDistanceField3D& implicitSDF, const Area& area)
{
	bool needsSubdivision = implicitSDF.cubeNeedsSubdivision(area);
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
		if (implicitSDF.getSample(area.getCornerVecs(0).second).signedDistance < 0)
			return node;
		delete node;
		Node* newNode = createNode(area, implicitSDF);
		newNode->invert();
		return newNode;
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		Node* newNode = createNode(area, implicitSDF);
		newNode->invert();
		return newNode;

	}

	GridNode* gridNode = (GridNode*)node;
	GridNode otherGridNode(area, implicitSDF);
	for (int i = 0; i < LEAF_SIZE_3D; i++)
	{
		if (-otherGridNode.m_Samples[i].signedDistance < gridNode->m_Samples[i].signedDistance)
		{
			gridNode->m_Samples[i] = otherGridNode.m_Samples[i];
			gridNode->m_Samples[i].signedDistance *= -1.0f;
		}
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::intersectAlignedNode(Node* node, Node* otherNode, const Area& area)
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
		if (otherEmptyNode->m_CornerSamples[0].signedDistance >= 0)
			return node;
		delete node;
		return otherNode->clone();
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		return otherNode->clone();
		
	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	for (int i = 0; i < LEAF_SIZE_3D; i++)
	{
		if (otherGridNode->m_Samples[i].signedDistance < gridNode->m_Samples[i].signedDistance)
			gridNode->m_Samples[i] = otherGridNode->m_Samples[i];
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::subtractAlignedNode(Node* node, Node* otherNode, const Area& area)
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
		if (otherEmptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		Node* inverted = otherNode->clone();
		inverted->invert();
		return inverted;
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		Node* inverted = otherNode->clone();
		inverted->invert();
		return inverted;
		
	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	for (int i = 0; i < LEAF_SIZE_3D; i++)
	{
		if (-otherGridNode->m_Samples[i].signedDistance < gridNode->m_Samples[i].signedDistance)
		{
			gridNode->m_Samples[i] = otherGridNode->m_Samples[i];
			gridNode->m_Samples[i].signedDistance *= -1.0f;
		}
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::mergeAlignedNode(Node* node, Node* otherNode, const Area& area)
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
		if (otherEmptyNode->m_CornerSamples[0].signedDistance < 0)
			return node;
		delete node;
		return otherNode->clone();
	}
	if (node->getNodeType() == Node::EMPTY)
	{
		EmptyNode* emptyNode = (EmptyNode*)node;
		if (emptyNode->m_CornerSamples[0].signedDistance >= 0)
			return node;
		delete node;
		return otherNode->clone();
		
	}

	GridNode* gridNode = (GridNode*)node;
	GridNode* otherGridNode = (GridNode*)otherNode;
	for (int i = 0; i < LEAF_SIZE_3D; i++)
	{
		if (otherGridNode->m_Samples[i].signedDistance > gridNode->m_Samples[i].signedDistance)
			gridNode->m_Samples[i] = otherGridNode->m_Samples[i];
	}
	return node;
}

std::shared_ptr<OctreeSDF> OctreeSDF::sampleSDF(SignedDistanceField3D* otherSDF, int maxDepth)
{
	AABB aabb = otherSDF->getAABB();
	Ogre::Vector3 epsilonVec(0.000001f, 0.000001f, 0.000001f);
	aabb.min -= epsilonVec;
	aabb.max += epsilonVec;
	return sampleSDF(otherSDF, aabb, maxDepth);
}

std::shared_ptr<OctreeSDF> OctreeSDF::sampleSDF(SignedDistanceField3D* otherSDF, const AABB& aabb, int maxDepth)
{
	std::shared_ptr<OctreeSDF> octreeSDF = std::make_shared<OctreeSDF>();
	Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
	float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
	octreeSDF->m_CellSize = cubeSize / (1 << maxDepth);
	otherSDF->prepareSampling(aabb, octreeSDF->m_CellSize);
	octreeSDF->m_RootArea = Area(Vector3i(0, 0, 0), maxDepth, aabb.getMin(), cubeSize);
	octreeSDF->m_RootNode = octreeSDF->createNode(octreeSDF->m_RootArea, *otherSDF);
	return octreeSDF;
}

float OctreeSDF::getInverseCellSize()
{
	return (float)(1 << m_RootArea.m_SizeExpo) / m_RootArea.m_RealSize;
}

AABB OctreeSDF::getAABB() const
{
	return m_RootArea.toAABB();
}

#include "Profiler.h"
vector<OctreeSDF::Cube> OctreeSDF::getCubesToMarch()
{
	int numLeaves = countLeaves();
	/*std::cout << "Reserving " << numLeaves * LEAF_SIZE_2D_INNER << std::endl;
	std::vector<Vertex> sharedVertices;
	sharedVertices.reserve(numLeaves * LEAF_SIZE_2D_INNER);
	Vector3iHashGrid<unsigned int> indexMap;
	indexMap.rehash(numLeaves * LEAF_SIZE_2D_INNER);
	auto ts = Profiler::timestamp();
	m_RootNode->getSharedVertices(m_RootArea, sharedVertices, indexMap);
	Profiler::printJobDuration("getSharedVertices", ts);
	std::cout << "Shared vertices: " << sharedVertices.size() << std::endl;*/

	auto ts = Profiler::timestamp();
	vector<Cube> cubes;
	cubes.reserve(numLeaves * LEAF_SIZE_2D_INNER * 2);		// reasonable upper bound
	std::stack<Node*> nodes;
	m_RootNode->getCubesToMarch(m_RootArea, cubes);
	Profiler::printJobDuration("getCubesToMarch", ts);
	return cubes;
}

OctreeSDF::Sample OctreeSDF::getSample(const Ogre::Vector3& point) const
{
	return m_RootNode->getSample(m_RootArea, point);
}

bool OctreeSDF::intersectsSurface(const AABB& aabb) const
{
	if (m_TriangleCache.getBVH())
		return m_TriangleCache.getBVH()->intersectsAABB(aabb);
	return true;
}

void OctreeSDF::subtract(SignedDistanceField3D* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	auto ts = Profiler::timestamp();
	m_RootNode = subtract(m_RootNode, *otherSDF, m_RootArea);
	Profiler::printJobDuration("Subtraction", ts);
}

void OctreeSDF::intersect(SignedDistanceField3D* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	auto ts = Profiler::timestamp();
	m_RootNode = intersect(m_RootNode, *otherSDF, m_RootArea);
	Profiler::printJobDuration("Intersection", ts);
}

void OctreeSDF::intersectAlignedOctree(OctreeSDF* otherOctree)
{
	m_RootNode = intersectAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSDF::subtractAlignedOctree(OctreeSDF* otherOctree)
{
	m_RootNode = subtractAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSDF::mergeAlignedOctree(OctreeSDF* otherOctree)
{
	m_RootNode = mergeAlignedNode(m_RootNode, otherOctree->m_RootNode, m_RootArea);
}

void OctreeSDF::resize(const AABB& aabb)
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

void OctreeSDF::merge(SignedDistanceField3D* otherSDF)
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

OctreeSDF::OctreeSDF(const OctreeSDF& other)
{
	m_RootNode = other.m_RootNode->clone();
	m_RootArea = other.m_RootArea;
	m_CellSize = other.m_CellSize;
	m_TriangleCache = other.m_TriangleCache;
}

OctreeSDF::~OctreeSDF()
{
	if (m_RootNode)
		delete m_RootNode;
}

std::shared_ptr<OctreeSDF> OctreeSDF::clone()
{
	return std::make_shared<OctreeSDF>(*this);
}

int OctreeSDF::countNodes()
{
	int counter = 0;
	m_RootNode->countNodes(counter);
	return counter;
}

int OctreeSDF::countLeaves()
{
	int counter = 0;
	m_RootNode->countLeaves(counter);
	return counter;
}

int OctreeSDF::countMemory()
{
	int counter = 0;
	m_RootNode->countMemory(counter);
	return counter;
}

Ogre::Vector3 OctreeSDF::getCenterOfMass(float& totalMass)
{
	Ogre::Vector3 centerOfMass(0, 0, 0);
	totalMass = 0;
	m_RootNode->sumPositionsAndMass(m_RootArea, centerOfMass, totalMass);
	if (totalMass > 0) centerOfMass /= totalMass;
	return centerOfMass;
}

Ogre::Vector3 OctreeSDF::getCenterOfMass()
{
	float mass = 0.0f;
	return getCenterOfMass(mass);
}

void OctreeSDF::simplify()
{
	// int nodeMask;
	// m_RootNode = simplifyNode(m_RootNode, m_RootArea, nodeMask);
}

std::shared_ptr<Mesh> OctreeSDF::generateMesh()
{
	std::vector<Cube> cubes = getCubesToMarch();
	return MarchingCubes::marchSDF(cubes, getInverseCellSize(), getAABB().min);
}

void OctreeSDF::generateTriangleCache()
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