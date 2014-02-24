
#include <stack>
#include "OctreeSDF.h"
#include "SignedDistanceField.h"
#include "MarchingCubes.h"
#include "Mesh.h"

OctreeSDF::Node* OctreeSDF::cloneNode(Node* node, const Area& area, const SignedDistanceGrid& sdfValues, SignedDistanceGrid& clonedSDFValues)
{
	Area subAreas[8];
	area.getSubAreas(subAreas);
	for (int i = 0; i < 8; i++)
	{
		Vector3i globalPos = area.getCorner(i);
		auto find = sdfValues.find(globalPos);
		vAssert(find != sdfValues.end())
		clonedSDFValues[globalPos] = find->second;
	}
	if (!node) return nullptr;
	Node* cloned = allocNode();
	for (int i = 0; i < 8; i++)
	{
		cloned->m_Children[i] = cloneNode(node->m_Children[i], subAreas[i], sdfValues, clonedSDFValues);
	}
	return cloned;
}

OctreeSDF::Sample OctreeSDF::lookupOrComputeSample(int corner, const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
{
	auto vecs = area.getCornerVecs(corner);
	auto tryInsert = sdfValues.insert(std::make_pair(vecs.first, Sample(0.0f)));
	if (tryInsert.second)
	{
		tryInsert.first->second = implicitSDF.getSample(vecs.second);
	}
	return tryInsert.first->second;
}

OctreeSDF::Sample OctreeSDF::lookupSample(int corner, const Area& area, const SignedDistanceGrid& sdfValues)
{
	auto find = sdfValues.find(area.getCorner(corner));
	vAssert(find != sdfValues.end())
		return find->second;
}

OctreeSDF::Sample OctreeSDF::lookupSample(int corner, const Area& area) const
{
	return lookupSample(corner, area, m_SDFValues);
}


OctreeSDF::Node* OctreeSDF::createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
{
	int nodeTypeMask = 0;
	return createNode(area, implicitSDF, sdfValues, nodeTypeMask);
}

OctreeSDF::Node* OctreeSDF::createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues, int& nodeTypeMask)
{
	nodeTypeMask = 3;
	if (area.m_SizeExpo <= 0 ||
		!implicitSDF.cubeIntersectsSurface(area))
	{
		float signedDistances[8];
		for (int i = 0; i < 8; i++)
		{
			signedDistances[i] = lookupOrComputeSample(i, area, implicitSDF, sdfValues).signedDistance;
		}

		bool mono = allSignsAreEqual(signedDistances);
		if (area.m_SizeExpo <= 0 || mono)
		{
			if (mono)
			{
				if (signedDistances[0] > 0.0f) nodeTypeMask = 1;
				else nodeTypeMask = 2;
			}
			return nullptr;	// leaf
		}
	}

	// create inner node
	Area subAreas[8];
	area.getSubAreas(subAreas);
	Node* node = allocNode();
	int childrenMask = 0;
	for (int i = 0; i < 8; i++)
	{
		int childMask;
		node->m_Children[i] = createNode(subAreas[i], implicitSDF, sdfValues, childMask);
		childrenMask |= childMask;
	}
	if (childrenMask != 3)
	{
		nodeTypeMask = childrenMask;
		deallocNode(node);
		node = nullptr;
	}
	return node;
}

// Computes a lower and upper bound inside the area given the 8 corner signed distances.
void OctreeSDF::getLowerAndUpperBound(Node* node, const Area& area, float* signedDistances, float& lowerBound, float& upperBound) const
{
	if (node) area.getLowerAndUpperBound(signedDistances, lowerBound, upperBound);
	else
	{	// if it's a leaf we can do even better and just return min and max of the corner values.
		lowerBound = std::numeric_limits<float>::max();
		upperBound = std::numeric_limits<float>::min();
		for (int i = 0; i < 8; i++)
		{
			lowerBound = std::min(lowerBound, signedDistances[i]);
			upperBound = std::max(upperBound, signedDistances[i]);
		}
	}
}

void OctreeSDF::countNodes(Node* node, const Area& area, int& counter)
{
	counter++;
	if (node)
	{
		vAssert(area.m_SizeExpo > 0)
			Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			countNodes(node->m_Children[i], subAreas[i], counter);
	}
}

void OctreeSDF::getCubesToMarch(Node* node, const Area& area, vector<Cube>& cubes)
{
	if (node)
	{
		vAssert(area.m_SizeExpo > 0)
			Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			getCubesToMarch(node->m_Children[i], subAreas[i], cubes);
	}
	else
	{	// leaf
		Cube cube;
		cube.posMin = area.m_MinPos;
		for (int i = 0; i < 8; i++)
		{
			cube.cornerSamples[i] = lookupSample(i, area);
		}
		if (allSignsAreEqual(cube.cornerSamples)) return;
		if (area.m_SizeExpo <= 0)
			cubes.push_back(cube);
		else
		{
			interpolateLeaf(area);
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
				getCubesToMarch(nullptr, subAreas[i], cubes);
		}
	}
}
OctreeSDF::Sample OctreeSDF::getSample(Node* node, const Area& area, const Ogre::Vector3& point) const
{
	if (!node)
	{
		float invNodeSize = 1.0f / area.m_RealSize;
		float weights[3];
		weights[0] = (point.x - area.m_MinRealPos.x) * invNodeSize;
		weights[1] = (point.y - area.m_MinRealPos.y) * invNodeSize;
		weights[2] = (point.z - area.m_MinRealPos.z) * invNodeSize;
		Sample cornerSamples[8];
		for (int i = 0; i < 8; i++)
			cornerSamples[i] = lookupSample(i, area);
		return MathMisc::trilinearInterpolation(cornerSamples, weights);
	}
	else
	{
		Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
		{
			if (subAreas[i].containsPoint(point, 0.0001f))
			{
				return getSample(node->m_Children[i], subAreas[i], point);
			}
		}
	}
	return Sample(-99999.0f);		// should never occur
}

OctreeSDF::Node* OctreeSDF::intersect(Node* node, Node* otherNode, const Area& area, SignedDistanceGrid& otherSDF, SignedDistanceGrid& newSDF)
{
	// compute signed distances for this node and the area of the other sdf
	float otherSignedDistances[8];
	float thisSignedDistances[8];
	for (int i = 0; i < 8; i++)
	{
		auto vecs = area.getCornerVecs(i);
		Vector3i globalPos = vecs.first;
		auto find1 = m_SDFValues.find(globalPos);
		auto find2 = otherSDF.find(globalPos);
		vAssert(find1 != m_SDFValues.end());
		vAssert(find2 != otherSDF.end());
		thisSignedDistances[i] = find1->second.signedDistance;
		otherSignedDistances[i] = find2->second.signedDistance;

		if (otherSignedDistances[i] < thisSignedDistances[i])
			newSDF[globalPos] = find2->second;
		else newSDF[globalPos] = find1->second;
	}

	float thisLowerBound, thisUpperBound;
	getLowerAndUpperBound(node, area, thisSignedDistances, thisLowerBound, thisUpperBound);

	float otherLowerBound, otherUpperBound;
	getLowerAndUpperBound(otherNode, area, otherSignedDistances, otherLowerBound, otherUpperBound);

	// Empty space stays empty - end of story.
	if (thisUpperBound < 0) return node;

	if (otherUpperBound < thisLowerBound)
	{	// this node is replaced with the other sdf
		if (node) deallocNode(node);
		return cloneNode(otherNode, area, otherSDF, newSDF);
	}
	else if (otherLowerBound > thisUpperBound)
	{	// no change for this node
		return node;
	}

	if (node)
	{	// need to recurse to node children
		vAssert(area.m_SizeExpo > 0)
		Area subAreas[8];
		area.getSubAreas(subAreas);
		if (!otherNode)
		{
			interpolateLeaf(area, otherSDF);
			for (int i = 0; i < 8; i++)
				node->m_Children[i] = intersect(node->m_Children[i], nullptr, subAreas[i], otherSDF, newSDF);
		}
		else
		{
			for (int i = 0; i < 8; i++)
				node->m_Children[i] = intersect(node->m_Children[i], otherNode->m_Children[i], subAreas[i], otherSDF, newSDF);
		}

	}
	else
	{	// it's a leaf in the octree
		if (area.m_SizeExpo > 0 && otherNode)
		{
			// need to subdivide this node
			node = allocNode();
			interpolateLeaf(area);
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
				node->m_Children[i] = intersect(nullptr, otherNode->m_Children[i], subAreas[i], otherSDF, newSDF);
		}
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::intersect(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache)
{
	// compute signed distances for this node and the area of the other sdf
	float otherSignedDistances[8];
	float thisSignedDistances[8];
	for (int i = 0; i < 8; i++)
	{
		auto vecs = area.getCornerVecs(i);
		Sample otherSample = lookupOrComputeSample(i, area, otherSDF, otherSDFCache);
		Vector3i globalPos = vecs.first;
		auto find = m_SDFValues.find(globalPos);
		vAssert(find != m_SDFValues.end())
			otherSignedDistances[i] = otherSample.signedDistance;
		thisSignedDistances[i] = find->second.signedDistance;

		if (otherSignedDistances[i] < thisSignedDistances[i])
			newSDF[globalPos] = otherSample;
		else newSDF[globalPos] = find->second;
	}

	float thisLowerBound, thisUpperBound;
	getLowerAndUpperBound(node, area, thisSignedDistances, thisLowerBound, thisUpperBound);

	// empty space stays empty - end of story
	if (thisUpperBound < 0) return node;

	// compute a lower and upper bound for this node and the other sdf
	float otherLowerBound, otherUpperBound;
	bool containsSurface = otherSDF.cubeIntersectsSurface(area);
	if (containsSurface)
		area.getLowerAndUpperBound(otherSignedDistances, otherLowerBound, otherUpperBound);
	else
		area.getLowerAndUpperBoundOptimistic(otherSignedDistances, otherLowerBound, otherUpperBound);

	if (otherUpperBound < thisLowerBound)
	{	// this node is replaced with the other sdf
		if (node) deallocNode(node);
		return createNode(area, otherSDF, newSDF);
	}
	else if (otherLowerBound > thisUpperBound)
	{	// no change for this node
		return node;
	}

	if (node)
	{	// need to recurse to node children
		vAssert(area.m_SizeExpo > 0)
			Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			node->m_Children[i] = intersect(node->m_Children[i], subAreas[i], otherSDF, newSDF, otherSDFCache);
	}
	else
	{	// it's a leaf in the octree
		if (area.m_SizeExpo > 0 && containsSurface)
		{
			// need to subdivide this node
			node = allocNode();
			interpolateLeaf(area);
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
				node->m_Children[i] = intersect(nullptr, subAreas[i], otherSDF, newSDF, otherSDFCache);
		}
	}
	return node;
}

OctreeSDF::Node* OctreeSDF::merge(Node* node, const Area& area, const SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache)
{
	// if otherSDF does not overlap with the node AABB we can stop here
	if (!area.toAABB().intersectsAABB(otherSDF.getAABB()))
		return node;

	// compute signed distances for this node and the area of the other sdf
	float otherSignedDistances[8];
	float thisSignedDistances[8];
	for (int i = 0; i < 8; i++)
	{
		auto vecs = area.getCornerVecs(i);
		Sample otherSample = lookupOrComputeSample(i, area, otherSDF, otherSDFCache);
		Vector3i globalPos = vecs.first;
		auto find = m_SDFValues.find(globalPos);
		vAssert(find != m_SDFValues.end())
			otherSignedDistances[i] = otherSample.signedDistance;
		thisSignedDistances[i] = find->second.signedDistance;

		if (otherSignedDistances[i] > thisSignedDistances[i])
			newSDF[globalPos] = otherSample;
		else newSDF[globalPos] = find->second;
	}

	// compute a lower and upper bound for this node and the other sdf
	float otherLowerBound, otherUpperBound;
	area.getLowerAndUpperBound(otherSignedDistances, otherLowerBound, otherUpperBound);

	float thisLowerBound, thisUpperBound;
	getLowerAndUpperBound(node, area, thisSignedDistances, thisLowerBound, thisUpperBound);

	if (!node && thisLowerBound > 0)
	{	// no change - already completely solid
		return node;
	}

	if (!node || otherLowerBound > thisUpperBound)
	{	// this node is replaced with the other sdf
		// this could lead to inaccurate distance outside the volume, however these are usually not required
		if (node) deallocNode(node);
		return createNode(area, otherSDF, newSDF);
	}
	if (node)
	{	// need to recurse to node children
		vAssert(area.m_SizeExpo > 0)
			Area subAreas[8];
		area.getSubAreas(subAreas);
		for (int i = 0; i < 8; i++)
			node->m_Children[i] = merge(node->m_Children[i], subAreas[i], otherSDF, newSDF, otherSDFCache);
	}
	return node;
}

void OctreeSDF::interpolateLeaf(const Area& area, SignedDistanceGrid& grid)
{
	Area subAreas[8];
	area.getSubAreas(subAreas);

	Sample cornerSamples[8];
	for (int i = 0; i < 8; i++)
	{
		cornerSamples[i] = lookupSample(i, area, grid);
	}

	int expoMultiplier = (1 << (subAreas[0].m_SizeExpo));
	int expoMultiplier2 = (expoMultiplier << 1);

	Vector3i minPos = subAreas[0].m_MinPos;

	// first do the xy plane at z = 0
	Sample edgeMid15 = (cornerSamples[0] + cornerSamples[4]) * 0.5f;
	Sample edgeMid13 = (cornerSamples[0] + cornerSamples[2]) * 0.5f;
	Sample edgeMid57 = (cornerSamples[4] + cornerSamples[6]) * 0.5f;
	Sample edgeMid37 = (cornerSamples[2] + cornerSamples[6]) * 0.5f;
	Sample faceMid1 = (edgeMid15 + edgeMid37) * 0.5f;
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, 0, 0), edgeMid15));
	grid.insert(std::make_pair(minPos + Vector3i(0, expoMultiplier, 0), edgeMid13));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier, 0), faceMid1));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier2, 0), edgeMid57));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier2, expoMultiplier, 0), edgeMid37));

	// then the xy plane at z = 2
	minPos = minPos + Vector3i(0, 0, 2 * expoMultiplier);
	Sample edgeMid26 = (cornerSamples[1] + cornerSamples[5]) * 0.5f;
	Sample edgeMid24 = (cornerSamples[1] + cornerSamples[3]) * 0.5f;
	Sample edgeMid68 = (cornerSamples[5] + cornerSamples[7]) * 0.5f;
	Sample edgeMid48 = (cornerSamples[3] + cornerSamples[7]) * 0.5f;
	Sample faceMid2 = (edgeMid26 + edgeMid48) * 0.5f;
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, 0, 0), edgeMid26));
	grid.insert(std::make_pair(minPos + Vector3i(0, expoMultiplier, 0), edgeMid24));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier, 0), faceMid2));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier2, 0), edgeMid68));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier2, expoMultiplier, 0), edgeMid48));

	// 4 edges at z = 1
	minPos = subAreas[0].m_MinPos + Vector3i(0, 0, expoMultiplier);
	grid.insert(std::make_pair(minPos, (cornerSamples[0] + cornerSamples[1]) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(0, expoMultiplier2, 0), (cornerSamples[2] + cornerSamples[3]) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier2, 0, 0), (cornerSamples[4] + cornerSamples[5]) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier2, expoMultiplier2, 0), (cornerSamples[6] + cornerSamples[7]) * 0.5f));

	// 4 faces at z = 1
	grid.insert(std::make_pair(minPos + Vector3i(0, expoMultiplier, 0), (edgeMid13 + edgeMid24) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, 0, 0), (edgeMid15 + edgeMid26) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier2, 0), (edgeMid37 + edgeMid48) * 0.5f));
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier2, expoMultiplier, 0), (edgeMid57 + edgeMid68) * 0.5f));

	// and finally, the mid point
	grid.insert(std::make_pair(minPos + Vector3i(expoMultiplier, expoMultiplier, 0), (faceMid1 + faceMid2) * 0.5f));

	/*Area subArea = subAreas[0];
	Sample currentSample = cornerSamples[0];
	for (int x = 0; x < 3; x++)
	{
	for (int y = 0; y < 3; y++)
	{
	subArea.m_MinPos.y += expoMultiplier, 0;
	currentSample +=
	}
	}
	subArea.m_MinPos = subArea.m_MinPos + Vector3i(1, 0, 0) * expoMultiplier;
	for (int x = 0; x < 3; x++)

	// interpolate 3x3x3 signed distance subgrid
	Vector3i subGridVecs[27];
	Vector3i::grid3(subGridVecs);
	for (int i = 0; i < 27; i++)
	{
	float weights[3];
	for (int d = 0; d < 3; d++)
	weights[d] = subGridVecs[i][d] * 0.5f;
	Area subArea = subAreas[0];
	subArea.m_MinPos = subArea.m_MinPos + subGridVecs[i] * (1 << (subArea.m_SizeExpo));
	auto tryInsert = m_SDFValues.insert(std::make_pair(subArea.getCorner(0), 0.0f));
	if (tryInsert.second)
	tryInsert.first->second = MathMisc::trilinearInterpolation(cornerSamples, weights);
	}*/
}

std::shared_ptr<OctreeSDF> OctreeSDF::sampleSDF(SignedDistanceField3D* otherSDF, int maxDepth)
{
	return sampleSDF(otherSDF, otherSDF->getAABB(), maxDepth);
}

std::shared_ptr<OctreeSDF> OctreeSDF::sampleSDF(SignedDistanceField3D* otherSDF, AABB& aabb, int maxDepth)
{
	std::shared_ptr<OctreeSDF> octreeSDF = std::make_shared<OctreeSDF>();
	Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
	float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
	octreeSDF->m_CellSize = cubeSize / (1 << maxDepth);
	otherSDF->prepareSampling(aabb, octreeSDF->m_CellSize);
	octreeSDF->m_RootArea = Area(Vector3i(0, 0, 0), maxDepth, aabb.getMin(), cubeSize);
	octreeSDF->m_RootNode = octreeSDF->createNode(octreeSDF->m_RootArea, *otherSDF, octreeSDF->m_SDFValues);
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

vector<OctreeSDF::Cube> OctreeSDF::getCubesToMarch()
{
	vector<Cube> cubes;
	std::stack<Node*> nodes;
	getCubesToMarch(m_RootNode, m_RootArea, cubes);
	return cubes;
}

OctreeSDF::Sample OctreeSDF::getSample(const Ogre::Vector3& point) const
{
	return getSample(m_RootNode, m_RootArea, point);
}

// TODO!
bool OctreeSDF::intersectsSurface(const AABB& aabb) const
{
	if (m_TriangleCache.getBVH())
		return m_TriangleCache.getBVH()->intersectsAABB(aabb);
	return true;
}

void OctreeSDF::subtract(SignedDistanceField3D* otherSDF)
{
	OpInvertSDF invertedSDF(otherSDF);
	intersect(&invertedSDF);
}

void OctreeSDF::intersect(SignedDistanceField3D* otherSDF)
{
	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	SignedDistanceGrid newSDF;
	m_RootNode = intersect(m_RootNode, m_RootArea, *otherSDF, newSDF, SignedDistanceGrid());
	for (auto i = newSDF.begin(); i != newSDF.end(); i++)
		m_SDFValues[i->first] = i->second;
}

void OctreeSDF::intersectAlignedOctree(OctreeSDF* otherOctree)
{
	SignedDistanceGrid newSDF;
	m_RootNode = intersect(m_RootNode, otherOctree->m_RootNode, m_RootArea, otherOctree->m_SDFValues, newSDF);
	std::cout << newSDF.size() << " new values" << std::endl;
	for (auto i = newSDF.begin(); i != newSDF.end(); i++)
		m_SDFValues[i->first] = i->second;
}

void OctreeSDF::subtractAlignedOctree(OctreeSDF* otherOctree)
{
	auto clone = otherOctree->clone();
	clone->invert();
	intersectAlignedOctree(clone.get());
}

void OctreeSDF::resize(const AABB& aabb)
{
	while (!m_RootArea.toAABB().containsPoint(aabb.min))
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
	}
}

void OctreeSDF::merge(SignedDistanceField3D* otherSDF)
{
	// this is not an optimal resize policy but it should work
	// it is recommended to avoid resizes anyway
	resize(otherSDF->getAABB());

	otherSDF->prepareSampling(m_RootArea.toAABB(), m_CellSize);
	SignedDistanceGrid newSDF;
	m_RootNode = merge(m_RootNode, m_RootArea, *otherSDF, newSDF, SignedDistanceGrid());
	for (auto i = newSDF.begin(); i != newSDF.end(); i++)
		m_SDFValues[i->first] = i->second;
}

void OctreeSDF::invert()
{
	for (auto i = m_SDFValues.begin(); i != m_SDFValues.end(); i++)
		i->second.signedDistance *= -1.0f;
}

OctreeSDF::OctreeSDF(const OctreeSDF& other)
{
	m_RootNode = cloneNode(other.m_RootNode, other.m_RootArea, other.m_SDFValues, m_SDFValues);
	m_RootArea = other.m_RootArea;
	m_CellSize = other.m_CellSize;
	m_TriangleCache = other.m_TriangleCache;
}

std::shared_ptr<OctreeSDF> OctreeSDF::clone()
{
	return std::make_shared<OctreeSDF>(*this);
}

int OctreeSDF::countNodes()
{
	int counter = 0;
	countNodes(m_RootNode, m_RootArea, counter);
	return counter;
}

void OctreeSDF::generateTriangleCache()
{
	std::vector<Cube> cubes = getCubesToMarch();
	auto mesh = MarchingCubes::marchSDF(*this, getInverseCellSize());
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