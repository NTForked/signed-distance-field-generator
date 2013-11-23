
#pragma once

#include <unordered_map>
#include "SignedDistanceField.h"
#include "Vector3i.h"
#include "AABB.h"
#include "Prerequisites.h"
#include "OpInvertSDF.h"

/*
Samples a signed distance field in an adaptive way.
For each node (includes inner nodes and leaves) signed distances are stores for the 8 corners. This allows to interpolate signed distances in the node cell using trilinear interpolation.
The actual signed distances are stored in a spatial hashmap because octree nodes share corners with other nodes.
*/
class OctreeSDF : public SampledSignedDistanceField3D
{
protected:
	struct Area
	{
		Area() {}
		Area(const Vector3i& minPos, int depth, const Ogre::Vector3& minRealPos, float realSize)
			: m_MinPos(minPos), m_Depth(depth), m_MinRealPos(minRealPos), m_RealSize(realSize) {}
		Vector3i m_MinPos;
		int m_Depth;

		Ogre::Vector3 m_MinRealPos;
		float m_RealSize;

		__forceinline bool containsPoint(const Ogre::Vector3& point)
		{
			return (point.x >= m_MinRealPos.x && point.x < (m_MinRealPos.x + m_RealSize)
				&& point.y >= m_MinRealPos.y && point.y < (m_MinRealPos.y + m_RealSize)
				&& point.z >= m_MinRealPos.z && point.z < (m_MinRealPos.z + m_RealSize));
		}

		// Computes a lower and upper bound inside the area given the 8 corner signed distances.
		void getLowerAndUpperBound(float* signedDistances, float& lowerBound, float& upperBound) const
		{
			float minDist = std::numeric_limits<float>::max();
			float maxDist = std::numeric_limits<float>::min();
			for (int i = 0; i < 8; i++)
			{
				minDist = std::min(minDist, signedDistances[i]);
				maxDist = std::max(maxDist, signedDistances[i]);
			}
			lowerBound = minDist - m_RealSize * 0.5f;
			upperBound = maxDist + m_RealSize * 0.5f;
		}

		Vector3i getCorner(int corner) const
		{
			return m_MinPos + Vector3i((corner & 4) != 0, (corner & 2) != 0, corner & 1);
		}

		/// Retrieves the i-th corner of the cube with 0 = min and 7 = max
		std::pair<Vector3i, Ogre::Vector3> getCornerVecs(int corner) const
		{
			Vector3i offset((corner & 4) != 0, (corner & 2) != 0, corner & 1);
			Ogre::Vector3 realPos = m_MinRealPos;
			for (int i = 0; i < 3; i++)
			{
				realPos[i] += m_RealSize * (float)offset[i];
			}
			return std::make_pair(m_MinPos + offset, realPos);
		}
		AABB toAABB() const
		{
			return AABB(m_MinRealPos, m_MinRealPos + Ogre::Vector3(m_RealSize, m_RealSize, m_RealSize));
		}

		void getSubAreas(Area* areas) const
		{
			float halfSize = m_RealSize * 0.5f;
			for (int i = 0; i < 8; i++)
			{
				Vector3i offset((i & 4) != 0, (i & 2) != 0, i & 1);
				areas[i] = Area(m_MinPos * 2 + offset, m_Depth + 1,
				m_MinRealPos + Ogre::Vector3(offset.x * halfSize, offset.y * halfSize, offset.z * halfSize), halfSize);
			}
		}
	};
	struct Node
	{
		Node* m_Children[8];
		Node()
		{
			for (int i = 0; i < 8; i++)
				m_Children[i] = nullptr;
		}
	};

	typedef std::unordered_map<Vector3i, Sample> SignedDistanceGrid;

	SignedDistanceGrid m_SDFValues;
	Node* m_RootNode;

	float m_CellSize;

	 int m_MaxDepth;

	/// The octree covers an axis aligned cube.
	 Area m_RootArea;

	 Vector3i getGlobalPos(const Vector3i& pos, int depth) const
	 {
		 return pos * (1<<(m_MaxDepth - depth));
	 }

	Sample lookupOrComputeSample(int corner, const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
	{
		auto vecs = area.getCornerVecs(corner);
		auto tryInsert = sdfValues.insert(std::make_pair(getGlobalPos(vecs.first, area.m_Depth), Sample(0.0f)));
		if (tryInsert.second)
		{
			tryInsert.first->second = implicitSDF.getSample(vecs.second);
		}
		return tryInsert.first->second;
	}

	void setSample(int corner, const Area& area, const Sample& sample)
	{
		m_SDFValues[getGlobalPos(area.getCorner(corner), area.m_Depth)] = sample;
	}

	Sample lookupSample(int corner, const Area& area) const
	{
		auto find = m_SDFValues.find(getGlobalPos(area.getCorner(corner), area.m_Depth));
		vAssert(find != m_SDFValues.end())
		return find->second;
	}

	/// Top down octree constructor given a SDF.
	Node* createNode(const Area& area, const SignedDistanceField3D& implicitSDF, SignedDistanceGrid& sdfValues)
	{
		if (area.m_Depth >= m_MaxDepth ||
			!implicitSDF.intersectsSurface(area.toAABB()))
		{
			float signedDistances[8];
			for (int i = 0; i < 8; i++)
			{
				signedDistances[i] = lookupOrComputeSample(i, area, implicitSDF, sdfValues).signedDistance;
			}
			if (area.m_Depth >= m_MaxDepth || allSignsAreEqual(signedDistances))
				return nullptr;	// leaf
		}

		// create inner node
		Area subAreas[8];
		area.getSubAreas(subAreas);
		Node* node = new Node();
		for (int i = 0; i < 8; i++)
			node->m_Children[i] = createNode(subAreas[i], implicitSDF, sdfValues);
		return node;
	}

	// Computes a lower and upper bound inside the area given the 8 corner signed distances.
	void getLowerAndUpperBound(Node* node, const Area& area, float* signedDistances, float& lowerBound, float& upperBound) const
	{
		if (node) area.getLowerAndUpperBound(signedDistances, lowerBound, upperBound);
		else
		{	// if it's a leaf we can do even better and just return min and max of the corner values.
			float thisMin = std::numeric_limits<float>::max();
			float thisMax = std::numeric_limits<float>::min();
			for (int i = 0; i < 8; i++)
			{
				thisMin = std::min(thisMin, signedDistances[i]);
				thisMax = std::max(thisMax, signedDistances[i]);
			}
		}
	}

	void getCubesToMarch(Node* node, const Area& area, vector<Cube>& cubes)
	{
		if (node)
		{
			vAssert(area.m_Depth < m_MaxDepth)
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
			if (area.m_Depth == m_MaxDepth)
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
	Sample getSample(Node* node, const Area& area, const Ogre::Vector3& point) const
	{
		if (!node)
		{
			float invNodeSize = 1.0f / area.m_RealSize;
			float weights[3];
			weights[0] = (point.x - area.m_MinPos.x) * invNodeSize;
			weights[1] = (point.y - area.m_MinPos.y) * invNodeSize;
			weights[2] = (point.z - area.m_MinPos.z) * invNodeSize;
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
				if (subAreas[i].containsPoint(point))
				{
					return getSample(node->m_Children[i], subAreas[i], point);
					break;
				}
			}
		}
		return 0.0f;		// should never occur
	}

	/// Intersects an sdf with the node and returns the new node. The new sdf values are written to newSDF.
	Node* intersect(Node* node, const Area& area, SignedDistanceField3D& otherSDF, SignedDistanceGrid& newSDF, SignedDistanceGrid& otherSDFCache)
	{
		// if otherSDF does not overlap with the node AABB we can stop here
		if (!area.toAABB().intersectsAABB(otherSDF.getAABB())) return node;

		// compute signed distances for this node and the area of the other sdf
		float otherSignedDistances[8];
		float thisSignedDistances[8];
		for (int i = 0; i < 8; i++)
		{
			auto vecs = area.getCornerVecs(i);
			Sample otherSample = lookupOrComputeSample(i, area, otherSDF, otherSDFCache);
			Vector3i globalPos = getGlobalPos(vecs.first, area.m_Depth);
			auto find = m_SDFValues.find(globalPos);
			vAssert(find != m_SDFValues.end())
			otherSignedDistances[i] = otherSample.signedDistance;
			thisSignedDistances[i] = find->second.signedDistance;

			if (otherSignedDistances[i] < thisSignedDistances[i])
				newSDF[globalPos] = otherSample;
			else newSDF[globalPos] = find->second;
		}

		// compute a lower and upper bound for this node for this and the other sdf
		float otherLowerBound, otherUpperBound;
		area.getLowerAndUpperBound(otherSignedDistances, otherLowerBound, otherUpperBound);

		float thisLowerBound, thisUpperBound;
		getLowerAndUpperBound(node, area, thisSignedDistances, thisLowerBound, thisUpperBound);

		if (otherUpperBound < thisLowerBound)
		{	// this node is replaced with the inverse of the other sdf
			if (node) delete node;
			node = createNode(area, otherSDF, newSDF);
		}
		else if (otherLowerBound > thisUpperBound)
		{	// no change for this node
			return node;
		}

		if (node)
		{	// need to recurse to node children
			vAssert(area.m_Depth < m_MaxDepth)
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
				node->m_Children[i] = intersect(node->m_Children[i], subAreas[i], otherSDF, newSDF, otherSDFCache);
		}
		else
		{	// it's a leaf in the octree
			if (area.m_Depth < m_MaxDepth && otherSDF.intersectsSurface(area.toAABB()))
			{
				// need to subdivide this node
				node = new Node();
				interpolateLeaf(area);
				Area subAreas[8];
				area.getSubAreas(subAreas);
				for (int i = 0; i < 8; i++)
					node->m_Children[i] = intersect(nullptr, subAreas[i], otherSDF, newSDF, otherSDFCache);
			}
		}
		return node;
	}

	/// Interpolates signed distance for the 3x3x3 subgrid of a leaf.
	void interpolateLeaf(const Area& area)
	{
		Area subAreas[8];
		area.getSubAreas(subAreas);

		Sample cornerSamples[8];
		for (int i = 0; i < 8; i++)
		{
			cornerSamples[i] = lookupSample(i, area);
		}

		// interpolate 3x3x3 signed distance subgrid
		Vector3i subGridVecs[27];
		Vector3i::grid3(subGridVecs);
		for (int i = 0; i < 27; i++)
		{
			float weights[3];
			for (int d = 0; d < 3; d++)
				weights[d] = subGridVecs[i][d] * 0.5f;
			Area subArea = subAreas[0];
			subArea.m_MinPos = subArea.m_MinPos + subGridVecs[i];
			setSample(0, subArea, MathMisc::trilinearInterpolation(cornerSamples, weights));
		}
	}
public:
	static std::shared_ptr<OctreeSDF> sampleSDF(SignedDistanceField3D& otherSDF, int maxDepth)
	{
		std::shared_ptr<OctreeSDF> octreeSDF = std::make_shared<OctreeSDF>();
		AABB aabb = otherSDF.getAABB();
		Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
		float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
		octreeSDF->m_CellSize = cubeSize / (1 << maxDepth);
		otherSDF.prepareSampling(aabb, octreeSDF->m_CellSize);
		octreeSDF->m_RootArea = Area(Vector3i(0, 0, 0), 0, aabb.getMin(), cubeSize);
		octreeSDF->m_MaxDepth = maxDepth;
		octreeSDF->m_RootNode = octreeSDF->createNode(octreeSDF->m_RootArea, otherSDF, octreeSDF->m_SDFValues);
		return octreeSDF;
	}

	float getInverseCellSize()
	{
		return (float)(1 << m_MaxDepth) / m_RootArea.m_RealSize;
	}

	AABB getAABB() const override { return m_RootArea.toAABB(); }

	vector<Cube> getCubesToMarch()
	{
		vector<Cube> cubes;
		std::stack<Node*> nodes;
		getCubesToMarch(m_RootNode, m_RootArea, cubes);
		return cubes;
	}

	Sample getSample(const Ogre::Vector3& point) const override
	{
		return getSample(m_RootNode, m_RootArea, point);
	}

	// TODO!
	bool intersectsSurface(const AABB &) const override
	{
		return true;
	}

	void subtract(SignedDistanceField3D& otherSDF)
	{
		otherSDF.prepareSampling(m_RootArea.toAABB(), m_CellSize);
		SignedDistanceGrid newSDF;
		m_RootNode = intersect(m_RootNode, m_RootArea, OpInvertSDF(&otherSDF), newSDF, SignedDistanceGrid());
		for (auto i = newSDF.begin(); i != newSDF.end(); i++)
			m_SDFValues[i->first] = i->second;
	}

	void intersect(SignedDistanceField3D& otherSDF)
	{
		otherSDF.prepareSampling(m_RootArea.toAABB(), m_CellSize);
		SignedDistanceGrid newSDF;
		m_RootNode = intersect(m_RootNode, m_RootArea, otherSDF, newSDF, SignedDistanceGrid());
		for (auto i = newSDF.begin(); i != newSDF.end(); i++)
			m_SDFValues[i->first] = i->second;
	}
};