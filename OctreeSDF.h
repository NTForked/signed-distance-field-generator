
#pragma once

#include <unordered_map>
#include "SignedDistanceField.h"
#include "Vector3i.h"
#include "AABB.h"
#include "Prerequisites.h"

class OctreeSDF : public SignedDistanceField3D<MaterialID>
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

	std::unordered_map<Vector3i, float> m_SDFValues;
	Node* m_RootNode;

	 int m_MaxDepth;

	/// The octree covers an axis aligned cube.
	 Area m_RootArea;

	 Vector3i getGlobalPos(const Vector3i& pos, int depth) const
	 {
		 return pos * (1<<(m_MaxDepth - depth));
	 }

	template<class ImplicitSDF>
	float lookupOrComputeSignedDistance(int corner, const Area& area, const ImplicitSDF& implicitSDF)
	{
		auto vecs = area.getCornerVecs(corner);
		auto tryInsert = m_SDFValues.insert(std::make_pair(getGlobalPos(vecs.first, area.m_Depth), 0.0f));
		if (tryInsert.second)
		{
			tryInsert.first->second = implicitSDF.getSignedDistance(vecs.second);
		}
		return tryInsert.first->second;
	}

	float lookupSignedDistance(const Vector3i& cornerOffset, const Area& area) const
	{
		return m_SDFValues.find(getGlobalPos(area.m_MinPos + cornerOffset, area.m_Depth))->second;
		
	}

	template<class ImplicitSDF>
	Node* createNode(const Area& area, const ImplicitSDF& implicitSDF)
	{
		for (int i = 0; i < 8; i++)
		{
			lookupOrComputeSignedDistance(i, area, implicitSDF);
		}
		if (area.m_Depth >= m_MaxDepth) return nullptr;
		if (!implicitSDF.intersectsSurface(area.toAABB())) return nullptr;

		// create inner node
		Area subAreas[8];
		area.getSubAreas(subAreas);
		Node* node = new Node();
		for (int i = 0; i < 8; i++)
			node->m_Children[i] = createNode(subAreas[i], implicitSDF);
		return node;
	}

	void getCubesToMarch(Node* node, const Area& area, vector<Cube>& cubes)
	{
		if (area.m_Depth == m_MaxDepth)
		{
			Cube cube;
			cube.posMin = area.m_MinPos;
			for (int i = 0; i < 8; i++)
			{
				auto find = m_SDFValues.find(area.getCornerVecs(i).first);
				vAssert(find != m_SDFValues.end());
				cube.signedDistances[i] = find->second;
			}
			cubes.push_back(cube);
		}
		else if (node)
		{
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
				getCubesToMarch(node->m_Children[i], subAreas[i], cubes);
		}
	}
	float getSignedDistance(Node* node, const Area& area, const Ogre::Vector3& point)
	{
		if (!node)
		{
			float invNodeSize = 1.0f / area.m_RealSize;
			float xWeight = (point.x - area.m_MinPos.x) * invNodeSize;
			float yWeight = (point.y - area.m_MinPos.y) * invNodeSize;
			float zWeight = (point.z - area.m_MinPos.z) * invNodeSize;
			
			return lookupSignedDistance(Vector3i(0, 0, 0), area) * (1 - xWeight) * (1 - yWeight) * (1 - zWeight)
				+ lookupSignedDistance(Vector3i(0, 0, 1), area) * (1 - xWeight) * (1 - yWeight) * zWeight
				+ lookupSignedDistance(Vector3i(0, 1, 0), area) * (1 - xWeight) * yWeight * (1 - zWeight)
				+ lookupSignedDistance(Vector3i(0, 1, 1), area) * (1 - xWeight) * yWeight * zWeight
				+ lookupSignedDistance(Vector3i(1, 0, 0), area) * xWeight * (1 - yWeight) * (1 - zWeight)
				+ lookupSignedDistance(Vector3i(1, 0, 1), area) * xWeight * (1 - yWeight) * zWeight
				+ lookupSignedDistance(Vector3i(1, 1, 0), area) * xWeight * yWeight * (1 - zWeight)
				+ lookupSignedDistance(Vector3i(1, 1, 1), area) * xWeight * yWeight * zWeight;
		}
		else
		{
			Area subAreas[8];
			area.getSubAreas(subAreas);
			for (int i = 0; i < 8; i++)
			{
				if (subAreas[i].containsPoint(point))
				{
					getSignedDistance(node->m_Children[i], subAreas[i], point);
					break;
				}
			}
		}
	}
public:
	template<class ImplicitSDF>
	static std::shared_ptr<OctreeSDF> sampleSDF(const ImplicitSDF& implicitSDF, int maxDepth)
	{
		std::shared_ptr<OctreeSDF> octreeSDF = std::make_shared<OctreeSDF>();
		AABB aabb = implicitSDF.getAABB();
		Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
		float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
		octreeSDF->m_RootArea = Area(Vector3i(0, 0, 0), 0, aabb.getMin(), cubeSize);
		octreeSDF->m_MaxDepth = maxDepth;
		octreeSDF->m_RootNode = octreeSDF->createNode(octreeSDF->m_RootArea, implicitSDF);
		return octreeSDF;
	}

	float getInverseCellSize()
	{
		return (float)(1 << m_MaxDepth) / m_RootArea.m_RealSize;
	}

	vector<Cube> getCubesToMarch()
	{
		vector<Cube> cubes;
		std::stack<Node*> nodes;
		getCubesToMarch(m_RootNode, m_RootArea, cubes);
		return cubes;
	}

	float getSignedDistance(const Ogre::Vector3& point)
	{
		return getSignedDistance(m_RootNode, m_RootArea, point);
	}
};