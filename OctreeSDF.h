
#pragma once

#include <unordered_map>
#include "SignedDistanceField.h"
#include "Vector3i.h"
#include "AABB.h"
#include "Prerequisites.h"

class OctreeSDF : public SampledSignedDistanceField3D<MaterialID>
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

	float m_CellSize;

	 int m_MaxDepth;

	/// The octree covers an axis aligned cube.
	 Area m_RootArea;

	 Vector3i getGlobalPos(const Vector3i& pos, int depth) const
	 {
		 return pos * (1<<(m_MaxDepth - depth));
	 }

	 bool allSignsAreEqual(float* signedDistances)
	 {
		 bool positive = (signedDistances[0] >= 0.0f);
		 for (int i = 1; i < 8; i++)
		 {
			 if ((signedDistances[i] >= 0.0f) != positive)
				 return false;
		 }
		 return true;
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

	//template<class ImplicitSDF>
	Node* createNode(const Area& area, const SignedDistanceField3D& implicitSDF)
	{
		float signedDistances[8];
		for (int i = 0; i < 8; i++)
		{
			signedDistances[i] = lookupOrComputeSignedDistance(i, area, implicitSDF);
		}
		if (area.m_Depth >= m_MaxDepth) return nullptr;
		if (!implicitSDF.intersectsSurface(area.toAABB()) && allSignsAreEqual(signedDistances)) return nullptr;

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
		else
		{
			/*float signedDistances[8];
			for (int i = 0; i < 8; i++)
			{
				auto find = m_SDFValues.find(getGlobalPos(area.getCornerVecs(i).first, area.m_Depth));
				vAssert(find != m_SDFValues.end());
				signedDistances[i] = find->second;
			}
			if (!allSignsAreEqual(signedDistances))
			{	// we need to interpolate the signed distances at level 0
				std::cout << "Subdividing node with depth " << area.m_Depth << std::endl;
				Vector3i globalMin = getGlobalPos(area.m_MinPos, area.m_Depth);
				Vector3i globalMax = getGlobalPos(area.m_MinPos + Vector3i(1, 1, 1), area.m_Depth);
				int cubeSize = 1<<area.m_Depth + 1;
				float* signedDistanceSubgrid = new float[cubeSize*cubeSize*cubeSize];
				
				float xWeight = 0;
				float yWeight = 0;
				float zWeight = 0;
				float stepSize = 1.0f / area.m_RealSize;
				for (int x = 0; x < cubeSize; x++)
				{
					for (int y = 0; y < cubeSize; y++)
					{
						for (int z = 0; z < cubeSize; z++)
						{
							signedDistanceSubgrid[x*cubeSize*cubeSize + y*cubeSize + z] = 
								signedDistances[0] * (1 - xWeight) * (1 - yWeight) * (1 - zWeight)
								+ signedDistances[1] * (1 - xWeight) * (1 - yWeight) * zWeight
								+ signedDistances[2] * (1 - xWeight) * yWeight * (1 - zWeight)
								+ signedDistances[3] * (1 - xWeight) * yWeight * zWeight
								+ signedDistances[4] * xWeight * (1 - yWeight) * (1 - zWeight)
								+ signedDistances[5] * xWeight * (1 - yWeight) * zWeight
								+ signedDistances[6] * xWeight * yWeight * (1 - zWeight)
								+ signedDistances[7] * xWeight * yWeight * zWeight;
							zWeight += stepSize;
						}
						yWeight += stepSize;
					}
					xWeight += stepSize;
				}
				Cube cube;
				for (int x = 0; x < cubeSize-1; x++)
				{
					for (int y = 0; y < cubeSize - 1; y++)
					{
						for (int z = 0; z < cubeSize - 1; z++)
						{
							cube.posMin = globalMin + Vector3i(x, y, z);
							cube.signedDistances[0] = signedDistanceSubgrid[x*cubeSize*cubeSize + y*cubeSize + z];
							cube.signedDistances[1] = signedDistanceSubgrid[x*cubeSize*cubeSize + y*cubeSize + z + 1];
							cube.signedDistances[2] = signedDistanceSubgrid[x*cubeSize*cubeSize + (y+1)*cubeSize + z];
							cube.signedDistances[3] = signedDistanceSubgrid[x*cubeSize*cubeSize + (y + 1)*cubeSize + z + 1];
							cube.signedDistances[4] = signedDistanceSubgrid[(x+1)*cubeSize*cubeSize + y*cubeSize + z];
							cube.signedDistances[5] = signedDistanceSubgrid[(x + 1)*cubeSize*cubeSize + y*cubeSize + z + 1];
							cube.signedDistances[6] = signedDistanceSubgrid[(x + 1)*cubeSize*cubeSize + (y + 1)*cubeSize + z];
							cube.signedDistances[7] = signedDistanceSubgrid[(x + 1)*cubeSize*cubeSize + (y + 1)*cubeSize + z + 1];
							cubes.push_back(cube);
						}
					}
				}
				delete signedDistanceSubgrid;
			}*/
		}
	}
	float getSignedDistance(Node* node, const Area& area, const Ogre::Vector3& point) const
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
					return getSignedDistance(node->m_Children[i], subAreas[i], point);
					break;
				}
			}
		}
		return 0.0f;		// should never occur
	}
public:
	//template<class ImplicitSDF>
	static std::shared_ptr<OctreeSDF> sampleSDF(SignedDistanceField3D& implicitSDF, int maxDepth)
	{
		std::shared_ptr<OctreeSDF> octreeSDF = std::make_shared<OctreeSDF>();
		AABB aabb = implicitSDF.getAABB();
		Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
		float cubeSize = std::max(std::max(aabbSize.x, aabbSize.y), aabbSize.z);
		octreeSDF->m_CellSize = cubeSize / (1 << maxDepth);
		implicitSDF.prepareSampling(octreeSDF->m_CellSize);
		octreeSDF->m_RootArea = Area(Vector3i(0, 0, 0), 0, aabb.getMin(), cubeSize);
		octreeSDF->m_MaxDepth = maxDepth;
		octreeSDF->m_RootNode = octreeSDF->createNode(octreeSDF->m_RootArea, implicitSDF);
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

	float getSignedDistance(const Ogre::Vector3& point) const override
	{
		return getSignedDistance(m_RootNode, m_RootArea, point);
	}

	// TODO!
	bool SignedDistanceField3D::intersectsSurface(const AABB &) const override
	{
		return true;
	}
};