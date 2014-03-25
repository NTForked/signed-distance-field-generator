
#include "Vector3i.h"
#include "AABB.h"

#pragma once

struct Area
{
	Area() {}
	Area(const Vector3i& minPos, int sizeExpo, const Ogre::Vector3& minRealPos, float realSize)
		: m_MinPos(minPos), m_MaxPos(minPos + Vector3i(1 << sizeExpo, 1 << sizeExpo, 1 << sizeExpo)), m_SizeExpo(sizeExpo), m_MinRealPos(minRealPos), m_RealSize(realSize) {}
	Vector3i m_MinPos;
	Vector3i m_MaxPos;
	int m_SizeExpo;

	Ogre::Vector3 m_MinRealPos;
	float m_RealSize;

	__forceinline bool containsPoint(const Ogre::Vector3& point) const
	{
		return (point.x >= m_MinRealPos.x && point.x < (m_MinRealPos.x + m_RealSize)
			&& point.y >= m_MinRealPos.y && point.y < (m_MinRealPos.y + m_RealSize)
			&& point.z >= m_MinRealPos.z && point.z < (m_MinRealPos.z + m_RealSize));
	}

	__forceinline bool containsPoint(const Ogre::Vector3& point, float epsilon) const
	{
		return (point.x >= m_MinRealPos.x - epsilon && point.x < (m_MinRealPos.x + m_RealSize + epsilon)
			&& point.y >= m_MinRealPos.y - epsilon && point.y < (m_MinRealPos.y + m_RealSize + epsilon)
			&& point.z >= m_MinRealPos.z - epsilon && point.z < (m_MinRealPos.z + m_RealSize + epsilon));
	}

	__forceinline bool containsPoint(const Vector3i& point) const
	{
		return (point.x >= m_MinPos.x && point.x <= m_MaxPos.x
			&& point.y >= m_MinPos.y && point.y <= m_MaxPos.y
			&& point.z >= m_MinPos.z && point.z <= m_MaxPos.z);
	}

	__forceinline bool intersectsArea(const Area& otherArea) const
	{
		// perform separating axis test
		if (MathMisc::intervalDoesNotOverlap(m_MinPos.x, m_MaxPos.x, otherArea.m_MinPos.x, otherArea.m_MaxPos.x)) return false;
		if (MathMisc::intervalDoesNotOverlap(m_MinPos.y, m_MaxPos.y, otherArea.m_MinPos.y, otherArea.m_MaxPos.y)) return false;
		if (MathMisc::intervalDoesNotOverlap(m_MinPos.z, m_MaxPos.z, otherArea.m_MinPos.z, otherArea.m_MaxPos.z)) return false;
		return true;
	}

	// Computes a lower and upper bound inside the area given the 8 corner signed distances.
	void getLowerAndUpperBound(const float* signedDistances, float& lowerBound, float& upperBound) const
	{
		// Ogre::Vector3 mid = m_MinRealPos + Ogre::Vector3(m_RealSize, m_RealSize, m_RealSize);
		float minDist = std::numeric_limits<float>::max();
		float maxDist = std::numeric_limits<float>::lowest();
		for (int i = 0; i < 8; i++)
		{
			minDist = std::min(minDist, signedDistances[i]);
			maxDist = std::max(maxDist, signedDistances[i]);
		}
		float halfSize = m_RealSize *0.5f;
		// maxOffset = dist to mid
		float maxOffset = std::sqrtf(3 * halfSize*halfSize);
		lowerBound = minDist - maxOffset;
		upperBound = maxDist + maxOffset;
	}

	// Simply returns the minimum / maximum corner.
	void getLowerAndUpperBoundOptimistic(const float* signedDistances, float& lowerBound, float& upperBound) const
	{
		lowerBound = std::numeric_limits<float>::max();
		upperBound = std::numeric_limits<float>::lowest();
		for (int i = 0; i < 8; i++)
		{
			lowerBound = std::min(lowerBound, signedDistances[i]);
			upperBound = std::max(upperBound, signedDistances[i]);
		}
	}

	Vector3i getCorner(int corner) const
	{
		return m_MinPos + Vector3i((corner & 4) != 0, (corner & 2) != 0, corner & 1) * (1 << m_SizeExpo);
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
		return std::make_pair(m_MinPos + offset * (1 << m_SizeExpo), realPos);
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
			areas[i] = Area(m_MinPos + offset * (1 << (m_SizeExpo - 1)),
				m_SizeExpo - 1,
				m_MinRealPos + Ogre::Vector3(offset.x * halfSize, offset.y * halfSize, offset.z * halfSize), halfSize);
		}
	}
};