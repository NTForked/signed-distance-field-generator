
#pragma once

#include "OgreMath/OgreVector3.h"
#include <vector>
#include "SignedDistanceField.h"
#include "AABB.h"
#include "FractalNoiseGenerator.h"

class FractalNoisePlaneSDF : public SignedDistanceField3D
{
protected:
	float** m_HeightMap;
	int m_HeightMapSize;
	float m_InverseCellSize;
	float m_Roughness;
	float m_ZRange;
	float m_Size;
	AABB m_AABB;
	AABB m_SurfaceAABB;

public:
	FractalNoisePlaneSDF(float size, float roughness, float zRange)
		: m_HeightMap(nullptr), m_Size(size), m_Roughness(roughness), m_ZRange(zRange)
	{
		float halfSize = m_Size * 0.5f;
		m_SurfaceAABB.min = Ogre::Vector3(-halfSize, -halfSize, -m_ZRange);
		m_SurfaceAABB.max = Ogre::Vector3(halfSize, halfSize, m_ZRange);

		m_AABB.min = Ogre::Vector3(-halfSize, -halfSize, -halfSize);
		m_AABB.max = Ogre::Vector3(halfSize, halfSize, m_ZRange);
	}
	~FractalNoisePlaneSDF()
	{
		if (m_HeightMap)
			FractalNoiseGenerator::freeHeightMap(m_HeightMapSize, m_HeightMap);
	}

	Sample getSample(const Ogre::Vector3& point) const override
	{
		Sample sample;
		sample.normal = Ogre::Vector3(0, 0, 1);

		// scale, move and project on xz plane
		Ogre::Vector3 scaled = (point - m_SurfaceAABB.min) * m_InverseCellSize;
		int x = (int)(scaled.x);
		int y = (int)(scaled.y);
		float surfaceZ = 0;
		if (x >= 0 && x < m_HeightMapSize && y >= 0 && y < m_HeightMapSize)
			surfaceZ = m_HeightMap[x][y];

		sample.signedDistance = surfaceZ - point.z;

		if (sample.signedDistance < 0) sample.normal *= -1;
		return sample;
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		return aabb.intersectsAABB(m_SurfaceAABB);
	}

	AABB getAABB() const override
	{
		return m_AABB;
	}

	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		if (m_HeightMap)
			FractalNoiseGenerator::freeHeightMap(m_HeightMapSize, m_HeightMap);

		m_InverseCellSize = 1.0f / cellSize;
		m_HeightMapSize = (int)std::ceil(m_Size * m_InverseCellSize);

		int pow2Size = 1;
		while (pow2Size < m_HeightMapSize)
			pow2Size = pow2Size << 1;
		m_HeightMapSize = pow2Size;
		m_HeightMap = FractalNoiseGenerator::allocHeightMap(m_HeightMapSize);	
		FractalNoiseGenerator::generate(m_HeightMapSize, m_Roughness, m_HeightMap);

		float noiseMax = 0.0f;
		for (int x = 0; x < m_HeightMapSize; x++)
			for (int y = 0; y < m_HeightMapSize; y++)
				if (m_HeightMap[x][y] > noiseMax) noiseMax = m_HeightMap[x][y];

		float multiplier = (float)m_ZRange / (noiseMax + 0.001f);
		for (int x = 0; x < m_HeightMapSize; x++)
			for (int y = 0; y < m_HeightMapSize; y++)
				m_HeightMap[x][y] *= multiplier;
	}
};