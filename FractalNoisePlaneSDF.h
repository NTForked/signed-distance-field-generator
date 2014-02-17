
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
	std::vector<float**> m_MinMipMaps;
	std::vector<float**> m_MaxMipMaps;
	int m_HeightMapSize;
	float m_InverseCellSize;
	float m_Roughness;
	float m_ZRange;
	float m_Size;
	AABB m_AABB;
	AABB m_SurfaceAABB;

public:
	FractalNoisePlaneSDF(float size, float roughness, float zRange)
		: m_HeightMap(nullptr), m_HeightMapSize(0), m_Size(size), m_Roughness(roughness), m_ZRange(zRange)
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

		for (int level = 0; level < (int)m_MinMipMaps.size(); level++)
		{
			int mapSize = 1 << getMipLevelSizeExpo(level);
			for (int x = 0; x < mapSize; x++)
			{
				delete[] m_MinMipMaps[level][x];
				delete[] m_MaxMipMaps[level][x];
			}
			delete[] m_MinMipMaps[level];
			delete[] m_MaxMipMaps[level];
		}
	}

	inline float lookupSafe(int x, int y, float** map, int mapSize) const
	{
		float surfaceZ = 0;
		if (x >= 0 && x < mapSize && y >= 0 && y < mapSize)
			surfaceZ = map[x][y];
		return surfaceZ;
	}

	Sample getSample(const Ogre::Vector3& point) const override
	{
		Sample sample;
		sample.normal = Ogre::Vector3(0, 0, 1);

		// scale, move and project on xz plane
		Ogre::Vector3 scaled = (point - m_SurfaceAABB.min) * m_InverseCellSize;
		int x = (int)(scaled.x);
		int y = (int)(scaled.y);
		float surfaceZ = lookupSafe(x, y, m_HeightMap, m_HeightMapSize);
		sample.signedDistance = surfaceZ - point.z;
		if (sample.signedDistance < 0) sample.normal *= -1;
		return sample;
	}

	static int roundToNextPowerOfTwo(int n, int& expo)
	{
		expo = 0;
		int pow2N = 1;
		while (pow2N < n)
		{
			pow2N <<= 1;
			expo++;
		}
		return pow2N;
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		// return aabb.intersectsAABB(m_SurfaceAABB);
		// if (!aabb.intersectsAABB(m_SurfaceAABB)) return false;
		Ogre::Vector3 scaledMin = (aabb.min - m_SurfaceAABB.min) * m_InverseCellSize;
		int totalMin = (int)std::min(scaledMin.x, scaledMin.y);
		Ogre::Vector3 scaledMax = (aabb.max - m_SurfaceAABB.min) * m_InverseCellSize;
		int totalMax = (int)std::ceil(std::max(scaledMax.x, scaledMax.y));
		int rangeExpo;
		int range = roundToNextPowerOfTwo(totalMax - totalMin, rangeExpo);		
		if (rangeExpo > (int)m_MinMipMaps.size())
			rangeExpo = (int)m_MinMipMaps.size();
		int x = (int)(scaledMin.x);
		int y = (int)(scaledMin.y);
		x >>= rangeExpo;
		y >>= rangeExpo;
		int mipLevel = getMipLevelFromPixelRange(rangeExpo);
		float minVal = lookupSafe(x, y, m_MinMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.max.z;
		float maxVal = lookupSafe(x, y, m_MaxMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.min.z;
		if (minVal <= 0 && maxVal > 0) return true;
		totalMin = std::min(x << rangeExpo, y << rangeExpo);
		if (totalMax - totalMin > range)
		{
			// need check 3 more for full aabb coverage
			minVal = lookupSafe(x+1, y, m_MinMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.max.z;
			maxVal = lookupSafe(x+1, y, m_MaxMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.min.z;
			if (minVal <= 0 && maxVal > 0) return true;
			minVal = lookupSafe(x, y+1, m_MinMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.max.z;
			maxVal = lookupSafe(x, y+1, m_MaxMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.min.z;
			if (minVal <= 0 && maxVal > 0) return true;
			minVal = lookupSafe(x+1, y+1, m_MinMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.max.z;
			maxVal = lookupSafe(x+1, y+1, m_MaxMipMaps[mipLevel], 1 << getMipLevelSizeExpo(mipLevel)) - aabb.min.z;
			if (minVal <= 0 && maxVal > 0) return true;
		}
		return false;
	}

	AABB getAABB() const override
	{
		return m_AABB;
	}

	void generateMipMap(int expo, float** inputValues, std::function<float(float, float)> combinerFn, float** outputMipMap)
	{
		int mapSize = 1 << expo;
		for (int x = 0; x < mapSize; x++)
		{
			for (int y = 0; y < mapSize; y++)
			{
				float parentMin = combinerFn(inputValues[2 * x][2 * y], inputValues[2 * x][2 * y + 1]);
				parentMin = combinerFn(parentMin, inputValues[2 * x + 1][2 * y]);
				parentMin = combinerFn(parentMin, inputValues[2 * x + 1][2 * y + 1]);
				outputMipMap[x][y] = parentMin;
			}
		}
	}

	int getMipLevelFromPixelRange(int pixelRangeExpo) const
	{
		return pixelRangeExpo - 1;
	}
	int getMipLevelPixelRangeExpo(int mipLevel) const
	{
		return mipLevel + 1;
	}
	int getMipLevelSizeExpo(int mipLevel) const
	{
		return (int)m_MinMipMaps.size() - mipLevel - 1;
	}

	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		m_InverseCellSize = 1.0f / cellSize;
		int pow2Expo = 0;
		int newMapSize = roundToNextPowerOfTwo((int)std::ceil(m_Size * m_InverseCellSize), pow2Expo);
		if (newMapSize == m_HeightMapSize) return;
		std::cout << "prepareSampling " << m_HeightMapSize << " != " << newMapSize << std::endl;

		if (m_HeightMap)
			FractalNoiseGenerator::freeHeightMap(m_HeightMapSize, m_HeightMap);

		m_HeightMapSize = newMapSize;

		m_HeightMap = FractalNoiseGenerator::allocHeightMap(m_HeightMapSize);
		FractalNoiseGenerator::generate(m_HeightMapSize, m_Roughness, m_HeightMap);

		float noiseMax = 0.0f;
		for (int x = 0; x < m_HeightMapSize; x++)
			for (int y = 0; y < m_HeightMapSize; y++)
				if (std::fabsf(m_HeightMap[x][y]) > noiseMax) noiseMax = std::fabsf(m_HeightMap[x][y]);

		float multiplier = (float)m_ZRange / (noiseMax + 0.001f);
		float max = 0;
		for (int x = 0; x < m_HeightMapSize; x++)
			for (int y = 0; y < m_HeightMapSize; y++)
				m_HeightMap[x][y] *= multiplier;
		
		// allocate mip maps
		if (pow2Expo < 2) return;
		m_MinMipMaps.resize(pow2Expo);
		m_MaxMipMaps.resize(pow2Expo);
		for (int level = 0; level < (int)m_MinMipMaps.size(); level++)
		{
			int mapSize = 1 << getMipLevelSizeExpo(level);
			m_MinMipMaps[level] = new float*[mapSize];
			m_MaxMipMaps[level] = new float*[mapSize];
			for (int x = 0; x < mapSize; x++)
			{
				m_MinMipMaps[level][x] = new float[mapSize];
				m_MaxMipMaps[level][x] = new float[mapSize];
			}	
		}
		generateMipMap(getMipLevelSizeExpo(0), m_HeightMap, fminf, m_MinMipMaps[0]);
		generateMipMap(getMipLevelSizeExpo(0), m_HeightMap, fmaxf, m_MaxMipMaps[0]);
		for (int level = 1; level < (int)m_MinMipMaps.size(); level++)
		{
			generateMipMap(getMipLevelSizeExpo(level), m_MinMipMaps[level - 1], fminf, m_MinMipMaps[level]);
			generateMipMap(getMipLevelSizeExpo(level), m_MaxMipMaps[level - 1], fmaxf, m_MaxMipMaps[level]);
		}

	}
};