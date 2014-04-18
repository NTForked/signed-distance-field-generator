
#pragma once

#include "OgreMath/OgreVector3.h"
#include <vector>
#include "SolidGeometry.h"
#include "AABB.h"
#include "FractalNoiseGenerator.h"
#include "Mesh.h"

class FractalNoisePlaneSDF : public SolidGeometry
{
protected:
	float** m_HeightMap;
	int m_HeightMapSize;
	float m_InverseCellSize;
	float m_CellSize;
	float m_Roughness;
	float m_ZRange;
	float m_Size;
	AABB m_AABB;
	AABB m_SurfaceAABB;
	BVHScene m_TriangleCache;

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
	}

	inline int clamp(int x, int minX, int maxX) const
	{
		return std::min(std::max(x, minX), maxX);
	}
	inline float lookupSafe(int x, int y, float** map, int mapSize) const
	{
		x = clamp(x, 0, mapSize-1);
		y = clamp(y, 0, mapSize-1);
		float surfaceZ = map[x][y];
		return surfaceZ;
	}

	virtual void getSample(const Ogre::Vector3& point, Sample& sample) const override
	{
		sample.normal = Ogre::Vector3(0, 0, 1);

		// scale, move and project on xz plane
		Ogre::Vector3 scaled = (point - m_SurfaceAABB.min) * m_InverseCellSize;
		int x = (int)(scaled.x + 0.5f);
		int y = (int)(scaled.y + 0.5f);
		float surfaceZ = lookupSafe(x, y, m_HeightMap, m_HeightMapSize);
		sample.closestSurfacePos = Ogre::Vector3(point.x, point.y, surfaceZ);
		sample.signedDistance = surfaceZ - point.z;
		if (sample.signedDistance < 0)
		{
			sample.normal *= -1;
		}
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
		// return !MathMisc::intervalDoesNotOverlap(aabb.min.z, aabb.max.z, -m_ZRange, m_ZRange);
		// return aabb.intersectsAABB(m_SurfaceAABB);
		// AABB epsilonAABB = aabb;
		// epsilonAABB.addEpsilon(0.01f);
		return m_TriangleCache.getBVH()->intersectsAABB(aabb);
	}

	AABB getAABB() const override
	{
		return m_AABB;
	}

	virtual void prepareSampling(const AABB& aabb, float cellSize) override
	{
		m_CellSize = cellSize;
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
		for (int x = 0; x < m_HeightMapSize; x++)
			for (int y = 0; y < m_HeightMapSize; y++)
				m_HeightMap[x][y] *= multiplier;

		generateTriangleCache();
	}

	void generateTriangleCache()
	{
		std::cout << "[FractalNoisePlaneSDF] Generating triangle cache..." << std::endl;
		std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
		for (int x = 0; x < m_HeightMapSize; x++)
		{
			for (int y = 0; y < m_HeightMapSize; y++)
			{
				Vertex v(Ogre::Vector3(x * m_CellSize, y * m_CellSize, m_HeightMap[x][y]));
				mesh->vertexBuffer.push_back(v);
			}
		}
		for (int x = 0; x < m_HeightMapSize - 1; x++)
		{
			for (int y = 0; y < m_HeightMapSize - 1; y++)
			{
				int index1 = x*m_HeightMapSize + y;
				int index2 = (x + 1)*m_HeightMapSize + y;
				int index3 = x*m_HeightMapSize + y + 1;
				int index4 = (x + 1)*m_HeightMapSize + y + 1;
				mesh->indexBuffer.push_back(index1);
				mesh->indexBuffer.push_back(index4);
				mesh->indexBuffer.push_back(index2);
				mesh->indexBuffer.push_back(index1);
				mesh->indexBuffer.push_back(index3);
				mesh->indexBuffer.push_back(index4);
			}
		}
		std::shared_ptr<TransformedMesh> transformedMesh = std::make_shared<TransformedMesh>(mesh);
		mesh->computeTriangleNormals();
		transformedMesh->setPosition(Ogre::Vector3(m_SurfaceAABB.getMin().x, m_SurfaceAABB.getMin().y, 0));
		transformedMesh->computeCache();
		// ExportOBJ::writeMesh("FractalNoisePlaneMesh", transformedMesh->vertexBufferVS, mesh->indexBuffer);
		m_TriangleCache.clearMeshes();
		m_TriangleCache.addMesh(transformedMesh);
		m_TriangleCache.generateBVH<AABB>();
		m_TriangleCache.addMesh(transformedMesh);
		std::cout << "[FractalNoisePlaneSDF] Finished!" << std::endl;
	}
};
