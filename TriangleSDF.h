
#pragma once

#include <vector>
#include "OgreMath/OgreVector3.h"
#include "OBJReader.h"
#include "Mesh.h"
#include "BVHScene.h"
#include "Surfaces.h"
#include "AABB.h"
#include "Profiler.h"
#include "SignedDistanceField.h"
#include <math.h>

using std::vector;
using Ogre::Vector3;

class TriangleMeshSDF : public SignedDistanceField3D
{
protected:
	BVHScene m_RootNode;
	AABB m_AABB;
public:
	virtual ~TriangleMeshSDF()
	{
	}
	TriangleMeshSDF(std::shared_ptr<TransformedMesh> mesh)
	{
		mesh->computeCache();
		m_RootNode.addMesh(mesh);
		Profiler::Timestamp timeStamp = Profiler::timestamp();
		m_RootNode.generateBVH<AABB>();
		Profiler::printJobDuration("BVH creation", timeStamp);
		std::cout << "Tree height max: " << m_RootNode.getBVH()->getHeight() << std::endl;
		std::cout << "Tree height avg: " << m_RootNode.getBVH()->getHeightAvg() << std::endl;

		vector<Vector3> positions;
		for (auto i = mesh->vertexBufferVS.begin(); i != mesh->vertexBufferVS.end(); ++i)
			positions.push_back(i->position);
		m_AABB = AABB(positions);
		m_AABB.max *= 1.05f;
		m_AABB.min *= 1.05f;
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		return m_RootNode.getBVH()->intersectsAABB(aabb);
	}

	AABB getAABB() const override { return m_AABB; }
};

/// Uses the angle weighted pseudo normal for the sign computation.
class TriangleMeshSDF_AWP : public TriangleMeshSDF
{
public:
	TriangleMeshSDF_AWP(std::shared_ptr<TransformedMesh> mesh) :
		TriangleMeshSDF(mesh) {}
	Sample getSample(const Ogre::Vector3& point) const override
	{
		BVH<Surface>::ClosestLeafResult result;
		const Surface* tri = m_RootNode.getBVH()->getClosestLeaf(point, result);
		// test sign
		Vector3 rayDir = result.closestPoint - point;
		if (rayDir.dotProduct(result.normal) < 0.0f) result.closestDistance *= -1;
		// std::cout << result.closestDistance << std::endl;
		return Sample(result.closestDistance);
	};
};

class RaycastCache
{
protected:
	struct RayHit
	{
		float t;
		bool frontFace;
	};
	std::vector<RayHit>** m_RayResults;

	int m_Width;
	int m_Height;

	float m_CellSize;
	float m_InverseCellSize;

	int m_ImagePlaneNormalAxis;
	int m_ImagePlaneAxis1;
	int m_ImagePlaneAxis2;

	Ogre::Vector3 m_ImagePlaneMin;
	Ogre::Vector3 m_PlaneNormal;
	Ogre::Vector3 m_PlaneStepVec1;
	Ogre::Vector3 m_PlaneStepVec2;

	static bool compareIntersections(const std::pair<const Surface*, Ray::Intersection> &int1, const std::pair<const Surface*, Ray::Intersection> &int2)
	{
		return int1.second.t < int2.second.t;
	}

public:
	RaycastCache(const BVH<Surface>* bvh,
		float cellSize,
		float width,
		float height,
		const Ogre::Vector3& imagePlaneMin,
		int imagePlaneNormalAxis)
	{
		m_CellSize = cellSize;
		m_InverseCellSize = 1.0f / cellSize;
		m_Width = (int)std::ceil(width * m_InverseCellSize);
		m_Height = (int)std::ceil(height * m_InverseCellSize);

		m_ImagePlaneMin = imagePlaneMin;
		m_ImagePlaneNormalAxis = imagePlaneNormalAxis;
		m_ImagePlaneAxis1 = (m_ImagePlaneNormalAxis + 1) % 3;
		m_ImagePlaneAxis2 = (m_ImagePlaneNormalAxis + 2) % 3;

		m_PlaneStepVec1 = Ogre::Vector3(0, 0, 0);
		m_PlaneStepVec2 = Ogre::Vector3(0, 0, 0);
		m_PlaneNormal = Ogre::Vector3(0, 0, 0);
		m_PlaneNormal[m_ImagePlaneNormalAxis] = 1;
		m_PlaneStepVec1[m_ImagePlaneAxis1] = m_CellSize;
		m_PlaneStepVec2[m_ImagePlaneAxis2] = m_CellSize;

		// Add a constant offset to the ray origins to avoid rays passing exactly through vertices.
		Ogre::Vector3 constantOffset(Ogre::Math::PI * 0.00001f, Ogre::Math::PI * 0.00001f, Ogre::Math::PI * 0.00001f);

		m_RayResults = new std::vector<RayHit>*[m_Width];
		for (int x = 0; x < m_Width; x++)
		{
			m_RayResults[x] = new std::vector<RayHit>[m_Height];
			for (int y = 0; y < m_Height; y++)
			{
				Ogre::Vector3 rayOrigin = m_ImagePlaneMin + (float)x * m_PlaneStepVec1 + (float)y * m_PlaneStepVec2;
				Ray ray(rayOrigin + constantOffset, m_PlaneNormal);
				std::vector<std::pair<const Surface*, Ray::Intersection>> intersections;
				bvh->rayIntersectAll(ray, intersections);
				std::sort(intersections.begin(), intersections.end(), compareIntersections);
				// if (intersections.size() % 2) std::cout << "muh" << std::endl;
				float lastT = -99999.0f;
				for (auto i = intersections.begin(); i != intersections.end(); ++i)
				{
					if (i->second.t - lastT < std::numeric_limits<float>::epsilon()) continue;
					RayHit hit;
					hit.t = i->second.t;
					hit.frontFace = (i->second.flags == 0);
					m_RayResults[x][y].push_back(hit);
					lastT = hit.t;
				}
			}
		}
	}

	int queryPointIsInside(Ogre::Vector3 pos, int& numValidVotes)
	{
		numValidVotes += 2;
		pos -= m_ImagePlaneMin;
		int x = (int)std::floorf (pos[m_ImagePlaneAxis1] * m_InverseCellSize + 0.5f);
		int y = (int)std::floorf(pos[m_ImagePlaneAxis2] * m_InverseCellSize + 0.5f);
		if (x < 0 || x >= m_Width || y < 0 || y >= m_Height) return false;
		float z = pos[m_ImagePlaneNormalAxis];
		if (z <= 0) return false;

		int insideCounter = 0;
		const std::vector<RayHit>& intersections = m_RayResults[x][y];
		// if (intersections.size() % 2 == 0) numValidVotes += 2;
		int faceCounter = 0;
		auto i = intersections.begin();
		for (; i != intersections.end(); ++i)
		{
			if (i->t > z)
			{
				insideCounter += ((faceCounter % 2) == 1);
				break;
			}
			faceCounter++;
		}
		faceCounter = (int)(intersections.end() - i);
		insideCounter += ((faceCounter % 2) == 1);
		return insideCounter;
	}

	~RaycastCache()
	{
		for (int x = 0; x < m_Width; x++)
			delete[] m_RayResults[x];
		delete[] m_RayResults;
	}
};

/// Uses raycasting to compute the sign.
class TriangleMeshSDF_Robust : public TriangleMeshSDF
{
private:
	RaycastCache* m_RaycastCache1;
	RaycastCache* m_RaycastCache2;
	RaycastCache* m_RaycastCache3;

public:
	~TriangleMeshSDF_Robust()
	{
		if (m_RaycastCache1)
		{
			delete m_RaycastCache1;
			delete m_RaycastCache2;
			delete m_RaycastCache3;
		}
	}
	TriangleMeshSDF_Robust(std::shared_ptr<TransformedMesh> mesh) :
		TriangleMeshSDF(mesh), m_RaycastCache1(nullptr) {}
	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		if (m_RaycastCache1)
		{
			delete m_RaycastCache1;
			delete m_RaycastCache2;
			delete m_RaycastCache3;
		}
		Ogre::Vector3 aabbSize = aabb.getMax() - aabb.getMin();
		Profiler::Timestamp timeStamp = Profiler::timestamp();
		m_RaycastCache1 = new RaycastCache(m_RootNode.getBVH(), cellSize, aabbSize.x, aabbSize.y, aabb.getMin(), 2);
		m_RaycastCache2 = new RaycastCache(m_RootNode.getBVH(), cellSize, aabbSize.x, aabbSize.z, aabb.getMin(), 1);
		m_RaycastCache3 = new RaycastCache(m_RootNode.getBVH(), cellSize, aabbSize.y, aabbSize.z, aabb.getMin(), 0);
		Profiler::printJobDuration("Sign cache computation", timeStamp);
	}

	/// Check whether a point lies inside
	bool isInside(const Ogre::Vector3& point) const
	{
		bool inside = false;
		if (m_AABB.containsPoint(point))
		{
			int numVotes = 0;
			int insideVotes = m_RaycastCache1->queryPointIsInside(point, numVotes) + m_RaycastCache2->queryPointIsInside(point, numVotes) + m_RaycastCache3->queryPointIsInside(point, numVotes);
			inside = (insideVotes >= ((numVotes / 2)));
		}
		return inside;
	}
	Sample getSample(const Ogre::Vector3& point) const override
	{
		BVH<Surface>::ClosestLeafResult result;
		const Surface* tri = m_RootNode.getBVH()->getClosestLeaf(point, result);
		if (!isInside(point)) result.closestDistance *= -1;
		return Sample(result.closestDistance);
	};
};