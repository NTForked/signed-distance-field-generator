
#pragma once

#include <vector>
#include "OgreMath/OgreVector3.h"
#include "OBJReader.h"
#include "Mesh.h"
#include "BVH.h"
#include "Surfaces.h"
#include "AABB.h"
#include "Profiler.h"

using std::vector;
using Ogre::Vector3;

class ImplicitTriangleMeshSDF
{
protected:
	BVHNodeThreaded<SphereBV, Surface>* m_RootNode;
	AABB m_AABB;
	std::shared_ptr<TransformedMesh> m_TransformedMesh;

	// Use aabbs for raycasting and AABB collision queries, it's faster.
	BVHNodeThreaded<AABB, Surface>* m_RootNodeAABB;
public:
	virtual ~ImplicitTriangleMeshSDF()
	{
		// It's important that the root node is destroyed before the TransformedMesh, because the TransformedMesh manages the leaves of the BVH.
		delete m_RootNode;
	}
	ImplicitTriangleMeshSDF(std::shared_ptr<TransformedMesh> mesh)
	{
		m_TransformedMesh = mesh;
		m_TransformedMesh->computeCache();
		vector<Surface*> surfaces;
		for (auto iTri = m_TransformedMesh->triangleSurfaces.begin(); iTri != m_TransformedMesh->triangleSurfaces.end(); ++iTri)
			surfaces.push_back(&(*iTri));

		Profiler::Timestamp timeStamp = Profiler::timestamp();

		const int numThreads = 8;
		m_RootNode = new BVHNodeThreaded<SphereBV, Surface>(surfaces, 0, (int)surfaces.size(), 0, static_cast<int>(std::log((double)numThreads)/std::log(2.0)));
		// BVHNodeThreaded<AABB, Surface> rootNodeAABB(surfaces, 0, surfaces.size(), 0, static_cast<int>(std::log((double)numThreads)/std::log(2.0)));
		Profiler::printJobDuration("BVH creation", timeStamp);
		std::cout << "Tree height max: " << m_RootNode->getHeight() << std::endl;
		std::cout << "Tree height avg: " << m_RootNode->getHeightAvg() << std::endl;

		timeStamp = Profiler::timestamp();
		m_RootNodeAABB = new BVHNodeThreaded<AABB, Surface>(surfaces, 0, (int)surfaces.size(), 0, static_cast<int>(std::log((double)numThreads)/std::log(2.0)));
		// BVHNodeThreaded<AABB, Surface> rootNodeAABB(surfaces, 0, surfaces.size(), 0, static_cast<int>(std::log((double)numThreads)/std::log(2.0)));
		Profiler::printJobDuration("AABB BVH creation", timeStamp);
		std::cout << "Tree height max: " << m_RootNodeAABB->getHeight() << std::endl;
		std::cout << "Tree height avg: " << m_RootNodeAABB->getHeightAvg() << std::endl;

		vector<Vector3> positions;
		for (auto i = m_TransformedMesh->vertexBufferVS.begin(); i != m_TransformedMesh->vertexBufferVS.end(); ++i)
			positions.push_back(i->position);
		m_AABB = AABB(positions);
		m_AABB.max *= 1.05f;
		m_AABB.min *= 1.05f;
	}

	bool intersectsSurface(const AABB& aabb) const
	{
		return m_RootNodeAABB->intersectsAABB(aabb);
	}

	AABB getAABB() const { return m_AABB; }
};

/// Uses the angle weighted pseudo normal for the sign computation.
class ImplicitTriangleMeshSDF_AWP : public ImplicitTriangleMeshSDF
{
public:
	ImplicitTriangleMeshSDF_AWP(std::shared_ptr<TransformedMesh> mesh) :
		ImplicitTriangleMeshSDF(mesh) {}
	inline float getSignedDistance(const Ogre::Vector3& point) const
	{
		BVH<Surface>::ClosestLeafResult result;
		const Surface* tri = m_RootNode->getClosestLeaf(point, result);
		// test sign
		Vector3 rayDir = result.closestPoint - point;
		if (rayDir.dotProduct(result.normal) >= 0.0f) result.closestDistance *= -1;
		// std::cout << result.closestDistance << std::endl;
		return result.closestDistance;
	};
};

/// Uses raycasting to compute the sign.
class ImplicitTriangleMeshSDF_RC : public ImplicitTriangleMeshSDF
{
private:
	const static int NUM_RAYCASTS = 6;
	Ogre::Vector3 m_CastVecs[NUM_RAYCASTS];

	static bool compareIntersections(const std::pair<const Surface*, Ray::Intersection> &int1, const std::pair<const Surface*, Ray::Intersection> &int2)
	{
		return int1.second.t < int2.second.t;
	}

	static int countRayHits(std::vector<std::pair<const Surface*, Ray::Intersection> >& intersections)
	{
		if (intersections.size() < 2) return (int)intersections.size();
		std::sort(intersections.begin(), intersections.end(), compareIntersections);
		int hits = 1;
		unsigned int lastTriFlags = intersections[0].second.flags;
		for (auto i = intersections.begin()+1; i != intersections.end(); ++i)
		{
			if (i->second.flags != lastTriFlags)
			{
				lastTriFlags = i->second.flags;
				hits++;
			}
		}
		return hits;
	}
	static int countRayHits2(std::vector<std::pair<const Surface*, Ray::Intersection> >& intersections)
	{
		if (intersections.size() < 2) return (int)intersections.size();
		std::sort(intersections.begin(), intersections.end(), compareIntersections);
		int hits = 1;
		float lastT = intersections[0].second.t;
		unsigned int lastTriFlags = intersections[0].second.flags;
		for (auto i = intersections.begin()+1; i != intersections.end(); ++i)
		{
			if (i->second.t > lastT + std::numeric_limits<float>::epsilon())
			{
				lastT = i->second.t;
				hits++;
			}
		}
		return hits;
	}
public:
	~ImplicitTriangleMeshSDF_RC()
	{
		delete m_RootNodeAABB;
	}
	ImplicitTriangleMeshSDF_RC(std::shared_ptr<TransformedMesh> mesh) :
		ImplicitTriangleMeshSDF(mesh)
	{
		m_CastVecs[0] = Ogre::Vector3(1, 0, 0);
		m_CastVecs[1] = Ogre::Vector3(0, 1, 0);
		m_CastVecs[2] = Ogre::Vector3(0, 0, 1);
		m_CastVecs[3] = Ogre::Vector3(-1, 0, 0);
		m_CastVecs[4] = Ogre::Vector3(0, -1, 0);
		m_CastVecs[5] = Ogre::Vector3(0, 0, -1);
	}
	inline float getSignedDistance(const Ogre::Vector3& point) const
	{
		BVH<Surface>::ClosestLeafResult result;
		const Surface* tri = m_RootNode->getClosestLeaf(point, result);
		// test sign
		float weightNotEven = 0.0f;
		float totalWeights = 0.0f;

		for (int i = 0; i < NUM_RAYCASTS; i++)
		{
			std::vector<std::pair<const Surface*, Ray::Intersection> > intersections;
			m_RootNodeAABB->rayIntersectAll(Ray(point, m_CastVecs[i]), intersections);
			if (intersections.empty())	// we are outside for sure
				return result.closestDistance * -1;
			float castWeight = 1.0f / intersections.size();
			weightNotEven += (countRayHits2(intersections) % 2) * castWeight;
			totalWeights += castWeight;
		}
		if (weightNotEven < totalWeights * 0.5f) result.closestDistance *= -1;
		return result.closestDistance;
	};
};