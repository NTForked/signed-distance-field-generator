
#pragma once

#include "Prerequisites.h"
#include <algorithm>
#include <stack>
#include <vector>
#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreMatrix4.h"
#include "Ray.h"
#include "Mesh.h"
#include "BVH.h"
#include "Sphere.h"
#include "MathMisc.h"

class Surface : public BVH<Surface>
{
protected:
	AABB m_AABB;
	// SphereBV m_SphereBV;
	std::vector<Ogre::Vector3> m_ExtremalPoints;
public:
	virtual ~Surface() {}

	/// Retrieves the aabb of the Surface.
	__forceinline void getBoundingVolume(AABB& aabb) { aabb = m_AABB; }

	// __forceinline void getBoundingVolume(SphereBV& sphereBV) { sphereBV = m_SphereBV; }

	__forceinline const std::vector<Ogre::Vector3>& getExtremalPoints() { return m_ExtremalPoints; }

	float squaredDistance(const Ogre::Vector3& point) const { return m_AABB.squaredDistance(point); }
};

class TriangleSurface : public Surface
{
private:
	TransformedMesh *mMesh;
	unsigned int mTriangleIndex, mIndex1, mIndex2, mIndex3;

public:
	TriangleSurface(TransformedMesh *mesh, unsigned int triangleIndex)
		: mMesh(mesh), mTriangleIndex(triangleIndex), mIndex1(mesh->getMesh()->indexBuffer[triangleIndex*3]), mIndex2(mesh->getMesh()->indexBuffer[triangleIndex*3+1]), mIndex3(mesh->getMesh()->indexBuffer[triangleIndex*3+2])
		{ mType = PRIMITIVE; }
	~TriangleSurface() {}
	const Surface* rayIntersectClosest(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (ray.intersectTriangle(intersection,
			mMesh->triangleDataVS[mTriangleIndex]))
			return this;
		return nullptr;
	}
	const Surface* rayIntersectUpdate(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (ray.intersectTriangleUpdate(intersection,
			mMesh->triangleDataVS[mTriangleIndex]))
			return this;
		return nullptr;
	}

	const TriangleCached& getCachedTriangle() const
	{
		return mMesh->triangleDataVS[mTriangleIndex];
	}

	const Surface* getClosestLeaf(const Ogre::Vector3& point, ClosestLeafResult& result) const override
	{
		// first check vertices
		Ogre::Vector3 trianglePoints[3];
		trianglePoints[0] = mMesh->vertexBufferVS[mIndex1].position;
		trianglePoints[1] = mMesh->vertexBufferVS[mIndex2].position;
		trianglePoints[2] = mMesh->vertexBufferVS[mIndex3].position;
		float bestSquaredDist = result.closestDistance * result.closestDistance;
		const Surface* ret = nullptr;
		for (int i = 0; i < 3; i++)
		{
			float squaredDist = trianglePoints[i].squaredDistance(point);
			if (squaredDist < bestSquaredDist)
			{
				bestSquaredDist = squaredDist;
				result.closestPoint = trianglePoints[i];
				result.normal = mMesh->triangleDataVS[mTriangleIndex].vertexPseudoNormals[i];
				ret = this;
			}
		}

		// then edges
		for (int i = 0; i < 3; i++)
		{
			if (mMesh->triangleDataVS[mTriangleIndex].dirtyEdgeNormals[i]) continue;
			float x;
			Ogre::Vector3 closestPointCandidate = MathMisc::projectPointOnLine(point, trianglePoints[i], trianglePoints[(i+1)%3], x);
			if (x > 0 && x < 1)
			{
				float squaredDist = closestPointCandidate.squaredDistance(point);
				if (squaredDist < bestSquaredDist)
				{
					bestSquaredDist = squaredDist;
					result.closestPoint = closestPointCandidate;
					result.normal = mMesh->triangleDataVS[mTriangleIndex].edgePseudoNormals[i];
					ret = this;
				}
			}
		}

		// and finally the triangle
		if (!mMesh->triangleDataVS[mTriangleIndex].degenerated)
		{
			float distToPlane = mMesh->triangleDataVS[mTriangleIndex].normal.dotProduct(point - trianglePoints[0]);
			Vector3 closestPointCandidate = point -  mMesh->triangleDataVS[mTriangleIndex].normal * distToPlane;
			Ogre::Vector3 barycentricCoordinates = mMesh->triangleDataVS[mTriangleIndex].getBarycentricCoordinates(closestPointCandidate);

			if (barycentricCoordinates.x > 0 && barycentricCoordinates.y > 0 && barycentricCoordinates.z > 0)
			{
				float squaredDist = closestPointCandidate.squaredDistance(point);
				if (squaredDist < bestSquaredDist)
				{
					bestSquaredDist = squaredDist;
					result.closestPoint = closestPointCandidate;
					result.normal = mMesh->triangleDataVS[mTriangleIndex].normal;
					ret = this;
				}
			}
		}
		if (ret) result.closestDistance = std::sqrtf(bestSquaredDist);
		return ret;
	}

	Ogre::Vector3 getTriangleNormal()
	{
		return mMesh->triangleDataVS[mTriangleIndex].normal;
	}

	void updateBoundingVolume()
	{
		m_ExtremalPoints.clear();
		m_ExtremalPoints.push_back(mMesh->vertexBufferVS[mIndex1].position);
		m_ExtremalPoints.push_back(mMesh->vertexBufferVS[mIndex2].position);
		m_ExtremalPoints.push_back(mMesh->vertexBufferVS[mIndex3].position);
		m_AABB = AABB(m_ExtremalPoints);
		// m_SphereBV = SphereBV(m_ExtremalPoints);
	}

	bool intersectsAABB(const AABB& aabb) const override
	{
		return aabb.intersectsTriangle(mMesh->vertexBufferVS[mIndex1].position, mMesh->vertexBufferVS[mIndex2].position, mMesh->vertexBufferVS[mIndex3].position, mMesh->triangleDataVS[mTriangleIndex].normal);
		// return m_AABB.intersectsAABB(aabb);
	}
};