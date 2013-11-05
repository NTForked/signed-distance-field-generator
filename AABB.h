
#pragma once

#include <algorithm>
#include "OgreMath/OgreVector3.h"
#include "Ray.h"

struct AABB
{
	Ogre::Vector3 min, max;
	AABB() {}
	AABB(const Ogre::Vector3 &_min, const Ogre::Vector3 &_max) : min(_min), max(_max) {}

	/// Merge constructor
	AABB(const AABB &aabb1, const AABB &aabb2)
	{
		float epsilon = 0.001f;
		min.x = std::min(aabb1.min.x, aabb2.min.x);
		min.y = std::min(aabb1.min.y, aabb2.min.y);
		min.z = std::min(aabb1.min.z, aabb2.min.z);
		max.x = std::max(aabb1.max.x, aabb2.max.x);
		max.y = std::max(aabb1.max.y, aabb2.max.y);
		max.z = std::max(aabb1.max.z, aabb2.max.z);
	}

	/// Sphere constructor
	AABB(const Ogre::Vector3 &sphereCenter, float sphereRadius)
	{
		min = sphereCenter - Ogre::Vector3(sphereRadius, sphereRadius, sphereRadius);
		max = sphereCenter + Ogre::Vector3(sphereRadius, sphereRadius, sphereRadius);
	}

	/// Points constructor.
	AABB(const std::vector<Ogre::Vector3> &points)
	{
		assert(points.size() > 1);
		min = *points.begin();
		max = min;
		for (auto i = points.begin() + 1; i != points.end(); ++i)
		{
			min.x = std::min(min.x, i->x);
			min.y = std::min(min.y, i->y);
			min.z = std::min(min.z, i->z);
			max.x = std::max(max.x, i->x);
			max.y = std::max(max.y, i->y);
			max.z = std::max(max.z, i->z);
		}
	}

	__forceinline bool intersectsAABB(const AABB& otherAABB) const
	{
		// perform separating axis test
		if (max.x < otherAABB.min.x || otherAABB.max.x < min.x) return false;
		if (max.y < otherAABB.min.y || otherAABB.max.y < min.y) return false;
		if (max.z < otherAABB.min.z || otherAABB.max.z < min.z) return false;
		return true;
	}

	__forceinline bool containsPoint(const Ogre::Vector3& point) const
	{
		return (point.x >= min.x && point.x < max.x
			&& point.y >= min.y && point.y < max.y
			&& point.z >= min.z && point.x < max.z);
	}

	__forceinline bool intersectsSphere(const Ogre::Vector3& center, float radius)
	{
		// NIY
		return false;
	}

	__forceinline float squaredDistance(const Ogre::Vector3& point) const
	{
		// NIY
		return 0;
	}

	__forceinline bool isBehindPlane(const Ogre::Vector3& point, const Ogre::Vector3& normal)
	{
		float dist = (min - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(min[0], min[1], max[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(min[0], max[1], min[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(min[0], max[1], max[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(max[0], min[1], min[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(max[0], min[1], max[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (Ogre::Vector3(max[0], max[1], min[2]) - point).dotProduct(normal);
		if (dist > 0) return false;
		dist = (max - point).dotProduct(normal);
		if (dist > 0) return false;
		return true;
	}

	__forceinline bool rayIntersect(const Ray& ray, float tNear, float tFar) const
	{
		return ray.intersectAABB(&min, tNear, tFar);
	}
	__forceinline bool rayIntersect(const Ray& ray) const
	{
		return ray.intersectAABB(&min);
	}

	__forceinline Ogre::Vector3 getCenter() const
	{
		return (min + max) * 0.5f;
	}

	__forceinline const Ogre::Vector3& getMin() const
	{
		return min;
	}

	__forceinline const Ogre::Vector3& getMax() const
	{
		return max;
	}
};