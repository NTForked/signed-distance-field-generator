
#pragma once

#include <algorithm>
#include "OgreMath/OgreVector3.h"
#include "Ray.h"
#include "MathMisc.h"

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

	Ogre::Vector3 getCorner(int corner) const
	{
		Ogre::Vector3 sizeVec(max - min);
		return min + Ogre::Vector3((float)((corner & 4) != 0), (float)((corner & 2) != 0), (float)(corner & 1)) * sizeVec;
	}

	__forceinline bool intersectsAABB(const AABB& otherAABB) const
	{
		// perform separating axis test
		if (MathMisc::intervalDoesNotOverlap(min.x, max.x, otherAABB.min.x, otherAABB.max.x)) return false;
		if (MathMisc::intervalDoesNotOverlap(min.y, max.y, otherAABB.min.y, otherAABB.max.y)) return false;
		if (MathMisc::intervalDoesNotOverlap(min.z, max.z, otherAABB.min.z, otherAABB.max.z)) return false;
		return true;
	}

	__forceinline bool containsPoint(const Ogre::Vector3& point) const
	{
		return (point.x >= min.x && point.x < max.x
			&& point.y >= min.y && point.y < max.y
			&& point.z >= min.z && point.x < max.z);
	}

	__forceinline bool intersectsSphere(const Ogre::Vector3& center, float radius) const
	{
		// if (!intersectsAABB(AABB(center, radius))) return false;
		return squaredDistance(center) < radius*radius;
	}

	__forceinline float squaredDistance(const Ogre::Vector3& point) const
	{
		return MathMisc::aabbPointSquaredDistance(min, max, point);
	}

	__forceinline bool intersectsTriangle(const Ogre::Vector3& p1, const Ogre::Vector3& p2, const Ogre::Vector3& p3, const Ogre::Vector3& normal) const
	{
		// perform separating axis test
		// first check AABB face normals
		float tMin, tMax;
		MathMisc::projectTriangleOnAxis(Ogre::Vector3(1, 0, 0), p1, p2, p3, tMin, tMax);
		if (MathMisc::intervalDoesNotOverlap(min.x, max.x, tMin, tMax)) return false;
		MathMisc::projectTriangleOnAxis(Ogre::Vector3(0, 1, 0), p1, p2, p3, tMin, tMax);
		if (MathMisc::intervalDoesNotOverlap(min.y, max.y, tMin, tMax)) return false;
		MathMisc::projectTriangleOnAxis(Ogre::Vector3(0, 0, 1), p1, p2, p3, tMin, tMax);
		if (MathMisc::intervalDoesNotOverlap(min.z, max.z, tMin, tMax)) return false;

		float aabbMin, aabbMax;
		Ogre::Vector3 aabbPoints[8];
		for (int i = 0; i < 8; i++)
		{
			aabbPoints[i] = min;
			if ( i & 4) aabbPoints[i].x = max.x;
			if ( i & 2) aabbPoints[i].y = max.y;
			if ( i & 1) aabbPoints[i].z = max.z;
		}

		MathMisc::projectTriangleOnAxis(normal, p1, p2, p3, tMin, tMax);
		MathMisc::projectAABBOnAxis(normal, aabbPoints, aabbMin, aabbMax);
		if (MathMisc::intervalDoesNotOverlap(aabbMin, aabbMax, tMin, tMax)) return false;

		Ogre::Vector3 edges[3] = {(p2-p1), (p3-p1), (p3-p2)};
		for (int i = 0; i < 3; i++)
		{
			Ogre::Vector3 axis = edges[i];
			MathMisc::projectTriangleOnAxis(axis, p1, p2, p3, tMin, tMax);
			MathMisc::projectAABBOnAxis(axis, aabbPoints, aabbMin, aabbMax);
			if (MathMisc::intervalDoesNotOverlap(aabbMin, aabbMax, tMin, tMax)) return false;

			axis = edges[i].crossProduct(Ogre::Vector3(1, 0, 0));
			MathMisc::projectTriangleOnAxis(axis, p1, p2, p3, tMin, tMax);
			MathMisc::projectAABBOnAxis(axis, aabbPoints, aabbMin, aabbMax);
			if (MathMisc::intervalDoesNotOverlap(aabbMin, aabbMax, tMin, tMax)) return false;

			axis = edges[i].crossProduct(Ogre::Vector3(0, 1, 0));
			MathMisc::projectTriangleOnAxis(axis, p1, p2, p3, tMin, tMax);
			MathMisc::projectAABBOnAxis(axis, aabbPoints, aabbMin, aabbMax);
			if (MathMisc::intervalDoesNotOverlap(aabbMin, aabbMax, tMin, tMax)) return false;

			axis = edges[i].crossProduct(Ogre::Vector3(0, 0, 1));
			MathMisc::projectTriangleOnAxis(axis, p1, p2, p3, tMin, tMax);
			MathMisc::projectAABBOnAxis(axis, aabbPoints, aabbMin, aabbMax);
			if (MathMisc::intervalDoesNotOverlap(aabbMin, aabbMax, tMin, tMax)) return false;
		}
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