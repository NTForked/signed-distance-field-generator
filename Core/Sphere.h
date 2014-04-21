
#pragma once

#include "OgreMath/OgreVector3.h"
#include "Ray.h"
#include "AABB.h"
#include "SolidGeometry.h"
#include "MathMisc.h"

class Sphere
{
public:
	Ogre::Vector3 center;
	float radius, radiusSquared;

	Sphere() : center(Ogre::Vector3(0, 0, 0)), radius(1.0f), radiusSquared(1.0f) {}
	Sphere(const Ogre::Vector3& center, float radius) : center(center), radius(radius), radiusSquared(radius*radius) {}
	virtual ~Sphere() {}

	__forceinline bool intersectsSphere(const Ogre::Vector3& otherCenter, float otherRadius) const
	{
		float radiusSum = radius + otherRadius;
		return center.squaredDistance(otherCenter) <= radiusSum*radiusSum;
	}

    __forceinline bool containsPoint(const Ogre::Vector3& point) const
	{
		return center.squaredDistance(point) <= radiusSquared;
	}

	__forceinline Ogre::Vector3 getCenter() const
	{
		return center;
	}

	__forceinline Ogre::Vector3 getMin() const
	{
		return center - Ogre::Vector3(radius, radius, radius);
	}

	__forceinline Ogre::Vector3 getMax() const
	{
		return center + Ogre::Vector3(radius, radius, radius);
	}

	bool intersectsAABB(const AABB& aabb) const
	{
		return MathMisc::aabbPointSquaredDistance(aabb.min, aabb.max, center) < radiusSquared;
	}
};

class SphereSDF : public Sphere, public SolidGeometry
{
public:
	SphereSDF() {}
	SphereSDF(const Ogre::Vector3& center, float radius) : Sphere(center, radius) {}

	virtual void getSample(const Ogre::Vector3& point, Sample& sample) const override
	{
        float distToCenter = std::sqrtf(center.squaredDistance(point));
        sample.signedDistance = radius - distToCenter;
		if (distToCenter == 0.0f)
			sample.closestSurfacePos = center;
		else
		{
			sample.normal = (point - center) / distToCenter;
            sample.closestSurfacePos = center + sample.normal * radius;
		}	
	}

	virtual bool getSign(const Ogre::Vector3& point) const override
	{
        return (radiusSquared - center.squaredDistance(point) > 0);
	}

	virtual bool intersectsSurface(const AABB& aabb) const override
	{
		if (!intersectsAABB(aabb))
			return false;		// aabb and sphere do not overlap
		for (int i = 0; i < 8; i++)
		{
			if (!containsPoint(aabb.getCorner(i)))
				return true;
		}
		// if the sphere contains all aabb corners, the AABB is completely inside the sphere and does not intersect the surface
		return false;
	}

    virtual void raycastClosest(const Ray& ray, Sample& sample) const override
    {
        Ray::Intersection intersection;
        if (ray.intersectSphere(intersection, center, radiusSquared))
        {
            sample.closestSurfacePos = ray.origin + ray.direction * intersection.t;
            sample.signedDistance = intersection.t;
            sample.normal = ray.origin - center;
            sample.normal.normalise();
            if (!getSign(ray.origin))
                sample.signedDistance *= -1.0f;
        }
    }

	virtual AABB getAABB() const override
	{
		return AABB(getMin(), getMax());
	}
};

class SphereBV : public Sphere
{
public:
	SphereBV() {}
	SphereBV(const Ogre::Vector3& center, float radius) : Sphere(center, radius) {}
	SphereBV(const SphereBV& sphere1, const SphereBV& sphere2)
	{
		Ogre::Vector3 diff = sphere2.center - sphere1.center;
		diff.normalise();
		Ogre::Vector3 p1 = sphere1.center - diff * sphere1.radius;
		Ogre::Vector3 p2 = sphere2.center + diff * sphere2.radius;
		if (sphere2.containsPoint(p1))
			*this = sphere2;
		else if (sphere1.containsPoint(p2))
			*this = sphere1;
		else
		{
			radius = p1.distance(p2) * 0.5f;
			center = (p1 + p2) * 0.5f;
		}
	}
	/// Points constructor.
	SphereBV(const std::vector<Ogre::Vector3> &points)
	{
		vAssert(points.size() > 2);
		/*
		Ritter: No convincing results compared to AABB center
		int aIndex = 0;
		int bIndex = 0;
		float maxDist = 0;
		for (int i = 1; i < points.size(); i++)
		{
			float dist = points[i].squaredDistance(points[0]);
			if (dist > maxDist)
			{
				aIndex = i;
				maxDist = dist;
			}
		}
		maxDist = 0;
		for (int i = 0; i < points.size(); i++)
		{
			float dist = points[i].squaredDistance(points[aIndex]);
			if (dist > maxDist)
			{
				bIndex = i;
				maxDist = dist;
			}
		}
		radius = std::sqrtf(maxDist) * 0.5f;
		center = (points[aIndex] + points[bIndex]) * 0.5f;

		for (auto i = points.begin(); i != points.end(); ++i)
			addPoint(*i);*/

		AABB aabb(points);
		radius = aabb.getMax().distance(aabb.getMin()) * 0.5f;
		center = (aabb.getMin() + aabb.getMax()) * 0.5f;

		Ogre::Vector3 centerOfMass(0, 0, 0);
		for (auto i = points.begin(); i != points.end(); ++i)
		{
			centerOfMass += *i;
		}
		centerOfMass /= (float)points.size();
		float maxDist = 0;
		for (auto i = points.begin(); i != points.end(); ++i)
		{
			float dist = i->squaredDistance(centerOfMass);
			if (dist > maxDist) maxDist = dist;
		}
		maxDist = std::sqrtf(maxDist);
		if (maxDist < radius)
		{
			radius = maxDist;
			center = centerOfMass;
		}
		// radius *= 1.1f;
		/*if (radius + radius * 0.1f < aabbradius)
		{
			std::cout << "Radius good." << std::endl;
		}*/

	}

	void addPoint(const Ogre::Vector3& point)
	{
		if (center.squaredDistance(point) > radius*radius)
		{
			Ogre::Vector3 dir = point - center;
			dir.normalise();
			center = ((center - dir * radius) + point) * 0.5f;
			radius = center.distance(point);
		}
	}

	__forceinline bool intersectsBV(const SphereBV& otherSphere) const
	{
		return intersectsSphere(otherSphere.center, otherSphere.radius);
	}
};
