
#pragma once

#include "AABB.h"
#include "SignedDistanceField.h"

class AABBSDF : public AABB, public SignedDistanceField3D
{
public:
	AABBSDF() {}
	AABBSDF(const Ogre::Vector3 &_min, const Ogre::Vector3 &_max) : AABB(_min, _max) {}
	virtual Sample getSample(const Ogre::Vector3& point) const override
	{
		Sample s;
		if (containsPoint(point))
		{
			Ogre::Vector3 minVec = point - min;
			Ogre::Vector3 maxVec = max - point;
			s.signedDistance = std::min(minVec.minComponent(), maxVec.minComponent());
		}
		else
		{
			s.signedDistance = -std::sqrtf(squaredDistance(point));
		}
		// todo: compute normal
		return s;
	}

	virtual bool getSign(const Ogre::Vector3& point) const override
	{
		return containsPoint(point);
	}

	virtual bool intersectsSurface(const AABB& aabb) const override
	{
		if (!intersectsAABB(aabb)) return false;
		for (int i = 0; i < 8; i++)
		{
			if (!containsPoint(aabb.getCorner(i)))
				return true;
		}
		return false;
	}

	/*bool cubeNeedsSubdivision(const Area& area) const override
	{
		if (!intersectsSurface(area.toAABB()))
			return false;

		for (int i = 0; i < 8; i++)
		{
			if (area.containsPoint(getCorner(i)))
				return true;
		}

		return false;

		// project mid on the cube surface and check the approximation error
	}*/

	virtual AABB getAABB() const override
	{
		return *this;
	}
};