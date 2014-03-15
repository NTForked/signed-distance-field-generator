
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

	virtual AABB getAABB() const override
	{
		return *this;
	}
};