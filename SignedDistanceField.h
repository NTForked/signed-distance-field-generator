
#pragma once

#include "Vector3i.h"
#include "OgreMath/OgreVector3.h"
#include "AABB.h"

class SignedDistanceField3D
{
public:
	virtual float getSignedDistance(const Ogre::Vector3& point) const = 0;

	virtual bool intersectsSurface(const AABB& aabb) const = 0;

	virtual AABB getAABB() const = 0;
};

template<class UserData>
class SampledSignedDistanceField3D : public SignedDistanceField3D
{
public:
	struct Cube
	{
		Vector3i posMin;
		float signedDistances[8];
		UserData userData;
	};
	virtual std::vector<Cube> getCubesToMarch() = 0;
};