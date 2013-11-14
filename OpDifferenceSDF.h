
#pragma once

#include "OgreMath/OgreVector3.h"
#include <vector>
#include "SignedDistanceField.h"
#include "AABB.h"

class OpDifferenceSDF : public SignedDistanceField3D
{
protected:
	SignedDistanceField3D* m_SDF1;
	SignedDistanceField3D* m_SDF2;
	AABB m_AABB;

public:
	OpDifferenceSDF(SignedDistanceField3D* sdf1, SignedDistanceField3D* sdf2) : m_SDF1(sdf1), m_SDF2(sdf2)
	{
		m_AABB = sdf1->getAABB();
		m_AABB = AABB(m_AABB, sdf2->getAABB());
	}
	float getSignedDistance(const Ogre::Vector3& point) const override
	{
		return std::min(m_SDF1->getSignedDistance(point), -m_SDF2->getSignedDistance(point));
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		return m_SDF1->intersectsSurface(aabb) || m_SDF2->intersectsSurface(aabb);
	}

	AABB getAABB() const override
	{
		return m_AABB;
	}

	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		m_SDF1->prepareSampling(aabb, cellSize);
		m_SDF2->prepareSampling(aabb, cellSize);
	}
};