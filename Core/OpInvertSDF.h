
#pragma once

#include "OgreMath/OgreVector3.h"
#include <vector>
#include "SignedDistanceField.h"
#include "AABB.h"

class OpInvertSDF : public SignedDistanceField3D
{
protected:
	SignedDistanceField3D* m_SDF;

public:
	OpInvertSDF(SignedDistanceField3D* sdf) : m_SDF(sdf) {}
	Sample getSample(const Ogre::Vector3& point) const override
	{
		Sample sample = m_SDF->getSample(point);
		sample.signedDistance *= -1.0f;
		return sample;
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		return m_SDF->intersectsSurface(aabb);
	}

	AABB getAABB() const override
	{
		return m_SDF->getAABB();
	}

	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		m_SDF->prepareSampling(aabb, cellSize);
	}
};