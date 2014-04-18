
#pragma once

#include "OgreMath/OgreVector3.h"
#include <vector>
#include "SolidGeometry.h"
#include "AABB.h"

class OpInvertSDF : public SolidGeometry
{
protected:
    SolidGeometry* m_SDF;

public:
    OpInvertSDF(SolidGeometry* sdf) : m_SDF(sdf) {}
    virtual void getSample(const Ogre::Vector3& point, Sample& sample) const override
	{
        m_SDF->getSample(point, sample);
		sample.signedDistance *= -1.0f;
	}

    virtual void raycastClosest(const Ray& ray, Sample& sample) const override
    {
        m_SDF->raycastClosest(ray, sample);
        sample.signedDistance *= -1.0f;
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
