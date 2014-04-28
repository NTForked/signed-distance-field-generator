
#pragma once

#include "SolidGeometry.h"
#include "BVH.h"
#include "Surfaces.h"

class VoronoiFragments : public SolidGeometry
{
protected:
    BVH<PointBVH>* m_BVH;
    int m_CurrentFragmentIndex;
    std::vector<PointBVH*> m_FragmentPoints;
    std::vector<std::pair<Ogre::Vector3, Ogre::Vector3 > > m_CutPlanes;
    AABB m_AABB;

public:
	static std::vector<Ogre::Vector3> generateFragmentPointsUniform(const AABB&& aabb, unsigned int numPoints)
	{
		std::vector<Ogre::Vector3> points;
		for (unsigned int i = 0; i < numPoints; i++)
		{
			points.push_back(Ogre::Vector3(
				Ogre::Math::RangeRandom(aabb.min.x, aabb.max.x),
				Ogre::Math::RangeRandom(aabb.min.y, aabb.max.y),
				Ogre::Math::RangeRandom(aabb.min.z, aabb.max.z)));
		}
		return points;
	}

    VoronoiFragments(const std::vector<Ogre::Vector3>& fragmentPoints);
    ~VoronoiFragments();

    void setFragment(int index);

	/// Retrieves the sample at the given point (exact for implicit SDFs, interpolated for sampled SDFs).
	virtual void getSample(const Ogre::Vector3& point, Sample& sample) const override;

    bool getSign(const Ogre::Vector3& point) const override;

	/// Retrieves whether the given AABB intersects the surface (zero contour of the sdf).
	virtual bool intersectsSurface(const AABB& aabb) const override;

	/// Retrieves the axis aligned bounding box of the sdf.
    virtual AABB getAABB() const override { return m_AABB; }
};
