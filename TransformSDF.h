
#pragma once

#include "OgreMath/OgreMatrix4.h"
#include <vector>
#include "SignedDistanceField.h"
#include "AABB.h"

class TransformSDF : public SignedDistanceField3D
{
protected:
	std::shared_ptr<SignedDistanceField3D> m_SDF;
	Ogre::Matrix4 m_Transform;
	Ogre::Matrix4 m_InverseTransform;
	AABB m_AABB;

public:
	TransformSDF(std::shared_ptr<SignedDistanceField3D> sdf, const Ogre::Matrix4& transform) : m_SDF(sdf), m_Transform(transform)
	{
		m_InverseTransform = transform.inverse();
		std::vector<Ogre::Vector3> points;
		for (int i = 0; i < 8; i++)
			points.push_back(m_Transform * m_SDF->getAABB().getCorner(i));
		m_AABB = AABB(points);
	}
	Sample getSample(const Ogre::Vector3& point) const override
	{
		return m_SDF->getSample(m_InverseTransform * point);
	}

	bool intersectsSurface(const AABB& aabb) const override
	{
		std::vector<Ogre::Vector3> points;
		for (int i = 0; i < 8; i++)
			points.push_back(m_InverseTransform * aabb.getCorner(i));
		return m_SDF->intersectsSurface(AABB(points));
	}

	AABB getAABB() const override
	{
		return m_AABB;
	}

	void prepareSampling(const AABB& aabb, float cellSize) override
	{
		std::vector<Ogre::Vector3> points;
		for (int i = 0; i < 8; i++)
			points.push_back(m_InverseTransform * aabb.getCorner(i));
		m_SDF->prepareSampling(AABB(points), cellSize);
	}
};