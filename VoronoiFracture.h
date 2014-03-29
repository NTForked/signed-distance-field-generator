
#pragma once

#include "FracturePattern.h"

class VoronoiFracture : public FracturePattern
{
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

	VoronoiFracture(const std::vector<Ogre::Vector3>& fragmentPoints)
	{
		for (auto i1 = fragmentPoints.begin(); i1 != fragmentPoints.end(); i1++)
		{

		}
	}
};