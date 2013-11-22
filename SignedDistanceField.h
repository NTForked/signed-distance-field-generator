
#pragma once

#include "Vector3i.h"
#include "OgreMath/OgreVector3.h"
#include "AABB.h"
#include "OgreMath/OgreVector2.h"

class SignedDistanceField3D
{
protected:
	struct Sample
	{
		Sample() {}
		Sample(float signedDistance) : signedDistance(signedDistance) {}
		Sample(float signedDistance, const Ogre::Vector3& normal, const Ogre::Vector2& uv)
			: signedDistance(signedDistance), normal(normal), uv(uv) {}
		float signedDistance;

		Ogre::Vector3 normal;
		Ogre::Vector2 uv;

		// Operators required for trilinear interpolation.
		inline Sample operator * (float rhs) const
		{
			return Sample(signedDistance * rhs, normal * rhs, uv * rhs);
		}
		inline Sample operator + (const Sample& rhs) const
		{
			return Sample(signedDistance + rhs.signedDistance, normal + rhs.normal, uv + rhs.uv);
		}
	};

	// Convenient helper methods for subclasses.
	bool allSignsAreEqual(float* signedDistances)
	{
		bool positive = (signedDistances[0] >= 0.0f);
		for (int i = 1; i < 8; i++)
		{
			if ((signedDistances[i] >= 0.0f) != positive)
				return false;
		}
		return true;
	}

	bool allSignsAreEqual(Sample* samples)
	{
		bool positive = (samples[0].signedDistance >= 0.0f);
		for (int i = 1; i < 8; i++)
		{
			if ((samples[i].signedDistance >= 0.0f) != positive)
				return false;
		}
		return true;
	}
public:
	virtual Sample getSample(const Ogre::Vector3& point) const = 0;

	virtual bool intersectsSurface(const AABB& aabb) const = 0;

	virtual AABB getAABB() const = 0;

	/// Usually not required, only used TriangleMeshSDF_Robust so far which builds a grid.
	virtual void prepareSampling(const AABB& aabb, float cellSize) {}
};

class SampledSignedDistanceField3D : public SignedDistanceField3D
{
public:
	struct Cube
	{
		Vector3i posMin;
		Sample cornerSamples[8];
	};
	virtual std::vector<Cube> getCubesToMarch() = 0;
};