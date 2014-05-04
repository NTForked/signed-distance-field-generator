
#pragma once

#include <memory>
#include "Vector3i.h"
#include "OgreMath/OgreVector3.h"
#include "AABB.h"
#include "Area.h"
#include "OgreMath/OgreVector2.h"
#include "Mesh.h"

/**
Interface for three-dimensional signed distance fields. Such a signed distance field can be sampled on a grid, but it could also be encoded implicitly.
An example is a sphere with center and radius, but also a triangle mesh.
*/

typedef unsigned int MaterialID;

class SolidGeometry
{
public:
	// If you want to store additional data in the signed distance grid, this is the right place to add it.
	struct Sample
	{
		Sample() {}
		Sample(float signedDistance) : signedDistance(signedDistance) {}
		Sample(float signedDistance, const Ogre::Vector3& closestSurfacePos) : signedDistance(signedDistance), closestSurfacePos(closestSurfacePos) {}
		Sample(float signedDistance, const Ogre::Vector3& closestSurfacePos, const Ogre::Vector3& normal, const Ogre::Vector2& uv, const MaterialID& materialID)
			: signedDistance(signedDistance), closestSurfacePos(closestSurfacePos), normal(normal), uv(uv), materialID(materialID) {}

		float signedDistance;

		MaterialID materialID;

		Ogre::Vector3 normal;
		Ogre::Vector3 closestSurfacePos;
		Ogre::Vector2 uv;

		// float alignmentFiller;

		// Operators required for trilinear interpolation, obviously this can not handle the materialID correctly.
		inline Sample operator * (float rhs) const
		{
			return Sample(signedDistance * rhs, closestSurfacePos, normal * rhs, uv * rhs, materialID);
		}
		inline Sample operator + (const Sample& rhs) const
		{
			return Sample(signedDistance + rhs.signedDistance, closestSurfacePos + rhs.closestSurfacePos, normal + rhs.normal, uv + rhs.uv, materialID);
		}
	};

	// Convenient helper methods for subclasses.
	static bool allSignsAreEqual(const float* signedDistances)
	{
		bool positive = (signedDistances[0] >= 0.0f);
		for (int i = 1; i < 8; i++)
		{
			if ((signedDistances[i] >= 0.0f) != positive)
				return false;
		}
		return true;
	}

	static bool allSignsAreEqual(const Sample** samples)
	{
		bool positive = (samples[0]->signedDistance >= 0.0f);
		for (int i = 1; i < 8; i++)
		{
			if ((samples[i]->signedDistance >= 0.0f) != positive)
				return false;
		}
		return true;
	}

	static bool signsAreEqual(float val1, float val2)
	{
		return (val1 >= 0.0f && val2 >= 0.0f) || (val1 < 0.0f && val2 < 0.0f);
	}

	/// Retrieves the sample at the given point (exact for implicit SDFs, interpolated for sampled SDFs).
    virtual void getSample(const Ogre::Vector3& point, Sample& sample) const = 0;

    /// Retrieves whether the given AABB intersects the surface (zero contour of the sdf).
    virtual bool intersectsSurface(const AABB& aabb) const = 0;

    /// Retrieves the axis aligned bounding box of the sdf.
    virtual AABB getAABB() const = 0;

    virtual bool raycastClosest(const Ray&, Sample&) const { return false; }

    virtual bool getSign(const Ogre::Vector3& point) const { return getSample(point).signedDistance >= 0.0f; }

    /// Implementations may override this to provide high speed implementations for cubic aabbs.
    virtual bool cubeNeedsSubdivision(const Area& area) const { return intersectsSurface(area.toAABB()); }

    /// Called before the first call to getSample. Usually not required, only used by TriangleMeshSDF_Robust so far which builds a grid.
    virtual void prepareSampling(const AABB&, float) {}

    Sample getSample(const Ogre::Vector3& point) const { Sample s; getSample(point, s); return s; }
};

// Interface for signed distance fields that can be processed using the Marching Cubes algorithm.
class SampledSolidGeometry : public SolidGeometry
{
public:
	struct Cube
	{
		Vector3i posMin;
		const Sample* cornerSamples[8];
	};

	virtual std::shared_ptr<Mesh> generateMesh() = 0;

	virtual float getInverseCellSize() = 0;
};
