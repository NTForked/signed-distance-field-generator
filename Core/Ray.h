
#pragma once

#include <algorithm>
#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreMatrix3.h"
#include "Triangle.h"

class SceneRenderer;

class Ray
{
public:
	Ray(const Ogre::Vector3 &_origin, const Ogre::Vector3 &_direction) : origin(_origin), direction(_direction)
	{
		directionInv = Ogre::Vector3(1.0f, 1.0f, 1.0f) / direction;
		sign[0] = (directionInv.x < 0);
		sign[1] = (directionInv.y < 0);
		sign[2] = (directionInv.z < 0);
		/*directionInv.x = direction.x != 0.0f ? 1.0f / direction.x : std::numeric_limits<float>::infinity();
		directionInv.y = direction.y != 0.0f ? 1.0f / direction.y : std::numeric_limits<float>::infinity();
		directionInv.z = direction.z != 0.0f ? 1.0f / direction.z : std::numeric_limits<float>::infinity();*/
	}

	Ogre::Vector3 origin;
	Ogre::Vector3 direction, directionInv;
	int sign[3];

	struct Intersection
	{
		Intersection() : flags(0) {}
		float t;
		unsigned int flags;
		Ogre::Vector3 userData[1];
	};

	/// Tests the ray against intersection with a sphere given by a position and a radius.
	__forceinline bool intersectSphere(Intersection &intersection, const Ogre::Vector3 &sphereCenter, float squaredSphereRadius, float tNear = 0.0f, float tFar = 100000.0f) const
	{
		Ogre::Vector3 movedRayOrigin(origin - sphereCenter);

		float a = direction.dotProduct(direction);
		float b = 2 * (direction.dotProduct(movedRayOrigin));
		float c = movedRayOrigin.dotProduct(movedRayOrigin) - squaredSphereRadius;

		float disc = b*b - 4*a*c;
		if (disc < 0) return false;

		float discRoot = std::sqrtf(disc);

		float t0 = (-b - discRoot) / (2*a);
		float t1 = (-b + discRoot) / (2*a);

		// select closest hit
		float tnear = std::min(t0, t1);
		float tfar = std::max(t0, t1);
		if (tfar < tNear || tnear > tFar) return false;

		intersection.t = tnear;
		if (intersection.t < 0)
			intersection.t = tfar;

		return true;
	}

	static __forceinline bool isZero(float val)
	{
		return val < 0.00001f && val > -0.00001f;
	}

	// Test the ray against intersection with plane given by a normal and a position.
	__forceinline bool intersectPlane(Intersection &intersection, const Ogre::Vector3 &planeNormal, const Ogre::Vector3 &planePosition, bool& backface) const
	{
		float rayNormalAngle = planeNormal.dotProduct(direction);
		if (std::abs(rayNormalAngle) == 0) return false;
		backface = rayNormalAngle < 0;
		// if (rayNormalAngle >= 0) return false;	// plane is parallel or faces away from the ray

		//first move the ray so that we can test against a plane that goes through (0,0,0)
		Ogre::Vector3 movedRayOrigin(origin - planePosition);

		//dot(t*rayDir, n) = -dot(movedRayOrigin, planeNormal)
		intersection.t = -movedRayOrigin.dotProduct(planeNormal) / rayNormalAngle;
		return (intersection.t > 0.0f);
	}

	// Tests if the ray hits the plane at a closer distance than closestT
	__forceinline bool intersectPlaneUpdate(float &tOut, float closestT, const Ogre::Vector3 &planeNormal, const Ogre::Vector3 &planePosition) const
	{
		float rayNormalAngle = planeNormal.dotProduct(direction);
		if (rayNormalAngle >= 0) return false;	// plane is parallel or faces away from the ray

		//first move the ray so that we can test against a plane that goes through (0,0,0)
		Ogre::Vector3 movedRayOrigin(origin - planePosition);

		//dot(t*rayDir, n) = -dot(movedRayOrigin, planeNormal)
		tOut = -movedRayOrigin.dotProduct(planeNormal) / rayNormalAngle;
		return (tOut > 0.0f && tOut < closestT);
	}

	// Tests the ray against intersection with an AABB given by min and max vectors.
	// Source: http://www.cs.utah.edu/~awilliam/box/box.pdf
	__forceinline bool intersectAABB(const Ogre::Vector3 bounds[2], float tNear, float tFar) const
	{
		float tMin, tMax, t2Min, t2Max;

		tMin = (bounds[sign[0]].x - origin.x) * this->directionInv.x;
		tMax = (bounds[1-sign[0]].x - origin.x) * this->directionInv.x;

		t2Min = (bounds[sign[1]].y - origin.y) * this->directionInv.y;
		t2Max = (bounds[1-sign[1]].y - origin.y) * this->directionInv.y;

		if (tMin > t2Max || t2Min > tMax) return false;
		tMin = std::max(tMin, t2Min);
		tMax = std::min(tMax, t2Max);

		t2Min = (bounds[sign[2]].z - origin.z) * this->directionInv.z;
		t2Max = (bounds[1-sign[2]].z - origin.z) * this->directionInv.z;

		if (tMin > t2Max || t2Min > tMax) return false;
		tMin = std::max(tMin, t2Min);
		tMax = std::min(tMax, t2Max);

		return (tMin < tFar && tMax > tNear);
	}
	__forceinline bool intersectAABB(const Ogre::Vector3 bounds[2]) const
	{
		float tMin, tMax, t2Min, t2Max;

		tMin = (bounds[sign[0]].x - origin.x) * this->directionInv.x;
		tMax = (bounds[1-sign[0]].x - origin.x) * this->directionInv.x;

		t2Min = (bounds[sign[1]].y - origin.y) * this->directionInv.y;
		t2Max = (bounds[1-sign[1]].y - origin.y) * this->directionInv.y;

		if (tMin > t2Max || t2Min > tMax) return false;
		tMin = std::max(tMin, t2Min);
		tMax = std::min(tMax, t2Max);

		t2Min = (bounds[sign[2]].z - origin.z) * this->directionInv.z;
		t2Max = (bounds[1-sign[2]].z - origin.z) * this->directionInv.z;

		if (tMin > t2Max || t2Min > tMax) return false;
		tMin = std::max(tMin, t2Min);
		tMax = std::min(tMax, t2Max);

		return (tMax > 0.0f);
	}

	/// Tests the ray against intersection with a triangle given by three points.
	/*__forceinline bool intersectTriangle(Intersection &intersection, const TriangleCached& triData) const
	{
		if (triData.normal.dotProduct(this->direction) >= 0)
			return false;
		Ogre::Matrix3 system(	triData.p2.x-triData.p1.x, triData.p3.x-triData.p1.x, -this->direction.x,
								triData.p2.y-triData.p1.y, triData.p3.y-triData.p1.y, -this->direction.y,
								triData.p2.z-triData.p1.z, triData.p3.z-triData.p1.z, -this->direction.z);
		Ogre::Matrix3 inverse;
		if (!system.Inverse(inverse, 0)) return false;
		Ogre::Vector3 &barycentricCoords = intersection.userData[0];
		Ogre::Vector3 solution = inverse*(this->origin-p1);
		if (solution.z <= 0.0f) return false;
		barycentricCoords.x = solution.x;
		barycentricCoords.y = solution.y;
		barycentricCoords.z = 1.0f-barycentricCoords.x-barycentricCoords.y;
		intersection.t = solution.z;
		return (barycentricCoords.x >= 0) && (barycentricCoords.y >= 0) && (barycentricCoords.z >= 0);
	}*/

	/// Tests the ray against intersection with a triangle.
	__forceinline bool intersectTriangle(Intersection &intersection, const TriangleCached& triData) const
	{
		bool backface = false;
		if (!intersectPlane(intersection, triData.normal, triData.p1, backface)) return false;
		intersection.flags = backface;

		Ogre::Vector3 intersectPosition = this->origin + intersection.t*this->direction;
		//write barycentric coordinates to intersection.userData[0]
		intersection.userData[0] = triData.getBarycentricCoordinates(intersectPosition);
		static const float epsilon = -0.0001f;
		return ((intersection.userData[0].x >= epsilon) && (intersection.userData[0].y >= epsilon) && (intersection.userData[0].z >= epsilon));
	}

	/// Tests the ray against intersection with a triangle and checks whether the intersection is closer than the closest intersection.
	__forceinline bool intersectTriangleUpdate(Intersection &intersection, const TriangleCached& triData) const
	{
		float t;
		if (!intersectPlaneUpdate(t, intersection.t, triData.normal, triData.p1)) return false;

		Ogre::Vector3 intersectPosition = this->origin + t*this->direction;
		Ogre::Vector3 barycentricCoords = triData.getBarycentricCoordinates(intersectPosition);
		if ((barycentricCoords.x >= 0) && (barycentricCoords.y >= 0) && (barycentricCoords.z >= 0))
		{
			intersection.t = t;
			intersection.userData[0] = barycentricCoords;
			return true;
		}
		return false;
	}
};