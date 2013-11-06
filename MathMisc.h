
#pragma once

#include "OgreMath/OgreVector3.h"

using Ogre::Vector3;

struct MathMisc
{
	// http://www.gamedev.net/topic/552906-closest-point-on-triangle/
	static Vector3 closestPointOnTriangle( const Vector3 *triangle, const Vector3 &sourcePosition, float& s, float &t )
	{
		Vector3 edge0 = triangle[1] - triangle[0];
		Vector3 edge1 = triangle[2] - triangle[0];
		Vector3 v0 = triangle[0] - sourcePosition;

		float a = edge0.dotProduct( edge0 );
		float b = edge0.dotProduct( edge1 );
		float c = edge1.dotProduct( edge1 );
		float d = edge0.dotProduct( v0 );
		float e = edge1.dotProduct( v0 );

		float det = a*c - b*b;
		s = b*e - c*d;
		t = b*d - a*e;

		if ( s + t < det )
		{
			if ( s < 0.f )
			{
				if ( t < 0.f )
				{
					if ( d < 0.f )
					{
						s = Ogre::Math::Clamp( -d/a, 0.f, 1.f );
						t = 0.f;
					}
					else
					{
						s = 0.f;
						t = Ogre::Math::Clamp( -e/c, 0.f, 1.f );
					}
				}
				else
				{
					s = 0.f;
					t = Ogre::Math::Clamp( -e/c, 0.f, 1.f );
				}
			}
			else if ( t < 0.f )
			{
				s = Ogre::Math::Clamp( -d/a, 0.f, 1.f );
				t = 0.f;
			}
			else
			{
				float invDet = 1.f / det;
				s *= invDet;
				t *= invDet;
			}
		}
		else
		{
			if ( s < 0.f )
			{
				float tmp0 = b+d;
				float tmp1 = c+e;
				if ( tmp1 > tmp0 )
				{
					float numer = tmp1 - tmp0;
					float denom = a-2*b+c;
					s = Ogre::Math::Clamp( numer/denom, 0.f, 1.f );
					t = 1-s;
				}
				else
				{
					t = Ogre::Math::Clamp( -e/c, 0.f, 1.f );
					s = 0.f;
				}
			}
			else if ( t < 0.f )
			{
				if ( a+d > b+e )
				{
					float numer = c+e-b-d;
					float denom = a-2*b+c;
					s = Ogre::Math::Clamp( numer/denom, 0.f, 1.f );
					t = 1-s;
				}
				else
				{
					s = Ogre::Math::Clamp( -e/c, 0.f, 1.f );
					t = 0.f;
				}
			}
			else
			{
				float numer = c+e-b-d;
				float denom = a-2*b+c;
				s = Ogre::Math::Clamp( numer/denom, 0.f, 1.f );
				t = 1.f - s;
			}
		}

		return triangle[0] + s * edge0 + t * edge1;
	}

	__forceinline static float rayPlaneIntersection(const Ogre::Vector3& rayOrigin, const Ogre::Vector3& rayDir, const Ogre::Vector3& planeNormal, const Ogre::Vector3& planePos)
	{
		float denom = rayDir.dotProduct(planeNormal);
		if (std::fabs(denom) <= 0.000001f) denom = 0.000001f;
		return planeNormal.dotProduct(planePos - rayOrigin) / denom;
	}

	__forceinline static Ogre::Vector3 projectPointOnLine(const Ogre::Vector3& p, const Ogre::Vector3& line1, const Ogre::Vector3& line2, float &t)
	{
		Ogre::Vector3 rayDir = line2 - line1;
		t = rayPlaneIntersection(line1, rayDir, rayDir, p);
		return line1 + rayDir * t;
	}

	/// Projects a triangle on an axis, retrieves interval (tMin, tMax).
	__forceinline static void projectTriangleOnAxis(const Ogre::Vector3& axis, const Ogre::Vector3& p1, const Ogre::Vector3& p2, const Ogre::Vector3& p3, float& tMin, float& tMax)
	{
		tMin = axis.dotProduct(p1);
		tMax = tMin;
		float t = axis.dotProduct(p2);
		if (t < tMin) tMin = t;
		else if (t > tMax) tMax = t;
		t = axis.dotProduct(p3);
		if (t < tMin) tMin = t;
		else if (t > tMax) tMax = t;
	}

	/// Projects an AABB on an axis, retrieves interval (tMin, tMax).
	__forceinline static void projectAABBOnAxis(const Ogre::Vector3& axis, const Ogre::Vector3* aabbPoints, float& tMin, float& tMax)
	{
		tMin = axis.dotProduct(aabbPoints[0]);
		tMax = tMin;
		for (int i = 1; i < 8; i++)
		{
			float t = axis.dotProduct(aabbPoints[i]);
			if (t < tMin) tMin = t;
			else if (t > tMax) tMax = t;
		}
	}

	__forceinline static bool intervalDoesNotOverlap(float i1Min, float i1Max, float i2Min, float i2Max)
	{
		return (i1Min > i2Max) || (i2Min > i1Max);
	}
};