
#pragma once

#include "OgreMath/OgreVector3.h"
#include <functional>
#include <vector>

template<class T>
class Triangle
{
public:
	T p1,p2,p3;
	Triangle() {}
	~Triangle() {}
	Triangle(const T &_p1, const T &_p2, const T &_p3) : p1(_p1), p2(_p2), p3(_p3) {}

	virtual Triangle transform(const std::function<T(const T&)> &transformF) const
	{
		return Triangle(transformF(p1), transformF(p2), transformF(p3));
	}
	static void transformTriangles(std::vector<Triangle> &output_triangles, const std::vector<Triangle> &input_triangles, const std::function<T(const T&)>& transformF)
	{
		for (unsigned int i = 0; i < input_triangles.size(); i++)
			output_triangles.push_back(input_triangles[i].transform(transformF));
	}

	friend std::ostream& operator<< (std::ostream &out, Triangle &tri)
	{
		out << tri.p1 << " " << tri.p2 << " " << tri.p3;
		return out;
	}
}; 

struct TriangleCached : public Triangle<Ogre::Vector3>
{
	TriangleCached() : degenerated(false)
	{
		for (int i = 0; i < 3; i++)
			dirtyEdgeNormals[i] = false;
	}
	Ogre::Vector3 normal;

	// Cached data for finding the barycentric coordinates for a point on the triangle plane.
	float aInv, bInv, cInv, dInv;
	unsigned short equationIndex1, equationIndex2;

	Ogre::Vector3 edgePseudoNormals[3];
	Ogre::Vector3 vertexPseudoNormals[3];

	bool dirtyEdgeNormals[3];

	bool degenerated;

	Ogre::Vector3 getBarycentricCoordinates(const Ogre::Vector3& pointOntrianglePlane) const
	{
		float b1 = pointOntrianglePlane[equationIndex1]-p1[equationIndex1];
		float b2 = pointOntrianglePlane[equationIndex2]-p1[equationIndex2];
		Ogre::Vector3 barycentricCoords;
		barycentricCoords.x = aInv*b1 + bInv*b2;
		barycentricCoords.y = cInv*b1 + dInv*b2;
		barycentricCoords.z = 1.0f - (barycentricCoords.x+barycentricCoords.y);
		return barycentricCoords;
	}
};
