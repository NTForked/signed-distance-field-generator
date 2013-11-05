/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "OgreMath/OgreVector2.h"
#include <string>
#include <sstream>
#include <algorithm>
#include "Prerequisites.h"
#include <unordered_set>
#include "Vector1i.h"

class Vector2i
{
public:
	int x,y;

	Vector2i() : x(0), y(0) {}
	Vector2i(int v) : x(v), y(v) {}
	Vector2i(const Ogre::Vector2 &ogreVec) : x((int)ogreVec.x), y((int)ogreVec.y) {}
	Vector2i(int _x, int _y) : x(_x), y(_y) {}

	typedef Ogre::Vector2 FloatVecType;

	inline Vector2i& operator = ( const Vector2i& rkVector )
	{
		x = rkVector.x;
		y = rkVector.y;

		return *this;
	}

	inline Vector2i& operator = ( const int iScaler )
	{
		x = iScaler;
		y = iScaler;

		return *this;
	}

	inline bool operator == ( const Vector2i& rkVector ) const
	{
		return ( x == rkVector.x && y == rkVector.y);
	}

	inline bool operator != ( const Vector2i& rkVector ) const
	{
		return ( x != rkVector.x || y != rkVector.y);
	}

	// arithmetic operations
	inline Vector2i operator + ( const Vector2i& rkVector ) const
	{
		return Vector2i(
			x + rkVector.x,
			y + rkVector.y);
	}

	inline Vector2i operator - ( const Vector2i& rkVector ) const
	{
		return Vector2i(
			x - rkVector.x,
			y - rkVector.y);
	}

	inline Vector2i operator * ( const int fScalar ) const
	{
		return Vector2i(
			x * fScalar,
			y * fScalar);
	}

	inline Vector2i operator * ( const Vector2i& rhs) const
	{
		return Vector2i(
			x * rhs.x,
			y * rhs.y);
	}

	inline bool operator < ( const Vector2i& rhs ) const
	{
		if(x < rhs.x && y <= rhs.y) return true;
		if(y < rhs.x && y <= rhs.y && x <= rhs.x) return true;
		return false;
	}

	inline int& operator [] ( const size_t i )
	{
		vAssert( i <  (size_t)dimension);
		return *(&x+i);
	}

	inline int operator [] ( const size_t i ) const
	{
		vAssert( i <  (size_t)dimension);
		return *(&x+i);
	}

	inline Vector2i reverse () const
		{
			return Vector2i(
				-x,
				-y);
		}

	inline int squaredLength () const
	{
		return x * x + y * y;
	}

	inline float length() const
	{
		return sqrtf((float)squaredLength());
	}

	inline Vector2i half () const
	{
		return Vector2i(
			x>>1,
			y>>1);
	}

	inline Vector2i doubleVec () const
	{
		return Vector2i(
			x<<1,
			y<<1);
	}

	Ogre::Vector2 toOgreVec() const
	{
		return Ogre::Vector2((Ogre::Real)x,(Ogre::Real)y);
	}

	void toFloatArray(float* pArr) const
	{
		pArr[0]=(float)x;
		pArr[1]=(float)y;
	}

	static const unsigned int dimension = 2;
	static const unsigned int childCount = 4;
	static const unsigned int neighborCount = 8;

	inline Vector2i* positiveNeighborOffsets(Vector2i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector2i(((i&2)>>1), (i&1));
		return arr;
	}

	inline Vector2i* positiveNeighbors(Vector2i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector2i(x+((i&2)>>1), y+(i&1));
		return arr;
	}

	inline Vector2i positiveNeighbor(const int& i) const
	{
		return Vector2i(x+((i&2)>>1), y+((i&1)));
	}

	Vector2i* neighbors(Vector2i *arr) const
	{
		static int lookup[]={0,-1,1};
		int i = 0;
		for(unsigned int _x = 0; _x < dimension; _x++)
			for(unsigned int _y = 0; _y < dimension; _y++)
				if(_x || _y)
					arr[i++]=Vector2i(x+lookup[_x],
					y+lookup[_y]);
		return arr;
	}

	static Vector2i* grid3(Vector2i *arr)
	{
		static int lookup[]={0,1,2};
		int i = 0;
		for(unsigned int _x = 0; _x < dimension; _x++)
			for(unsigned int _y = 0; _y < dimension; _y++)
					arr[i++] = Vector2i(lookup[_x], lookup[_y]);
		return arr;
	}

	int grid3Index() const
	{
		return 3 * x + y;
	}

	static const int numControlVecs = 5;
	inline static void getDistributedFloatsInCube(Ogre::Vector2* floatVecs)
	{
		static Ogre::Vector2 controlPoints[5] =
		{
			// edge mids
			Ogre::Vector2(0, 0.5f), 
			Ogre::Vector2(1, 0.5f), 
			Ogre::Vector2(0.5f, 0), 
			Ogre::Vector2(0.5f, 1), 
			// mid
			Ogre::Vector2(0.5f, 0.5f), 
		};
		memcpy((void*)floatVecs, controlPoints, 5 * sizeof(Ogre::Vector2));
	}

	static inline float spatialInterpolation(const float* cornerValues, float* weights)
	{
		return cornerValues[0] * (1 - weights[0]) * (1 - weights[1])
			+ cornerValues[1] * (1 - weights[0]) * weights[1]
			+ cornerValues[2] * weights[0] * (1 - weights[1])
			+ cornerValues[3] * weights[0] * weights[1];
	}

	static inline float spatialInterpolation(const float* cornerValues, const Ogre::Vector2& weights)
	{
		return cornerValues[0] * (1 - weights[0]) * (1 - weights[1])
			+ cornerValues[1] * (1 - weights[0]) * weights[1]
			+ cornerValues[2] * weights[0] * (1 - weights[1])
			+ cornerValues[3] * weights[0] * weights[1];
	}

	inline int getComponent(const unsigned int iIndex) const
	{
		vAssert(iIndex < (size_t)dimension)
		return *(&x+iIndex);
	}

	inline int maxComponent() const
	{
		return std::max<int>(std::abs(x), std::abs(y));
	}

	std::string toString() const
	{
		std::string str;
		std::string temp;
		std::stringstream ss;
		ss << "(" << x;
		ss >> temp;
		str += temp;
		ss.clear();
		ss << "," << y;
		ss >> temp;
		str += temp + std::string(")");
		return str;
	}

	typedef int scalarType;

	typedef Vector1i lowerDim;

	friend std::ostream& operator<< (std::ostream &out, Vector2i &pos) {
		out << pos.x << " " << pos.y;
		return out;
	}
};

namespace std
{

	template<>
	struct hash<Vector2i>
	{
	private:
	template <class T>
		inline static void hash_combine(std::size_t & seed, const T & v)
		{
		  std::hash<T> hasher;
		  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
	public:
		size_t operator()(const Vector2i &v) const
		{
			static const int size = 4;//sizeof(size_t)
			int iShift=(size*8)/Vector2i::dimension;
			size_t iMask=((size_t)1<<iShift)-1;
			return ((v.x)&(iMask)) | ((v.y&iMask)<<iShift);
		}
	};

}
