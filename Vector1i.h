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

#include <string>
#include <sstream>
#include <algorithm>
#include "Prerequisites.h"
#include <unordered_set>

class Vector1i
{
public:
	int x;

	typedef Vector1i FloatVecType;

	Vector1i() : x(0) {}
	Vector1i(int v) : x(v) {}
	Vector1i(const Ogre::Real &ogreVec) : x((int)ogreVec) {}
	Vector1i(int _x, int _y) : x(_x) {}

	inline Vector1i& operator = ( const Vector1i& rkVector )
	{
		x = rkVector.x;
		return *this;
	}

	inline Vector1i& operator = ( const int iScaler )
	{
		x = iScaler;
		return *this;
	}

	inline bool operator == ( const Vector1i& rkVector ) const
	{
		return ( x == rkVector.x);
	}

	inline bool operator != ( const Vector1i& rkVector ) const
	{
		return ( x != rkVector.x);
	}

	// arithmetic operations
	inline Vector1i operator + ( const Vector1i& rkVector ) const
	{
		return Vector1i(x + rkVector.x);
	}

	inline Vector1i operator - ( const Vector1i& rkVector ) const
	{
		return Vector1i(x - rkVector.x);
	}

	inline Vector1i operator * ( const int fScalar ) const
	{
		return Vector1i(x * fScalar);
	}

	inline Vector1i operator * ( const Vector1i& rhs) const
	{
		return Vector1i(x * rhs.x);
	}

	inline bool operator < ( const Vector1i& rhs ) const
	{
		if(x < rhs.x) return true;
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

	inline int squaredLength () const
	{
		return x * x;
	}

	inline float length() const
	{
		return (float)std::abs((double)x);
	}

	inline Vector1i half () const
	{
		return Vector1i(x>>1);
	}

	inline Vector1i doubleVec () const
	{
		return Vector1i(x<<1);
	}

	Ogre::Real toOgreVec() const
	{
		return (Ogre::Real)x;
	}

	void toFloatArray(float* pArr) const
	{
		pArr[0]=(float)x;
	}

	static const unsigned int dimension = 1;
	static const unsigned int childCount = 2;
	static const unsigned int neighborCount = 2;

	inline Vector1i* positiveNeighborOffsets(Vector1i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector1i((int)i&1);
		return arr;
	}

	inline Vector1i* positiveNeighbors(Vector1i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector1i(x+(int)(i&1));
		return arr;
	}

	Vector1i* neighbors(Vector1i *arr) const
	{
		static int lookup[]={0,-1,1};
		int i = 0;
		for(unsigned int _x = 0; _x < dimension; _x++)
			if(_x)
				arr[i++]=Vector1i(x+lookup[_x]);
		return arr;
	}

	inline int getComponent(const unsigned int iIndex) const
	{
		vAssert(iIndex < (size_t)dimension)
		return *(&x+iIndex);
	}

	inline int maxComponent() const
	{
		return x;
	}

	std::string toString() const
	{
		std::string str;
		std::string temp;
		std::stringstream ss;
		ss << "(" << x;
		ss >> temp;
		str += temp + std::string(")");
		return str;
	}

	typedef int scalarType;
	typedef Vector1i lowerDim;


	friend std::ostream& operator<< (std::ostream &out, Vector1i &pos) {
		out << pos.x;
		return out;
	}
};

namespace std
{

	template<>
	struct hash<Vector1i>
	{
	private:
	template <class T>
		inline static void hash_combine(std::size_t & seed, const T & v)
		{
		  std::hash<T> hasher;
		  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
	public:
		size_t operator()(const Vector1i &v) const
		{
			static const int size = 4;
			size_t iMask=((size_t)1<<size)-1;
			return v.x&iMask;
		}
	};

}
