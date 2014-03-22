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

#include "OgreMath/OgreVector3.h"
#include <string>
#include <sstream>
#include <algorithm>
#include "Prerequisites.h"
#include <unordered_set>

class Vector3i
{
public:
	int x,y,z;

	Vector3i() : x(0), y(0), z(0) {}
	Vector3i(int v) : x(v), y(v), z(v) {}
	Vector3i(const Ogre::Vector3 &ogreVec) : x((int)ogreVec.x), y((int)ogreVec.y), z((int)ogreVec.z) {}
	Vector3i(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

	typedef Ogre::Vector3 FloatVecType;

	void set(int x, int y, int z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	inline Vector3i& operator = ( const Vector3i& rkVector )
	{
		x = rkVector.x;
		y = rkVector.y;
		z = rkVector.z;

		return *this;
	}

	inline Vector3i& operator = ( const int iScaler )
	{
		x = iScaler;
		y = iScaler;
		z = iScaler;

		return *this;
	}

	inline bool operator == ( const Vector3i& rkVector ) const
	{
		return ( x == rkVector.x && y == rkVector.y && z == rkVector.z );
	}

	inline bool operator != ( const Vector3i& rkVector ) const
	{
		return ( x != rkVector.x || y != rkVector.y || z != rkVector.z );
	}

	// arithmetic operations
	inline Vector3i operator + ( const Vector3i& rkVector ) const
	{
		return Vector3i(
			x + rkVector.x,
			y + rkVector.y,
			z + rkVector.z);
	}

	inline Vector3i operator - ( const Vector3i& rkVector ) const
	{
		return Vector3i(
			x - rkVector.x,
			y - rkVector.y,
			z - rkVector.z);
	}

	inline Vector3i operator * ( const int fScalar ) const
	{
		return Vector3i(
			x * fScalar,
			y * fScalar,
			z * fScalar);
	}

	inline Vector3i operator * ( const Vector3i& rhs) const
	{
		return Vector3i(
			x * rhs.x,
			y * rhs.y,
			z * rhs.z);
	}

	inline bool operator < ( const Vector3i& rhs ) const
	{
		if(x < rhs.x && y <= rhs.y && z <= rhs.z) return true;
		if(y < rhs.x && y <= rhs.y && x <= rhs.x) return true;
		if(z < rhs.x && y <= rhs.y && y <= rhs.y) return true;
		return false;
	}

	inline int& operator [] ( const size_t i )
	{
		vAssert( i < (size_t)dimension );
		return *(&x+i);
	}

	inline int operator [] ( const size_t i ) const
	{
		vAssert( i < (size_t)dimension );
		return *(&x+i);
	}

	inline Vector3i reverse () const
	{
		return Vector3i(
			-x,
			-y,
			-z);
	}

	inline int squaredLength () const
	{
		return x * x + y * y + z * z;
	}

	inline float length() const
	{
		return sqrtf((float)squaredLength());
	}

	inline Vector3i half () const
	{
		return Vector3i(
			x>>1,
			y>>1,
			z>>1);
	}

	inline Vector3i doubleVec () const
	{
		return Vector3i(
			x<<1,
			y<<1,
			z<<1);
	}

#include <memory>

	static const int numControlVecs = 19;
	inline static void getDistributedFloatsInCube(Ogre::Vector3* floatVecs)
	{
		static Ogre::Vector3 controlPoints[19] =
		{
			// edge mids
			Ogre::Vector3(0, 0.5f, 0), 
			Ogre::Vector3(0.5f, 0, 0), 
			Ogre::Vector3(0.5f, 1, 0), 
			Ogre::Vector3(1, 0.5f, 0), 
			Ogre::Vector3(0, 0, 0.5f), 
			Ogre::Vector3(0, 1, 0.5f), 
			Ogre::Vector3(1, 0, 0.5f), 
			Ogre::Vector3(1, 1, 0.5f), 
			Ogre::Vector3(0, 0.5f, 1), 
			Ogre::Vector3(0.5f, 0, 1), 
			Ogre::Vector3(0.5f, 1, 1), 
			Ogre::Vector3(1, 0.5f, 1),
			// faces
			Ogre::Vector3(0, 0.5f, 0.5f),
			Ogre::Vector3(0.5f, 0.5f, 0),
			Ogre::Vector3(0.5f, 0, 0.5f),
			Ogre::Vector3(1, 0.5f, 0.5f),
			Ogre::Vector3(0.5f, 0.5f, 1),
			Ogre::Vector3(0.5f, 1, 0.5f),
			// mid
			Ogre::Vector3(0.5f, 0.5f, 0.5f),
		};
		memcpy((void*)floatVecs, controlPoints, 19 * sizeof(Ogre::Vector3));
		/*floatVecs[0] = Ogre::Vector3(0.824634f, 0.208382f, 0.751779f);
		floatVecs[1] = Ogre::Vector3(0.875094f, 0.201589f, 0.768007f);
		floatVecs[2] = Ogre::Vector3(0.605887f, 0.276827f, 0.830701f);
		floatVecs[3] = Ogre::Vector3(0.537776f, 0.178032f, 0.346534f);
		floatVecs[4] = Ogre::Vector3(0.537505f, 0.250706f, 0.322799f);
		floatVecs[5] = Ogre::Vector3(0.897169f, 0.866005f, 0.894305f);
		floatVecs[6] = Ogre::Vector3(0.22609f, 0.874156f, 0.871911f);
		floatVecs[7] = Ogre::Vector3(0.884888f, 0.876474f, 0.680671f);*/
	}

	Ogre::Vector3 toOgreVec() const
	{
		return Ogre::Vector3((Ogre::Real)x,(Ogre::Real)y,(Ogre::Real)z);
	}

	void toFloatArray(float* pArr) const
	{
		pArr[0]=(float)x;
		pArr[1]=(float)y;
		pArr[2]=(float)z;
	}

	static const unsigned int dimension = 3;
	static const unsigned int childCount = 8;
	static const unsigned int neighborCount = 26;

	inline Vector3i* positiveNeighborOffsets(Vector3i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector3i(((i&4)>>2), ((i&2)>>1), (i&1));
		return arr;
	}

	inline Vector3i* positiveNeighbors(Vector3i arr[8]) const
	{
		for (unsigned int i = 0; i < childCount; i++)
			arr[i]=Vector3i(x+((i&4)>>2), y+((i&2)>>1), z+(i&1));
		return arr;
	}

	inline Vector3i positiveNeighbor(const int& i) const
	{
		return Vector3i(x+((i&4)>>2), y+((i&2)>>1), z+(i&1));
	}

	Vector3i* neighbors(Vector3i *arr) const
	{
		static int lookup[]={0,-1,1};
		int i = -2;
		for(unsigned int _x = 0; _x < dimension; _x++)
			for(unsigned int _y = 0; _y < dimension; _y++)
				for(unsigned int _z = 0; _z < dimension; _z++)
					if(++i>=0)//mask out the very first iteration
						arr[i]=Vector3i(x+lookup[_x],
						y+lookup[_y],
						z+lookup[_z]);
		return arr;
	}

	static Vector3i* grid3(Vector3i *arr)
	{
		static int lookup[]={0,1,2};
		int i = 0;
		for(unsigned int _x = 0; _x < dimension; _x++)
			for(unsigned int _y = 0; _y < dimension; _y++)
				for(unsigned int _z = 0; _z < dimension; _z++)
					arr[i++] = Vector3i(lookup[_x], lookup[_y], lookup[_z]);
		return arr;
	}

	int grid3Index() const
	{
		return 9 * x + 3 * y + z;
	}

	Vector3i* directNeighbors(Vector3i* arr) const
	{
		arr[0] = *this + Vector3i(-1, 0, 0);
		arr[1] = *this + Vector3i(1, 0, 0);
		arr[2] = *this + Vector3i(0, -1, 0);
		arr[3] = *this + Vector3i(0, 1, 0);
		arr[4] = *this + Vector3i(0, 0, -1);
		arr[5] = *this + Vector3i(0, 0, 1);
		return arr;
	}

	static inline float spatialInterpolation(const float* cornerValues, float* weights)
	{
		return cornerValues[0] * (1 - weights[0]) * (1 - weights[1]) * (1 - weights[2])
			+ cornerValues[1] * (1 - weights[0]) * (1 - weights[1]) * weights[2]
			+ cornerValues[2] * (1 - weights[0]) * weights[1] * (1 - weights[2])
			+ cornerValues[3] * (1 - weights[0]) * weights[1] * weights[2]
			+ cornerValues[4] * weights[0] * (1 - weights[1]) * (1 - weights[2])
			+ cornerValues[5] * weights[0] * (1 - weights[1]) * weights[2]
			+ cornerValues[6] * weights[0] * weights[1] * (1 - weights[2])
			+ cornerValues[7] * weights[0] * weights[1] * weights[2];
	}

	static inline float spatialInterpolation(const float* cornerValues, const Ogre::Vector3& weights)
	{
		return cornerValues[0] * (1 - weights[0]) * (1 - weights[1]) * (1 - weights[2])
			+ cornerValues[1] * (1 - weights[0]) * (1 - weights[1]) * weights[2]
			+ cornerValues[2] * (1 - weights[0]) * weights[1] * (1 - weights[2])
			+ cornerValues[3] * (1 - weights[0]) * weights[1] * weights[2]
			+ cornerValues[4] * weights[0] * (1 - weights[1]) * (1 - weights[2])
			+ cornerValues[5] * weights[0] * (1 - weights[1]) * weights[2]
			+ cornerValues[6] * weights[0] * weights[1] * (1 - weights[2])
			+ cornerValues[7] * weights[0] * weights[1] * weights[2];
	}

	inline int getComponent(const unsigned int& iIndex) const
	{
		vAssert(iIndex < (size_t)dimension)
		return *(&x+iIndex);
	}

	inline int maxComponent() const
	{
		return std::max<int>(std::max<int>(abs(x), abs(y)), abs(z));
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
		str += temp;
		ss.clear();
		ss << "," << z;
		ss >> temp;
		str += temp + std::string(")");
		return str;
	}

	typedef int scalarType;

	friend std::ostream& operator<< (std::ostream &out, Vector3i &pos) {
		out << pos.x << " " << pos.y << " " << pos.z;
		return out;
	}
};

// boost::unordered_map: 0.775 seconds without assertions
template<class T>
class Vector3iHashGrid
{
protected:
	T m_Dummy;
	std::vector<std::vector<std::pair<Vector3i, T> > > m_Buckets;
	static const size_t PRIME1 = 73856093;
	static const size_t PRIME2 = 19349663;
	static const size_t PRIME3 = 83492791;

public:
	Vector3iHashGrid()
	{
		rehash(50000);
	}
	void rehash(unsigned int size)
	{
		std::vector<std::vector<std::pair<Vector3i, T> > > oldBuckets = m_Buckets;
		m_Buckets.clear();
		m_Buckets.resize(size);
		for (auto i = oldBuckets.begin(); i != oldBuckets.end(); i++)
		{
			for (auto i2 = i->begin(); i2 != i->end(); i2++)
			{
				m_Buckets[keyIndex(i2->first)].push_back(*i2);
			}
		}
		for (auto i = m_Buckets.begin(); i != m_Buckets.end(); i++)
		{
			i->reserve(8);
		}
	}
	unsigned int keyIndex(const Vector3i& v) const
	{
		return ((PRIME1*v.x) ^ (PRIME2*v.y) ^ (PRIME3*v.z)) % m_Buckets.size();
	}

	bool insert(unsigned int index, const std::pair<Vector3i, T>& keyValue)
	{
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == keyValue.first)
				return false;
		}
		m_Buckets[index].push_back(keyValue);
		return true;
	}
	bool insert(const std::pair<Vector3i, T>& keyValue)
	{
		return insert(keyIndex(keyValue.first), keyValue);
	}

	void insertUnsafe(unsigned int index, const std::pair<Vector3i, T>& keyValue)
	{
		m_Buckets[index].push_back(keyValue);
	}
	void insertUnsafe(const std::pair<Vector3i, T>& keyValue)
	{
		insertUnsafe(keyIndex(keyValue.first), keyValue);
	}

	T& lookup(unsigned int index, const Vector3i& key)
	{
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				return i->second;
			}
		}
		vAssert(false);
		return m_Dummy;
	}
	const T& lookup(unsigned int index, const Vector3i& key) const
	{
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				return i->second;
			}
		}
		vAssert(false);
		return m_Dummy;
	}
	T& lookupOrCreate(unsigned int index, const Vector3i& key, bool& created)
	{
		created = false;
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				return i->second;
			}
		}
		created = true;
		m_Buckets[index].push_back(std::make_pair(key, T()));
		return m_Buckets[index].back().second;
	}
	T& lookupOrCreate(unsigned int index, const Vector3i& key)
	{
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				return i->second;
			}
		}
		m_Buckets[index].push_back(std::make_pair(key, T()));
		return m_Buckets[index].back().second;
	}
	T& lookupOrCreate(const Vector3i& key, bool& created)
	{
		return lookupOrCreate(keyIndex(key), key, created);
	}
	T& lookupOrCreate(const Vector3i& key)
	{
		return lookupOrCreate(keyIndex(key), key);
	}

	bool hasKey(unsigned int index, const Vector3i& key) const
	{
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				return true;
			}
		}
		return false;
	}
	bool hasKey(const Vector3i& key) const
	{
		return hasKey(keyIndex(key));
	}

	inline T& operator[](const Vector3i& key)
	{
		bool dummy;
		return lookupOrCreate(keyIndex(key), key, dummy);
	}
	inline const T& operator[](const Vector3i& key) const
	{
		return lookup(keyIndex(key), key);
	}

	void remove(const Vector3i& key)
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				m_Buckets[index].erase(i);
				return;
			}
		}
	}

	typedef typename std::vector<typename std::vector<typename std::pair<typename Vector3i, typename T> > >::iterator iterator;
	typedef typename std::vector<typename std::vector<typename std::pair<typename Vector3i, typename T> > >::const_iterator const_iterator;
	iterator begin() { return m_Buckets.begin(); }
	const_iterator begin() const { return m_Buckets.begin(); }
	iterator end() { return m_Buckets.end(); }
	const_iterator end() const { return m_Buckets.end(); }
};

namespace std
{
	template<>
	struct hash<Vector3i>
	{
	private:
		static const size_t PRIME1 = 73856093;
		static const size_t PRIME2 = 19349663;
		static const size_t PRIME3 = 83492791;
	public:
		size_t operator()(const Vector3i &v) const
		{
			/*static const int size = 4;//sizeof(size_t)
			int iShift=(size*8)/Vector3i::dimension;
			size_t iMask=((size_t)1<<iShift)-1;
			return ((v.x)&(iMask)) | ((v.y&iMask)<<iShift) | ((v.z&iMask)<<(iShift<<1));*/
			return ((PRIME1*v.x) ^ (PRIME2*v.y) ^ (PRIME3*v.z));
		}
	};
}