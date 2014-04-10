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

	inline Vector3i operator % (int value) const
	{
		return Vector3i(
			x % value,
			y % value,
			z % value);
	}

	inline bool operator < ( const Vector3i& rhs ) const
	{
		if (x < rhs.x) return true;
		if (x > rhs.x) return false;
		if (y < rhs.y) return true;
		if (y > rhs.y) return false;
		return (z < rhs.z);
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

	static inline Vector3i fromBitMask(int bitmask)
	{
		return Vector3i((bitmask & 4) != 0, (bitmask & 2) != 0, bitmask & 1);
	}

	inline int toBitMask() const
	{
		return x * 4 + y * 2 + z;
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

	friend std::ostream& operator<< (std::ostream &out, const Vector3i &pos) {
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

	inline int computeZCurveIndex(const Vector3i& key) const
	{
		int cellId = interleaveMap(key.x >> 10) | (interleaveMap(key.y >> 10) << 1) | (interleaveMap(key.z >> 10) << 2);
		cellId = cellId << 30 | interleaveMap(key.x & 0x3FF) | (interleaveMap(key.y & 0x3FF) << 1) | (interleaveMap(key.z & 0x3FF) << 2);
		return cellId;
	}

	inline int interleaveMap(const int& i) const
	{
		static const int _interleaveMap[1024] =
		{
			0, 1, 8, 9, 64, 65, 72, 73, 512, 513, 520, 521, 576, 577, 584, 585,
			4096, 4097, 4104, 4105, 4160, 4161, 4168, 4169, 4608, 4609, 4616, 4617, 4672, 4673, 4680, 4681,
			32768, 32769, 32776, 32777, 32832, 32833, 32840, 32841, 33280, 33281, 33288, 33289, 33344, 33345, 33352, 33353,
			36864, 36865, 36872, 36873, 36928, 36929, 36936, 36937, 37376, 37377, 37384, 37385, 37440, 37441, 37448, 37449,
			262144, 262145, 262152, 262153, 262208, 262209, 262216, 262217, 262656, 262657, 262664, 262665, 262720, 262721, 262728, 262729,
			266240, 266241, 266248, 266249, 266304, 266305, 266312, 266313, 266752, 266753, 266760, 266761, 266816, 266817, 266824, 266825,
			294912, 294913, 294920, 294921, 294976, 294977, 294984, 294985, 295424, 295425, 295432, 295433, 295488, 295489, 295496, 295497,
			299008, 299009, 299016, 299017, 299072, 299073, 299080, 299081, 299520, 299521, 299528, 299529, 299584, 299585, 299592, 299593,
			2097152, 2097153, 2097160, 2097161, 2097216, 2097217, 2097224, 2097225, 2097664, 2097665, 2097672, 2097673, 2097728, 2097729, 2097736, 2097737,
			2101248, 2101249, 2101256, 2101257, 2101312, 2101313, 2101320, 2101321, 2101760, 2101761, 2101768, 2101769, 2101824, 2101825, 2101832, 2101833,
			2129920, 2129921, 2129928, 2129929, 2129984, 2129985, 2129992, 2129993, 2130432, 2130433, 2130440, 2130441, 2130496, 2130497, 2130504, 2130505,
			2134016, 2134017, 2134024, 2134025, 2134080, 2134081, 2134088, 2134089, 2134528, 2134529, 2134536, 2134537, 2134592, 2134593, 2134600, 2134601,
			2359296, 2359297, 2359304, 2359305, 2359360, 2359361, 2359368, 2359369, 2359808, 2359809, 2359816, 2359817, 2359872, 2359873, 2359880, 2359881,
			2363392, 2363393, 2363400, 2363401, 2363456, 2363457, 2363464, 2363465, 2363904, 2363905, 2363912, 2363913, 2363968, 2363969, 2363976, 2363977,
			2392064, 2392065, 2392072, 2392073, 2392128, 2392129, 2392136, 2392137, 2392576, 2392577, 2392584, 2392585, 2392640, 2392641, 2392648, 2392649,
			2396160, 2396161, 2396168, 2396169, 2396224, 2396225, 2396232, 2396233, 2396672, 2396673, 2396680, 2396681, 2396736, 2396737, 2396744, 2396745,
			16777216, 16777217, 16777224, 16777225, 16777280, 16777281, 16777288, 16777289, 16777728, 16777729, 16777736, 16777737, 16777792, 16777793, 16777800, 16777801,
			16781312, 16781313, 16781320, 16781321, 16781376, 16781377, 16781384, 16781385, 16781824, 16781825, 16781832, 16781833, 16781888, 16781889, 16781896, 16781897,
			16809984, 16809985, 16809992, 16809993, 16810048, 16810049, 16810056, 16810057, 16810496, 16810497, 16810504, 16810505, 16810560, 16810561, 16810568, 16810569,
			16814080, 16814081, 16814088, 16814089, 16814144, 16814145, 16814152, 16814153, 16814592, 16814593, 16814600, 16814601, 16814656, 16814657, 16814664, 16814665,
			17039360, 17039361, 17039368, 17039369, 17039424, 17039425, 17039432, 17039433, 17039872, 17039873, 17039880, 17039881, 17039936, 17039937, 17039944, 17039945,
			17043456, 17043457, 17043464, 17043465, 17043520, 17043521, 17043528, 17043529, 17043968, 17043969, 17043976, 17043977, 17044032, 17044033, 17044040, 17044041,
			17072128, 17072129, 17072136, 17072137, 17072192, 17072193, 17072200, 17072201, 17072640, 17072641, 17072648, 17072649, 17072704, 17072705, 17072712, 17072713,
			17076224, 17076225, 17076232, 17076233, 17076288, 17076289, 17076296, 17076297, 17076736, 17076737, 17076744, 17076745, 17076800, 17076801, 17076808, 17076809,
			18874368, 18874369, 18874376, 18874377, 18874432, 18874433, 18874440, 18874441, 18874880, 18874881, 18874888, 18874889, 18874944, 18874945, 18874952, 18874953,
			18878464, 18878465, 18878472, 18878473, 18878528, 18878529, 18878536, 18878537, 18878976, 18878977, 18878984, 18878985, 18879040, 18879041, 18879048, 18879049,
			18907136, 18907137, 18907144, 18907145, 18907200, 18907201, 18907208, 18907209, 18907648, 18907649, 18907656, 18907657, 18907712, 18907713, 18907720, 18907721,
			18911232, 18911233, 18911240, 18911241, 18911296, 18911297, 18911304, 18911305, 18911744, 18911745, 18911752, 18911753, 18911808, 18911809, 18911816, 18911817,
			19136512, 19136513, 19136520, 19136521, 19136576, 19136577, 19136584, 19136585, 19137024, 19137025, 19137032, 19137033, 19137088, 19137089, 19137096, 19137097,
			19140608, 19140609, 19140616, 19140617, 19140672, 19140673, 19140680, 19140681, 19141120, 19141121, 19141128, 19141129, 19141184, 19141185, 19141192, 19141193,
			19169280, 19169281, 19169288, 19169289, 19169344, 19169345, 19169352, 19169353, 19169792, 19169793, 19169800, 19169801, 19169856, 19169857, 19169864, 19169865,
			19173376, 19173377, 19173384, 19173385, 19173440, 19173441, 19173448, 19173449, 19173888, 19173889, 19173896, 19173897, 19173952, 19173953, 19173960, 19173961,
			134217728, 134217729, 134217736, 134217737, 134217792, 134217793, 134217800, 134217801, 134218240, 134218241, 134218248, 134218249, 134218304, 134218305, 134218312, 134218313,
			134221824, 134221825, 134221832, 134221833, 134221888, 134221889, 134221896, 134221897, 134222336, 134222337, 134222344, 134222345, 134222400, 134222401, 134222408, 134222409,
			134250496, 134250497, 134250504, 134250505, 134250560, 134250561, 134250568, 134250569, 134251008, 134251009, 134251016, 134251017, 134251072, 134251073, 134251080, 134251081,
			134254592, 134254593, 134254600, 134254601, 134254656, 134254657, 134254664, 134254665, 134255104, 134255105, 134255112, 134255113, 134255168, 134255169, 134255176, 134255177,
			134479872, 134479873, 134479880, 134479881, 134479936, 134479937, 134479944, 134479945, 134480384, 134480385, 134480392, 134480393, 134480448, 134480449, 134480456, 134480457,
			134483968, 134483969, 134483976, 134483977, 134484032, 134484033, 134484040, 134484041, 134484480, 134484481, 134484488, 134484489, 134484544, 134484545, 134484552, 134484553,
			134512640, 134512641, 134512648, 134512649, 134512704, 134512705, 134512712, 134512713, 134513152, 134513153, 134513160, 134513161, 134513216, 134513217, 134513224, 134513225,
			134516736, 134516737, 134516744, 134516745, 134516800, 134516801, 134516808, 134516809, 134517248, 134517249, 134517256, 134517257, 134517312, 134517313, 134517320, 134517321,
			136314880, 136314881, 136314888, 136314889, 136314944, 136314945, 136314952, 136314953, 136315392, 136315393, 136315400, 136315401, 136315456, 136315457, 136315464, 136315465,
			136318976, 136318977, 136318984, 136318985, 136319040, 136319041, 136319048, 136319049, 136319488, 136319489, 136319496, 136319497, 136319552, 136319553, 136319560, 136319561,
			136347648, 136347649, 136347656, 136347657, 136347712, 136347713, 136347720, 136347721, 136348160, 136348161, 136348168, 136348169, 136348224, 136348225, 136348232, 136348233,
			136351744, 136351745, 136351752, 136351753, 136351808, 136351809, 136351816, 136351817, 136352256, 136352257, 136352264, 136352265, 136352320, 136352321, 136352328, 136352329,
			136577024, 136577025, 136577032, 136577033, 136577088, 136577089, 136577096, 136577097, 136577536, 136577537, 136577544, 136577545, 136577600, 136577601, 136577608, 136577609,
			136581120, 136581121, 136581128, 136581129, 136581184, 136581185, 136581192, 136581193, 136581632, 136581633, 136581640, 136581641, 136581696, 136581697, 136581704, 136581705,
			136609792, 136609793, 136609800, 136609801, 136609856, 136609857, 136609864, 136609865, 136610304, 136610305, 136610312, 136610313, 136610368, 136610369, 136610376, 136610377,
			136613888, 136613889, 136613896, 136613897, 136613952, 136613953, 136613960, 136613961, 136614400, 136614401, 136614408, 136614409, 136614464, 136614465, 136614472, 136614473,
			150994944, 150994945, 150994952, 150994953, 150995008, 150995009, 150995016, 150995017, 150995456, 150995457, 150995464, 150995465, 150995520, 150995521, 150995528, 150995529,
			150999040, 150999041, 150999048, 150999049, 150999104, 150999105, 150999112, 150999113, 150999552, 150999553, 150999560, 150999561, 150999616, 150999617, 150999624, 150999625,
			151027712, 151027713, 151027720, 151027721, 151027776, 151027777, 151027784, 151027785, 151028224, 151028225, 151028232, 151028233, 151028288, 151028289, 151028296, 151028297,
			151031808, 151031809, 151031816, 151031817, 151031872, 151031873, 151031880, 151031881, 151032320, 151032321, 151032328, 151032329, 151032384, 151032385, 151032392, 151032393,
			151257088, 151257089, 151257096, 151257097, 151257152, 151257153, 151257160, 151257161, 151257600, 151257601, 151257608, 151257609, 151257664, 151257665, 151257672, 151257673,
			151261184, 151261185, 151261192, 151261193, 151261248, 151261249, 151261256, 151261257, 151261696, 151261697, 151261704, 151261705, 151261760, 151261761, 151261768, 151261769,
			151289856, 151289857, 151289864, 151289865, 151289920, 151289921, 151289928, 151289929, 151290368, 151290369, 151290376, 151290377, 151290432, 151290433, 151290440, 151290441,
			151293952, 151293953, 151293960, 151293961, 151294016, 151294017, 151294024, 151294025, 151294464, 151294465, 151294472, 151294473, 151294528, 151294529, 151294536, 151294537,
			153092096, 153092097, 153092104, 153092105, 153092160, 153092161, 153092168, 153092169, 153092608, 153092609, 153092616, 153092617, 153092672, 153092673, 153092680, 153092681,
			153096192, 153096193, 153096200, 153096201, 153096256, 153096257, 153096264, 153096265, 153096704, 153096705, 153096712, 153096713, 153096768, 153096769, 153096776, 153096777,
			153124864, 153124865, 153124872, 153124873, 153124928, 153124929, 153124936, 153124937, 153125376, 153125377, 153125384, 153125385, 153125440, 153125441, 153125448, 153125449,
			153128960, 153128961, 153128968, 153128969, 153129024, 153129025, 153129032, 153129033, 153129472, 153129473, 153129480, 153129481, 153129536, 153129537, 153129544, 153129545,
			153354240, 153354241, 153354248, 153354249, 153354304, 153354305, 153354312, 153354313, 153354752, 153354753, 153354760, 153354761, 153354816, 153354817, 153354824, 153354825,
			153358336, 153358337, 153358344, 153358345, 153358400, 153358401, 153358408, 153358409, 153358848, 153358849, 153358856, 153358857, 153358912, 153358913, 153358920, 153358921,
			153387008, 153387009, 153387016, 153387017, 153387072, 153387073, 153387080, 153387081, 153387520, 153387521, 153387528, 153387529, 153387584, 153387585, 153387592, 153387593,
			153391104, 153391105, 153391112, 153391113, 153391168, 153391169, 153391176, 153391177, 153391616, 153391617, 153391624, 153391625, 153391680, 153391681, 153391688, 153391689
		};
		return _interleaveMap[i];
	}

	unsigned int keyIndex(const Vector3i& v) const
	{
		return ((PRIME1*v.x) ^ (PRIME2*v.y) ^ (PRIME3*v.z)) % m_Buckets.size();	// computeZCurveIndex(v) % m_Buckets.size();
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

	bool find(const Vector3i& key, T& value)
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first == key)
			{
				value = i->second;
				return true;
			}
		}
		return false;
	}

	T& lookup(const Vector3i& key)
	{
		int index = keyIndex(key);
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
	const T& lookup(const Vector3i& key) const
	{
		int index = keyIndex(key);
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

	T& emplace(const Vector3i& key)
	{
		int index = keyIndex(key);
		m_Buckets[index].emplace_back();
		m_Buckets[index].back().first = key;
		return m_Buckets[index].back().second;
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
		return hasKey(keyIndex(key), key);
	}

	inline T& operator[](const Vector3i& key)
	{
		bool dummy;
		return lookupOrCreate(keyIndex(key), key, dummy);
	}
	inline const T& operator[](const Vector3i& key) const
	{
		return lookup(key);
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