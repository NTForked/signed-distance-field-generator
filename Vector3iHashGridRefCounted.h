
#pragma once

#include "Prerequisites.h"
#include "Vector3i.h"

template<class T>
class Vector3iHashGridRefCounted
{
public:
	typedef std::pair<std::pair<Vector3i, int>, typename T> Bucket;
protected:
	T m_Dummy;
	std::vector<std::vector<Bucket> > m_Buckets;
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
		std::vector<std::vector<Bucket> > oldBuckets = m_Buckets;
		m_Buckets.clear();
		m_Buckets.resize(size);
		for (auto i = oldBuckets.begin(); i != oldBuckets.end(); i++)
		{
			for (auto i2 = i->begin(); i2 != i->end(); i2++)
			{
				m_Buckets[keyIndex(i2->first.first)].push_back(*i2);
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

	bool insert(const Vector3i& key, const T>& value)
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first.first == key)
				return false;
		}
		m_Buckets[index].push_back(std::make_pair(std::make_pair(key, 1), value));
		return true;
	}

	bool find(const Vector3i& key, T& value)
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first.first == key)
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
			if (i->first.first == key)
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
			if (i->first.first == key)
			{
				return i->second;
			}
		}
		vAssert(false);
		return m_Dummy;
	}
	T& lookupOrCreate(const Vector3i& key, bool& created)
	{
		int index = keyIndex(key);
		created = false;
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first.first == key)
			{
				return i->second;
			}
		}
		created = true;
		m_Buckets[index].push_back(std::make_pair(std::make_pair(key, 1), T()));
		return m_Buckets[index].back().second;
	}
	T& lookupOrCreate(const Vector3i& key)
	{
		bool created;
		return lookupOrCreate(key, created);
	}

	bool hasKey(const Vector3i& key) const
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first.first == key)
			{
				return true;
			}
		}
		return false;
	}

	inline T& operator[](const Vector3i& key)
	{
		bool dummy;
		return lookupOrCreate(key, dummy);
	}
	inline const T& operator[](const Vector3i& key) const
	{
		return lookup(key);
	}

	void unref(const Vector3i& key)
	{
		int index = keyIndex(key);
		for (auto i = m_Buckets[index].begin(); i != m_Buckets[index].end(); ++i)
		{
			if (i->first.first == key)
			{
				i->first.second--;
				if (i->first.second <= 0)
					m_Buckets[index].erase(i);
				return;
			}
		}
	}

	typedef typename std::vector<std::vector<Bucket> >::iterator iterator;
	typedef typename std::vector<std::vector<Bucket> >::const_iterator const_iterator;
	iterator begin() { return m_Buckets.begin(); }
	const_iterator begin() const { return m_Buckets.begin(); }
	iterator end() { return m_Buckets.end(); }
	const_iterator end() const { return m_Buckets.end(); }
};