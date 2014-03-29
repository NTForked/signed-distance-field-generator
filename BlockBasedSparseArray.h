
#pragma once

#include "Vector3i.h"

template<class T>
class BlockBasedSparseArray
{
protected:
	T*** m_Array;
	Vector3i m_MinPos;
	int m_Level0Size;
	int m_Level0SizeSquared;
	int m_Level0SizeCubed;
	int m_Level1Size;
	int m_Level1SizeSquared;
	int m_Level0SizeCubed;

	void computeArrayIndices(const Vector3i& position, int& level0Index, int& level1Index)
	{
		Vector3i indexPos = position - m_MinPos;
		Vector3i level0IndexPos = indexPos % m_Level0Size;
		Vector3i level1IndexPos = indexPos / m_Level0Size;
		level0Index = level0IndexPos.x * m_Level0SizeSquared + level0IndexPos.y * m_Level0Size + level0Index.z;
		level1Index = level1IndexPos.x * m_Level1SizeSquared + level1IndexPos.y * m_Level1Size + level1Index.z;
	}

public:
	BlockBasedSparseArray(const Vector3i& min, int level0Size, int level1Size)
	{
		m_MinPos = min;
		m_Level0Size = level0Size;
		m_Level1Size = level1Size;
		m_Level0SizeSquared = m_Level0Size * m_Level0Size;
		m_Level1SizeSquared = m_Level1Size * m_Level1Size;
		m_Level0SizeCubed = m_Level0SizeSquared * m_Level0Size;
		m_Level1SizeCubed = m_Level1SizeSquared * m_Level1Size;

		m_Array = new T**[m_Level1SizeCubed];
		for (int i = 0; i < m_Level1SizeCubed; i++)
		{
			m_Array[i] = nullptr;
		}
	}
	~BlockBasedSparseArray()
	{
		clear();
		delete[] m_Array;
	}

	bool insert(const Vector3i& position, const T& value)
	{
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		if (m_Array[index1])
		{
			if (m_Array[index1][index0]) return false;
			m_Array[index1][index0] = new T(value);
		}
		m_Array[index1] = new T**[m_Level0SizeCubed];
		m_Array[index1][index0] = new T(value);
		return true;
	}

	bool find(const Vector3i& position, T& value)
	{
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		if (m_Array[index1] && m_Array[index1][index0])
		{
			value = *m_Array[index1][index0];
			return true;
		}
		return false;
	}

	T& lookup(const Vector3i& position)
	{
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		return m_Array[index1][index0];
	}
	const T& lookup(const Vector3i& position) const
	{
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		return m_Array[index1][index0];
	}
	T& lookupOrCreate(const Vector3i& position, bool& created)
	{
		created = false;
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		if (m_Array[index1])
		{
			if (!m_Array[index1][index0])
			{
				m_Array[index1][index0] = new T();
				created = true;
			}
		}
		else
		{
			m_Array[index1] = new T**[m_Level0SizeCubed];
			m_Array[index1][index0] = new T(value);
			created = true;
		}
		return *m_Array[index1][index0];
	}
	T& lookupOrCreate(const Vector3i& position)
	{
		bool created;
		return lookupOrCreate(key, created);
	}

	bool hasKey(const Vector3i& position) const
	{
		T value;
		return find(position, value);
	}

	inline T& operator[](const Vector3i& position)
	{
		return lookupOrCreate(key);
	}
	inline const T& operator[](const Vector3i& position) const
	{
		return lookup(key);
	}

	bool remove(const Vector3i& position)
	{
		int index0, index1;
		computeArrayIndices(position, index0, index1);
		if (m_Array[index1] && m_Array[index1][index0])
		{
			delete m_Array[index1][index0];
			m_Array[index1][index0] = nullptr;
			return true;
		}
		return false;
	}

	void clear()
	{
		for (int i = 0; i < m_Level1SizeCubed; i++)
		{
			if (m_Array[i])
			{
				for (int j = 0; j < m_Level1SizeCubed; j++)
				{
					if (m_Array[i][j]) delete m_Array[i][j];
				}
				delete[] m_Array[i];
				m_Array[i] = nullptr;
			}
		}
	}

	void freeUnusedLevel0Blocks()
	{
		for (int i = 0; i < m_Level1SizeCubed; i++)
		{
			if (m_Array[i])
			{
				bool allNull = true;
				for (int j = 0; j < m_Level1SizeCubed; j++)
				{
					if (m_Array[i][j]) allNull = false;
				}
				if (allNull)
				{
					delete[] m_Array[i];
					m_Array[i] = nullptr;
				}
			}
		}
	}
};