
#pragma once

#include <memory>
#include <iostream>
#include <vector>
#include "AABB.h"
#include "SignedDistanceField.h"

using std::vector;

template<class T>
class Generic3DArray
{
protected:
	T* m_Buffer;
	T m_Zero;

	AABB aabb;
	Ogre::Vector3 translation;
	unsigned int xNumCells, yNumCells, zNumCells, xyNumCells, totalNumCells;
	float cellSize, inverseCellSize;

	static const int m_ZeroBlockSize = 4096;
	T m_Zeroes[m_ZeroBlockSize];

	// 4GB limit
	const static int MEMORY_MAX = 1000000000;


public:
	virtual ~Generic3DArray()
	{
		delete[] m_Buffer;
	}
	Generic3DArray() : m_Buffer(nullptr) {}

	AABB getAABB()
	{
		return aabb;
	}

	bool isNull() { return (m_Buffer == nullptr); }

	inline unsigned int gridCellId(int x, int y, int z) const
	{
		return x + y * xNumCells + z * xyNumCells;
	}

	inline unsigned int totalNrOfCells()
	{
		return totalNumCells;
	}
	inline unsigned int getNumCellsX()
	{
		return xNumCells;
	}
	inline unsigned int getNumCellsY()
	{
		return yNumCells;
	}
	inline unsigned int getNumCellsZ()
	{
		return zNumCells;
	}

	inline float getCellSize()
	{
		return cellSize;
	}
	inline float getInverseCellSize()
	{
		return inverseCellSize;
	}


	inline bool isValidCell(int x, int y, int z) const
	{
		unsigned int cellId = gridCellId(x, y, z);
		return cellId >= 0 && cellId < totalNrOfCells();
	}

	void initialize(const AABB& aabb, float cellSize)
	{
		int nrOfCellsOld = totalNrOfCells();

		this->aabb = aabb;
		translation = aabb.getCenter();
		const float offset = 0.5;		
		xNumCells = (unsigned int)std::floor(((aabb.max.x - aabb.min.x) / cellSize) + offset)+1;
		yNumCells = (unsigned int)std::floor(((aabb.max.y - aabb.min.y) / cellSize) + offset)+1;
		zNumCells = (unsigned int)std::floor(((aabb.max.z - aabb.min.z) / cellSize) + offset)+1;
		xyNumCells = xNumCells * yNumCells;
		totalNumCells = xyNumCells * zNumCells;
		this->cellSize = cellSize;
		inverseCellSize = 1.0f / cellSize;

		if (m_Buffer && nrOfCellsOld != totalNrOfCells())
		{
			delete[] m_Buffer;
			m_Buffer = nullptr;
		}
		if (!m_Buffer)
		{
			if (sizeof(T) * totalNrOfCells() > MEMORY_MAX)
			{
				std::cout << "[Generic3DArray] Requested array size (" <<  sizeof(T) * totalNrOfCells() << ") is too large!" << std::endl;
				return;
			}
			m_Buffer = new T[totalNrOfCells()];
		}
	}

	inline T& lookupSafe(int cellX, int cellY, int cellZ)
	{
		if (isValidCell(cellX, cellY, cellZ))
			return m_Buffer[gridCellId(cellX, cellY, cellZ)];
		else return m_Zero;
	}
	inline T& lookupOrCreateSafe(int cellX, int cellY, int cellZ)
	{
		if (isValidCell(cellX, cellY, cellZ))
			return m_Buffer[gridCellId(cellX, cellY, cellZ)];
		else return m_Zero;
	}

	// no check whether cell is valid, crashes if out of bounds!
	inline T& lookup(int cellX, int cellY, int cellZ)
	{
		return m_Buffer[gridCellId(cellX, cellY, cellZ)];
	}
	inline T& lookupOrCreate(int cellX, int cellY, int cellZ)
	{
		return m_Buffer[gridCellId(cellX, cellY, cellZ)];
	}

	void clearGrid()
	{
		int i = 0;
		int numCells = (int)totalNrOfCells();
		while (i < numCells - m_ZeroBlockSize)
		{
			memcpy(&m_Buffer[i], m_Zeroes, m_ZeroBlockSize * sizeof(T));
			i += m_ZeroBlockSize;
		}
		if (i < numCells)
			memcpy(&m_Buffer[i], m_Zeroes, (numCells - i) * sizeof(T));
	}
};

class SignedDistanceField3DArray : public Generic3DArray<float>, public SampledSignedDistanceField3D<MaterialID>
{
public:
	vector<Cube> getCubesToMarch()
	{
		vector<Cube> cubes;
		for (unsigned int x = 0; x < xNumCells-1; x++)
		{
			for (unsigned int y = 0; y < yNumCells-1; y++)
			{
				for (unsigned int z = 0; z < zNumCells-1; z++)
				{
					Cube cube;
					cube.signedDistances[0] = m_Buffer[gridCellId(x, y, z)];
					cube.signedDistances[1] = m_Buffer[gridCellId(x, y, z+1)];
					cube.signedDistances[2] = m_Buffer[gridCellId(x, y+1, z)];
					cube.signedDistances[3] = m_Buffer[gridCellId(x, y+1, z+1)];
					cube.signedDistances[4] = m_Buffer[gridCellId(x+1, y, z)];
					cube.signedDistances[5] = m_Buffer[gridCellId(x+1, y, z+1)];
					cube.signedDistances[6] = m_Buffer[gridCellId(x+1, y+1, z)];
					cube.signedDistances[7] = m_Buffer[gridCellId(x+1, y+1, z+1)];
					bool positive = cube.signedDistances[0] >= 0;
					bool sameSign = true;
					for (int i = 1; i < 8; i++)
						sameSign = sameSign && (positive == (cube.signedDistances[i] >= 0));
					if (!sameSign)
					{
						cube.posMin = Vector3i(x, y, z);
						cube.userData = 0;
						cubes.push_back(cube);
					}
				}
			}
		}
		return cubes;
	}

	template<class SDFConstructor>
	static std::shared_ptr<SampledSignedDistanceField3D> sampleSDF(const SDFConstructor& constructor, float cellSize)
	{
		std::shared_ptr<SampledSignedDistanceField3D> sdf = std::make_shared<SampledSignedDistanceField3D>();
		sdf->initialize(constructor.getAABB(), cellSize);
		std::cout << "Num cells: " << sdf->totalNrOfCells() << std::endl;
		for (unsigned int x = 0; x < sdf->getNumCellsX(); x++)
		{
			for (unsigned int y = 0; y < sdf->getNumCellsY(); y++)
			{
				for (unsigned int z = 0; z < sdf->getNumCellsZ(); z++)
				{
					Vector3 point = Vector3((float)x, (float)y, (float)z) * sdf->getCellSize() + sdf->aabb.getMin();
					sdf->lookupOrCreate(x, y, z) = constructor.getSignedDistance(point);
				}
			}
		}
		return sdf;
	}
};