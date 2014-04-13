
#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "OgreMath/OgreVector3.h"

class OBJReader
{
private:
	/// Triangulates a polygon (face) and writes the indices to triIndices.
	static void triangulateFace(std::vector<unsigned int> &faceIndices, std::vector<unsigned int> &triIndices)
	{
		if (faceIndices.size() < 3) return;
		if (faceIndices.size() == 3)
		{
			triIndices.push_back(faceIndices[0]);
			triIndices.push_back(faceIndices[1]);
			triIndices.push_back(faceIndices[2]);
		}
		else
		{
			triIndices.push_back(faceIndices[faceIndices.size()-2]);
			triIndices.push_back(faceIndices[faceIndices.size()-1]);
			triIndices.push_back(faceIndices[0]);
			faceIndices.pop_back();
			triangulateFace(faceIndices, triIndices);
		}
	}
public:
	static void readObjFile(std::ifstream &file, std::vector<Ogre::Vector3> &vertexPositions, std::vector<Ogre::Vector3> &vertexNormals, std::vector<unsigned int> &indexBuffer)
	{
		std::string line;
		while (!file.eof())
		{
			std::getline(file, line);
			std::stringstream lineStream(line);
			if (lineStream.peek() == 'v')
			{
				lineStream.seekg(1, std::ios::cur);
				bool isNormal = lineStream.peek() == 'n';
				if (isNormal) lineStream.seekg(1, std::ios::cur);
				if (lineStream.peek() == ' ')
				{
					while (lineStream.peek() != ' ') lineStream.seekg(1, std::ios::cur);
					lineStream.seekg(1, std::ios::cur);
					Ogre::Vector3 v;
					lineStream >> v.x;
					while (lineStream.peek() != ' ') lineStream.seekg(1, std::ios::cur);
					lineStream.seekg(1, std::ios::cur);
                    lineStream >> v.y;
					while (lineStream.peek() != ' ') lineStream.seekg(1, std::ios::cur);
					lineStream.seekg(1, std::ios::cur);
                    lineStream >> v.z;
                    // v.y = -v.y;
					if (!isNormal) vertexPositions.push_back(v);
					else vertexNormals.push_back(v);
				}
			}
			else if (lineStream.peek() == 'f')
			{
				lineStream.seekg(1, std::ios::cur);
				std::vector<unsigned int> faceIndices;
				while (lineStream.peek() == ' ')
				{
					lineStream.seekg(1, std::ios::cur);
					int index;
					lineStream >> index;
					faceIndices.push_back(index-1);
					if (lineStream.peek() == '/')
					{
						lineStream.seekg(1, std::ios::cur);
						int vTexIndex;
						if (lineStream.peek() >= 48 && lineStream.peek() <= 57)
							lineStream >> vTexIndex;
					}
					if (lineStream.peek() == '/')
					{
						lineStream.seekg(1, std::ios::cur);
						int vNormalIndex;
						if (lineStream.peek() >= 48 && lineStream.peek() <= 57)
							lineStream >> vNormalIndex;
					}
				}
				triangulateFace(faceIndices, indexBuffer);
			}
		}
	}
};
