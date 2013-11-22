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
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "OgreMath/OgreVector3.h"
#include "Prerequisites.h"
#include "Vertex.h"

class ExportOBJ
{
public:
	static void writeMesh(const std::string &fileName, const std::vector<Vertex > &vertexBuffer, const std::vector<Ogre::Vector3> &normalBuffer, const std::vector<unsigned int> &indexBuffer)
	{
		std::ofstream objFile(fileName + ".obj", std::ios_base::out|std::ios_base::trunc);

		for (unsigned int i = 0; i < vertexBuffer.size(); i++)
			objFile << "v " << vertexBuffer[i].position.x << " " << vertexBuffer[i].position.z << " " << -vertexBuffer[i].position.y << std::endl;

		objFile << "s 1" << std::endl;

		for (int i = 0; i < ((int)indexBuffer.size())-2; i+=3)
		{
			objFile << "f " << indexBuffer[i]+1 << " " << indexBuffer[i+1]+1 << " " << indexBuffer[i+2]+1 << std::endl;
		}

		objFile.close();
	}
};

class ExportOBJWithNormals
{
public:
	static void writeMesh(const std::string &fileName, const std::vector<Vertex > &vertexBuffer, const std::vector<Ogre::Vector3> &normalBuffer, const std::vector<unsigned int> &indexBuffer)
	{
		std::ofstream objFile(fileName + ".obj", std::ios_base::out|std::ios_base::trunc);
		std::ostringstream stringStream;

		for (unsigned int i = 0; i < vertexBuffer.size(); i++)
			objFile << "v " << vertexBuffer[i].position.x << " " << vertexBuffer[i].position.z << " " << -vertexBuffer[i].position.y << std::endl;

		for (unsigned int i = 0; i < normalBuffer.size(); i++)
			objFile << "vn " << normalBuffer[i].x << " " << normalBuffer[i].z << " " << -normalBuffer[i].y << std::endl;

		objFile << "s 1" << std::endl;

		for (int i = 0; i < ((int)indexBuffer.size())-2; i+=3)
		{
			objFile << "f " << indexBuffer[i]+1 << "//" << indexBuffer[i]+1 << " "
			<< indexBuffer[i+1]+1 << "//" << indexBuffer[i+1]+1 << " "
			<< indexBuffer[i+2]+1 << "//" << indexBuffer[i+2]+1 << std::endl;
		}

		objFile.close();
	}
};