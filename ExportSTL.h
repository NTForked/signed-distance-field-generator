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

#include "Prerequisites.h"
#include <fstream>
#include "OgreMath/OgreVector3.h"
#include <vector>
#include "Triangle.h"
#include "Vertex.h"

class ExportSTL
{
public:
	static void writeMesh(const std::string &fileName, std::vector<Vertex > &vertexBuffer, const std::vector<Ogre::Vector3> &normalBuffer, const std::vector<unsigned int> &indexBuffer)
	{
		std::vector<Triangle<Ogre::Vector3>> faces;
		size_t numTris = indexBuffer.size() / 3;
		faces.resize(numTris);
		for (size_t i = 0; i < numTris; i++)
		{
			faces[i].p1 = vertexBuffer[indexBuffer[i*3]].position;
			faces[i].p2 = vertexBuffer[indexBuffer[i*3+1]].position;
			faces[i].p3 = vertexBuffer[indexBuffer[i*3+2]].position;
		}

		static char acHeader[80]="A simple STL file writer. Originally written to test marching cubes output.";
		std::ofstream osSTLFile(fileName+ ".stl", std::ios_base::binary|std::ios_base::out|std::ios_base::trunc);
		osSTLFile.write(acHeader, 80);
		size_t uiFileSize=faces.size();
		osSTLFile.write((char*)&uiFileSize, 4);//must be 4 here, see spec

		const int bufferSize = 4096;
		char buffer[bufferSize];
		unsigned int bufferOffset = 0;

		for(size_t iFace=0; iFace<faces.size(); iFace++)
		{
			if (bufferOffset > bufferSize-50)
			{
				osSTLFile.write(buffer, bufferOffset);
				bufferOffset = 0;
			}
			Triangle<Ogre::Vector3> &f=faces[iFace];
			Ogre::Vector3 vNormal((f.p3-f.p1).crossProduct((f.p2-f.p1)));
			vNormal.normalise();
			for(int iCoord=0; iCoord<3; iCoord++)
			{
				*(float*)(buffer+bufferOffset) = vNormal[iCoord];
				bufferOffset+=4;
			}
			for(int iCoord=0; iCoord<3; iCoord++)
			{
				*(float*)(buffer+bufferOffset) = f.p1[iCoord];
				bufferOffset+=4;
			}
			for(int iCoord=0; iCoord<3; iCoord++)
			{
				*(float*)(buffer+bufferOffset) = f.p2[iCoord];
				bufferOffset+=4;
			}
			for(int iCoord=0; iCoord<3; iCoord++)
			{
				*(float*)(buffer+bufferOffset) = f.p3[iCoord];
				bufferOffset+=4;
			}
			buffer[bufferOffset++] = 0;
			buffer[bufferOffset++] = 0;
		}
		osSTLFile.write(buffer, bufferOffset);
		osSTLFile.close();
	}
};