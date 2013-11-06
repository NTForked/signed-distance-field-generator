// Stanislaw Adaszewski, 2013

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "OgreMath/OgreVector3.h"
#include "GenericVertex.h"

class ExportPLY
{
public:
	static void writeMesh(const std::string &fileName, std::vector<GenericVertex<MaterialID> > &vertexBuffer, const std::vector<Ogre::Vector3> &normalBuffer, const std::vector<unsigned int> &indexBuffer) {
		std::ofstream ply(fileName + ".ply", std::ios_base::out|std::ios_base::trunc);
		ply << "ply\n\
format ascii 1.0\n\
element vertex " << vertexBuffer.size() << "\n\
property float x\n\
property float y\n\
property float z\n\
property float nx\n\
property float ny\n\
property float nz\n\
property uchar red\n\
property uchar green\n\
property uchar blue\n\
element face " << indexBuffer.size()/3 << "\n\
property list uchar uint vertex_indices\n\
end_header\n";
		for (unsigned int i = 0; i < vertexBuffer.size(); i++) {
			ply << vertexBuffer[i].position.x << " " << vertexBuffer[i].position.y << " " << vertexBuffer[i].position.z << " " << normalBuffer[i].x << " " << normalBuffer[i].y << " " << normalBuffer[i].z << " " << ((vertexBuffer[i].data >> 24)&0xFF) << " " << ((vertexBuffer[i].data >> 16)&0xFF) << " " << ((vertexBuffer[i].data >> 8)&0xFF) << "\n";
		}
		for (unsigned int i = 0; i < indexBuffer.size(); i+= 3) {
			ply << "3 " << indexBuffer[i] << " " << indexBuffer[i+1] << " " << indexBuffer[i+2] << "\n";
		}
		ply.close();
	}

};