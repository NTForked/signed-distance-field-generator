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
#include <vector>
#include <ostream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include "Prerequisites.h"
#include "TriangleLookupTable.h"
#include "Vertex.h"
#include "SignedDistanceField.h"
#include "Mesh.h"

using std::vector;

class MarchingCubes {
	static inline void getCubeVecs(Ogre::Vector3 *vecs, const Vector3i &min)
	{
		Ogre::Vector3 minF = min.toOgreVec();	// + Ogre::Vector3(-0.5f, -0.5f, -0.5f);
		vecs[0] = minF + Ogre::Vector3(0, 0, 0);
		vecs[1] = minF + Ogre::Vector3(0, 0, 1);
		vecs[2] = minF + Ogre::Vector3(0, 1, 0);
		vecs[3] = minF + Ogre::Vector3(0, 1, 1);
		vecs[4] = minF + Ogre::Vector3(1, 0, 0);
		vecs[5] = minF + Ogre::Vector3(1, 0, 1);
		vecs[6] = minF + Ogre::Vector3(1, 1, 0);
		vecs[7] = minF + Ogre::Vector3(1, 1, 1);
	}

	static int min2(int a, int b)
	{
		return (a < b) ? a : b;
	}

	typedef Vector3i CubeVecs[8];

	struct VertexIndexed
	{
		size_t index;
		Vertex vertex;
	};

	static inline void marchCube(
		const SampledSignedDistanceField3D::Cube& cube,
		std::unordered_map<Vector3i, VertexIndexed> &vertexMap,
		std::vector<unsigned int> &indexBuffer)
	{
		Ogre::Vector3 cubeVecs[8];
		getCubeVecs(cubeVecs, cube.posMin);
		unsigned char word = 0;
		for (int i = 0; i < 8; i++)
		{
			if (cube.cornerSamples[i].signedDistance >= 0)
				word |= (1<<i);
		}
		Vector3i offset = cube.posMin.doubleVec()+Vector3i(1,1,1);
		const std::vector<Triangle<Vector3i> >& triangles = TLT::getSingleton().table[word];
		for (unsigned int i = 0; i < triangles.size(); i++)
		{
			// c1 * (1-w) + c2 * w = 0
			// w = c1 / (c1 - c2)
			VertexIndexed indexed;
			indexed.vertex.normal = cube.cornerSamples[0].normal;
			indexed.vertex.uv = cube.cornerSamples[0].uv;
			indexed.index = vertexMap.size();
			Vector3i vertex1(triangles[i].p1+offset);
			auto tryInsert = vertexMap.insert(std::make_pair(vertex1, indexed));
			if (tryInsert.second)
			{
				auto nodes = TLT::getSingleton().edgeMidsToNodes[triangles[i].p1];
				float w = cube.cornerSamples[nodes.first].signedDistance
					/ (cube.cornerSamples[nodes.first].signedDistance - cube.cornerSamples[nodes.second].signedDistance);
				vAssert(w >= 0 && w <= 1);
				tryInsert.first->second.vertex.position = cubeVecs[nodes.first] * (1-w) + cubeVecs[nodes.second] * w;
			}
			indexBuffer.push_back((unsigned int)tryInsert.first->second.index);

			indexed.index = vertexMap.size();

			Vector3i vertex2(triangles[i].p2+offset);
			tryInsert = vertexMap.insert(std::make_pair(vertex2, indexed));
			if (tryInsert.second)
			{
				auto nodes = TLT::getSingleton().edgeMidsToNodes[triangles[i].p2];
				float w = cube.cornerSamples[nodes.first].signedDistance
					/ (cube.cornerSamples[nodes.first].signedDistance - cube.cornerSamples[nodes.second].signedDistance);
				vAssert(w >= 0 && w <= 1);
				tryInsert.first->second.vertex.position = cubeVecs[nodes.first] * (1 - w) + cubeVecs[nodes.second] * w;
			}
			indexBuffer.push_back((unsigned int)tryInsert.first->second.index);

			indexed.index = vertexMap.size();

			Vector3i vertex3(triangles[i].p3+offset);
			tryInsert = vertexMap.insert(std::make_pair(vertex3, indexed));
			if (tryInsert.second)
			{
				auto nodes = TLT::getSingleton().edgeMidsToNodes[triangles[i].p3];
				float w = cube.cornerSamples[nodes.first].signedDistance
					/ (cube.cornerSamples[nodes.first].signedDistance - cube.cornerSamples[nodes.second].signedDistance);
				vAssert(w >= 0 && w <= 1);
				tryInsert.first->second.vertex.position = cubeVecs[nodes.first] * (1 - w) + cubeVecs[nodes.second] * w;
			}
			indexBuffer.push_back((unsigned int)tryInsert.first->second.index);
		}
	}

public:
	static void computeVertexNormals(std::vector<Ogre::Vector3> &normalBuffer, const std::vector<Vertex > &vertexBuffer, const std::vector<unsigned int> &indexBuffer)
	{
		normalBuffer.resize(vertexBuffer.size());
		std::vector<std::vector<Ogre::Vector3>> vertexNormalNeighbors;
		vertexNormalNeighbors.resize(vertexBuffer.size());
		for (int i = 0; i < ((int)indexBuffer.size())-3; i+=3)
		{
			Ogre::Vector3 normal((vertexBuffer[indexBuffer[i+2]].position-vertexBuffer[indexBuffer[i]].position).
				crossProduct((vertexBuffer[indexBuffer[i+1]].position-vertexBuffer[indexBuffer[i]].position)));
			normal.normalise();
			vertexNormalNeighbors[indexBuffer[i]].push_back(normal);
			vertexNormalNeighbors[indexBuffer[i+1]].push_back(normal);
			vertexNormalNeighbors[indexBuffer[i+2]].push_back(normal);
		}
		for (unsigned int i = 0; i < normalBuffer.size(); i++)
		{
			normalBuffer[i] = Ogre::Vector3(0,0,0);
			for (unsigned int n = 0; n < vertexNormalNeighbors[i].size(); n++)
				normalBuffer[i] += vertexNormalNeighbors[i][n];
			normalBuffer[i].normalise();
		}
	}

	static std::shared_ptr<Mesh> marchSDF( SampledSignedDistanceField3D& sdf, float voxelsPerUnit)
	{
		std::cout << "[Marching cubes] Fetching cubes..." << std::endl;
		vector<SampledSignedDistanceField3D::Cube > cubes = sdf.getCubesToMarch();
		std::cout << "Fetched " << cubes.size() << " cubes!" << std::endl;

		std::shared_ptr<Mesh> outMesh = std::make_shared<Mesh>();
		std::unordered_map<Vector3i, VertexIndexed> vertexMap;
		std::cout << "[Marching cubes] Marching..." << std::endl;
		for (auto ic = cubes.begin(); ic != cubes.end(); ++ic)
		{
			marchCube(*ic, vertexMap, outMesh->indexBuffer);
		}
		std::cout << "[Marching cubes] " << vertexMap.size() << " vertices created." << std::endl;

		// build vertex buffer
		outMesh->vertexBuffer.resize(vertexMap.size());
		for (auto it = vertexMap.begin(); it != vertexMap.end(); it++)
		{
			outMesh->vertexBuffer[it->second.index] = it->second.vertex;
		}

		// finally scale with respect to voxelsPerUnit 
		float scale = 1.0f / voxelsPerUnit;
		for (auto it = outMesh->vertexBuffer.begin(); it != outMesh->vertexBuffer.end(); it++)
			it->position = (it->position * scale) + sdf.getAABB().getMin();

		return outMesh;
	}
};
