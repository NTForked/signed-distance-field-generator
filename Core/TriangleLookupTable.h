
#pragma once

#include <functional>
#include "Vector3i.h"
#include "Triangle.h"
#include <unordered_map>

/*
	   d------------h
	  /|           /|
	 / |          / |
	c------------g  |
	|  |         |  |
	|  b---------|--f
	| /          | /
	|/           |/
	a------------e
*/

class TLT
{
private:
	unsigned char boolsToByte(bool *bools)
	{
		unsigned char byte = 0;
		for (unsigned int i = 0; i < 8; i++)
			if (bools[i]) byte |= (1<<i);
		return byte;

	}
	unsigned char rotateX(unsigned char word)
	{
		bool newConfig[8] = {(word&4)!=0, (word&1)!=0, (word&8)!=0, (word&2)!=0, (word&64)!=0, (word&16)!=0, (word&128)!=0, (word&32)!=0};
		return boolsToByte(newConfig);
	}
	unsigned char rotateY(unsigned char word)
	{
		bool newConfig[8] = {(word&2)!=0, (word&32)!=0, (word&8)!=0, (word&128)!=0, (word&1)!=0, (word&16)!=0, (word&4)!=0, (word&64)!=0};
		return boolsToByte(newConfig);
	}
	static Vector3i rotateVectorX (const Vector3i &vec)
	{
		return Vector3i(vec.x,vec.z,-vec.y);
	}
	static Vector3i rotateVectorY (const Vector3i &vec)
	{
		return Vector3i(-vec.z,vec.y,vec.x);
	}

	void addBaseConfig(unsigned char config, const std::vector<Triangle<Vector3i>> &triangles, int rotateN = 0)
	{
		if (rotateN < 6)
		{
			unsigned char rotXConfig = rotateX(config);
			if (table[rotXConfig].size() == 0)
				Triangle<Vector3i>::transformTriangles(table[rotXConfig], triangles, std::function<Vector3i(const Vector3i&)>(rotateVectorX));
			addBaseConfig(rotXConfig, table[rotXConfig], rotateN+1);

			unsigned char rotYConfig = rotateY(config);
			if (table[rotYConfig].size() == 0)
				Triangle<Vector3i>::transformTriangles(table[rotYConfig], triangles, std::function<Vector3i(const Vector3i&)>(rotateVectorY));
			addBaseConfig(rotYConfig, table[rotYConfig], rotateN+1);
		}
	}
	void addBaseConfig(unsigned char config, const Triangle<Vector3i> &t1)
	{
		table[config].push_back(t1);
		addBaseConfig(config, table[config]);
	}
	void addBaseConfig(unsigned char config, const Triangle<Vector3i> &t1, const Triangle<Vector3i> &t2)
	{
		table[config].push_back(t1); table[config].push_back(t2);
		addBaseConfig(config, table[config]);
	}
	void addBaseConfig(unsigned char config, const Triangle<Vector3i> &t1, const Triangle<Vector3i> &t2, const Triangle<Vector3i> &t3)
	{
		table[config].push_back(t1); table[config].push_back(t2); table[config].push_back(t3);
		addBaseConfig(config, table[config]);
	}
	void addBaseConfig(unsigned char config, const Triangle<Vector3i> &t1, const Triangle<Vector3i> &t2, const Triangle<Vector3i> &t3, const Triangle<Vector3i> &t4)
	{
		table[config].push_back(t1); table[config].push_back(t2); table[config].push_back(t3); table[config].push_back(t4);
		addBaseConfig(config, table[config]);
	}
	void addBaseConfig(unsigned char config, const Triangle<Vector3i> &t1, const Triangle<Vector3i> &t2, const Triangle<Vector3i> &t3, const Triangle<Vector3i> &t4, const Triangle<Vector3i> &t5)
	{
		table[config].push_back(t1); table[config].push_back(t2); table[config].push_back(t3); table[config].push_back(t4); table[config].push_back(t5);
		addBaseConfig(config, table[config]);
	}


public:
	// table is stored in two different formats for convenience
	std::vector<Triangle<Vector3i>> table[256];
	std::vector<Triangle<int>> indexTable[256];

	// maps vertices at edge mids to the two neighbor nodes
	std::unordered_map<Vector3i, std::pair<unsigned char, unsigned char> > edgeMidsToNodes;

	std::unordered_map<Vector3i, int> edgeIndexMap;

	struct DirectedEdge
	{
		DirectedEdge() {}
		DirectedEdge(unsigned char minCornerIndex, unsigned char direction) : minCornerIndex(minCornerIndex), direction(direction) {}
		unsigned char minCornerIndex;
		unsigned char direction;
	};

	DirectedEdge directedEdges[12];

	TLT()
	{
		Vector3i verts[12]={
			Vector3i(-1,0,-1),
			Vector3i(0,-1,-1),
			Vector3i(0,1,-1),
			Vector3i(1,0,-1),
			Vector3i(-1,-1,0),
			Vector3i(-1,1,0),
			Vector3i(1,-1,0),
			Vector3i(1,1,0),
			Vector3i(-1,0,1),
			Vector3i(0,-1,1),
			Vector3i(0,1,1),
			Vector3i(1,0,1),
		};

		directedEdges[0] = DirectedEdge(0, 1);
		directedEdges[1] = DirectedEdge(0, 0);
		directedEdges[2] = DirectedEdge(2, 0);
		directedEdges[3] = DirectedEdge(4, 1);
		directedEdges[4] = DirectedEdge(0, 2);
		directedEdges[5] = DirectedEdge(2, 2);
		directedEdges[6] = DirectedEdge(4, 2);
		directedEdges[7] = DirectedEdge(6, 2);
		directedEdges[8] = DirectedEdge(1, 1);
		directedEdges[9] = DirectedEdge(1, 0);
		directedEdges[10] = DirectedEdge(3, 0);
		directedEdges[11] = DirectedEdge(5, 1);

		edgeIndexMap.insert(std::make_pair(Vector3i(-1, 0, -1), 0));
		edgeIndexMap.insert(std::make_pair(Vector3i(0, -1, -1), 1));
		edgeIndexMap.insert(std::make_pair(Vector3i(0, 1, -1), 2));
		edgeIndexMap.insert(std::make_pair(Vector3i(1, 0, -1), 3));
		edgeIndexMap.insert(std::make_pair(Vector3i(-1, -1, 0), 4));
		edgeIndexMap.insert(std::make_pair(Vector3i(-1, 1, 0), 5));
		edgeIndexMap.insert(std::make_pair(Vector3i(1, -1, 0), 6));
		edgeIndexMap.insert(std::make_pair(Vector3i(1, 1, 0), 7));
		edgeIndexMap.insert(std::make_pair(Vector3i(-1, 0, 1), 8));
		edgeIndexMap.insert(std::make_pair(Vector3i(0, -1, 1), 9));
		edgeIndexMap.insert(std::make_pair(Vector3i(0, 1, 1), 10));
		edgeIndexMap.insert(std::make_pair(Vector3i(1, 0, 1), 11));

		edgeMidsToNodes.insert(std::make_pair(Vector3i(-1,0,-1), std::make_pair(0, 2)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(0,-1,-1), std::make_pair(0, 4)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(0,1,-1), std::make_pair(2, 6)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(1,0,-1), std::make_pair(4, 6)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(-1,-1,0), std::make_pair(0, 1)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(-1,1,0), std::make_pair(2, 3)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(1,-1,0), std::make_pair(4, 5)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(1,1,0), std::make_pair(6, 7)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(-1,0,1), std::make_pair(1, 3)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(0,-1,1), std::make_pair(1, 5)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(0,1,1), std::make_pair(3, 7)));
		edgeMidsToNodes.insert(std::make_pair(Vector3i(1,0,1), std::make_pair(5, 7)));

		addBaseConfig(1, Triangle<Vector3i>(verts[1], verts[0], verts[4]));

		addBaseConfig(3, Triangle<Vector3i>(verts[1], verts[0], verts[8]),
			Triangle<Vector3i>(verts[1], verts[8], verts[9]));

		addBaseConfig(6, Triangle<Vector3i>(verts[0], verts[9],  verts[4]),
			Triangle<Vector3i>(verts[0], verts[2],  verts[9]),
			Triangle<Vector3i>(verts[5], verts[8],  verts[9]),
			Triangle<Vector3i>(verts[5], verts[9],  verts[2]));

		addBaseConfig (7, Triangle<Vector3i>(verts[1], verts[2],  verts[9]),
			Triangle<Vector3i>(verts[2], verts[5],  verts[9]),
			Triangle<Vector3i>(verts[5], verts[8],  verts[9]));

		addBaseConfig(15, Triangle<Vector3i>(verts[10], verts[1], verts[2]),
			Triangle<Vector3i>(verts[1], verts[10], verts[9]));

		addBaseConfig(22, Triangle<Vector3i>(verts[0], verts[1],  verts[4]),
			Triangle<Vector3i>(verts[2], verts[5],  verts[8]),
			Triangle<Vector3i>(verts[2], verts[8],  verts[9]),
			Triangle<Vector3i>(verts[2], verts[9],  verts[6]),
			Triangle<Vector3i>(verts[2], verts[6],  verts[3]));

		addBaseConfig(23, Triangle<Vector3i>(verts[2], verts[5],  verts[6]),
			Triangle<Vector3i>(verts[2], verts[6],  verts[3]),
			Triangle<Vector3i>(verts[5], verts[9],  verts[6]),
			Triangle<Vector3i>(verts[5], verts[8],  verts[9]));

		addBaseConfig(24, Triangle<Vector3i>(verts[8], verts[5],  verts[10]),
			Triangle<Vector3i>(verts[6], verts[3],  verts[1]));

		addBaseConfig(25, Triangle<Vector3i>(verts[6], verts[10], verts[8]),
			Triangle<Vector3i>(verts[6], verts[8], verts[4]),
			Triangle<Vector3i>(verts[3], verts[5], verts[10]),
			Triangle<Vector3i>(verts[3], verts[0], verts[5]),
			Triangle<Vector3i>(verts[3], verts[10], verts[6]));

		addBaseConfig(27, Triangle<Vector3i>(verts[10], verts[0], verts[5]),
			Triangle<Vector3i>(verts[6], verts[3], verts[9]),
			Triangle<Vector3i>(verts[10], verts[9], verts[3]),
			Triangle<Vector3i>(verts[10], verts[3], verts[0]));

		addBaseConfig(29, Triangle<Vector3i>(verts[6], verts[3], verts[2]),
			Triangle<Vector3i>(verts[6], verts[2], verts[4]),
			Triangle<Vector3i>(verts[4], verts[10], verts[8]),
			Triangle<Vector3i>(verts[4], verts[2], verts[10]));

		addBaseConfig(30, Triangle<Vector3i>(verts[2], verts[10], verts[3]),
			Triangle<Vector3i>(verts[3], verts[10], verts[6]),
			Triangle<Vector3i>(verts[1], verts[4], verts[0]),
			Triangle<Vector3i>(verts[10], verts[9], verts[6]));

		addBaseConfig(31, Triangle<Vector3i>(verts[3], verts[9], verts[6]),
			Triangle<Vector3i>(verts[3], verts[10], verts[9]),
			Triangle<Vector3i>(verts[2], verts[10], verts[3]));

		addBaseConfig(60, Triangle<Vector3i>(verts[3], verts[2], verts[10]),
			Triangle<Vector3i>(verts[3], verts[10], verts[11]),
			Triangle<Vector3i>(verts[9], verts[8], verts[0]),
			Triangle<Vector3i>(verts[1], verts[9], verts[0]));

		addBaseConfig(61, Triangle<Vector3i>(verts[3], verts[2], verts[10]),
			Triangle<Vector3i>(verts[3], verts[10], verts[11]),
			Triangle<Vector3i>(verts[4], verts[9], verts[8]));

		addBaseConfig(63, Triangle<Vector3i>(verts[3], verts[2], verts[10]),
			Triangle<Vector3i>(verts[3], verts[10], verts[11]));

		addBaseConfig(105, Triangle<Vector3i>(verts[3], verts[6], verts[1]),
			Triangle<Vector3i>(verts[4], verts[9], verts[8]),
			Triangle<Vector3i>(verts[10], verts[11], verts[7]),
			Triangle<Vector3i>(verts[0], verts[5], verts[2]));

		addBaseConfig(107, Triangle<Vector3i>(verts[10], verts[11], verts[7]),
			Triangle<Vector3i>(verts[3], verts[6], verts[1]),
			Triangle<Vector3i>(verts[0], verts[5], verts[2]));

		addBaseConfig(111, Triangle<Vector3i>(verts[10], verts[11], verts[7]),
			Triangle<Vector3i>(verts[3], verts[6], verts[1]));

		addBaseConfig(126, Triangle<Vector3i>(verts[10], verts[11], verts[7]),
			Triangle<Vector3i>(verts[1], verts[4], verts[0]));

		addBaseConfig(127, Triangle<Vector3i>(verts[10], verts[11], verts[7]));

		for (int i = 0; i < 256; i++)
		{
			for (auto it = table[i].begin(); it != table[i].end(); ++it)
			{
				Triangle<int> tri(
					(int)edgeIndexMap[it->p1],
					(int)edgeIndexMap[it->p2],
					(int)edgeIndexMap[it->p3]);
				indexTable[i].push_back(tri);
			}
		}
	}

	friend std::ostream& operator<< (std::ostream &out, TLT &tlt) {
		for (unsigned i = 0; i < 255; i++) {
			out << "[";
			for (auto it = tlt.table[i].begin(); it != tlt.table[i].end(); it++) {
				out << *it << ", ";
			}
			out << "]" << std::endl;
		}
		return out;
	}

	static TLT& getSingleton()
	{
		static TLT theOneAndOnly;
		return theOneAndOnly;
	}

};