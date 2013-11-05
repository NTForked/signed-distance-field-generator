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

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <list>
#include <map>
#include <set>
#include "OgreMath/OgreVector3.h"
#include "GenericVertex.h"

namespace std
{

	template<>
	struct hash<std::pair<unsigned int, unsigned int>>
	{
	public:
		size_t operator()(const std::pair<unsigned int, unsigned int> &p) const
		{
			static const int size = 4;//4 bytes
			static const int iShift=(size*4);//half the bits
			static const size_t iMask=((size_t)1<<iShift)-1;
			return (p.first&(iMask)) | (p.second<<iShift);
		}
	};

}

class MeshSimplify
{
private:
	typedef std::pair<unsigned int, unsigned int> UIPair;
	static UIPair
	makeEdge(unsigned int vert1, unsigned int vert2)
	{
		if(vert1 < vert2)
			return UIPair(vert1, vert2);
		//else
		return UIPair(vert2, vert1);
	}
public:
	template<class T>
	static void simplify(std::vector<GenericVertex<T> > &GenericVertexBuffer,
			std::vector<unsigned int> &indexBuffer)
	{
		puts("[MeshSimplify] building index ...");

		//marks invalidated vertices in the GenericVertex buffer
		static const Ogre::Vector3 invalidGenericVertex(
				Ogre::Math::POS_INFINITY,
				Ogre::Math::POS_INFINITY,
				Ogre::Math::POS_INFINITY);
		//marks invalidated indices
		static const unsigned int uiInvalidIndex = -1;

		if(!indexBuffer.size() || !GenericVertexBuffer.size())
			return;
		//buffer for normals on each tri
		std::vector<Ogre::Vector3> normalBuffer;
		normalBuffer.resize(indexBuffer.size()/3);

		//mapping GenericVertex -> triangles
		std::vector< std::list<unsigned int> > vVertTri;
		vVertTri.resize(GenericVertexBuffer.size());
		//an edge set
		std::set<UIPair> sEdges;
		//sEdges.rehash(indexBuffer.size());//at maximum as many edges as triangle edges

		const unsigned int nTris = (unsigned int)indexBuffer.size()/3;
		for (unsigned int iTri=0; iTri < nTris; iTri++)
		{
			unsigned int iTriOffset = iTri*3;
			for(unsigned int iTriVert=0; iTriVert<3; iTriVert++)
			{
				unsigned int uiCurrVert=indexBuffer[iTriOffset + iTriVert];
				vVertTri[uiCurrVert].push_back(iTri);
				//add an edge
				unsigned int uiNextVert = iTriVert + 1;
				if(uiNextVert == 3)
					uiNextVert=0;
				const UIPair edge=makeEdge(uiCurrVert, indexBuffer[iTriOffset + uiNextVert]);
				sEdges.insert(edge);

				/*
				////DEBUG!!!
				if(uiCurrVert==21)
					printf("added tri %d, edge (%d, %d)\n", iTri, edge.first, edge.second);
				////DEBUG!!!
				*/
			}
			unsigned int v0=indexBuffer[iTriOffset+0],
					v1=indexBuffer[iTriOffset+1],
					v2=indexBuffer[iTriOffset+2];
			Ogre::Vector3 vNormal=(GenericVertexBuffer[v2].position-GenericVertexBuffer[v0].position).crossProduct(
					GenericVertexBuffer[v1].position-GenericVertexBuffer[v0].position);

			vNormal.normalise();
			normalBuffer[iTri]=vNormal;
		}
		//sort tri lists in GenericVertex -> triangles mapping
		for (unsigned int iVert = 0; iVert < vVertTri.size(); iVert++)
			vVertTri[iVert].sort();

		puts("[MeshSimplify] collapsing edges ...");

		while(sEdges.size())
		{
			//get first vert and check it
			UIPair currEdge=(*sEdges.begin());
			sEdges.erase(*sEdges.begin());

			//printf("checking edge (%d, %d)\n", currEdge.first, currEdge.second);

			//get all neighbors
			//get all the triangles belonging to this edge
			std::list<unsigned int>* aTriLists[2]={
					& vVertTri[currEdge.first],
					& vVertTri[currEdge.second],
				};

			//check if there are triangles (should be)
			if(aTriLists[0]->empty() || aTriLists[1]->empty())
			{
				//this happens when a collapsed edge is processed
				puts("[MeshSimplify] error: edge GenericVertex had no associated tris!");
				continue;
			}
			if(aTriLists[0]->size() < 3 || aTriLists[1]->size() < 3)
			{
				//this happens when a collapsed edge is processed
				puts("[MeshSimplify] error: GenericVertex had less than three associated tris!");
				continue;
			}
			/*
			for (unsigned int iVert = 0; iVert < vVertTri.size(); iVert++)
			{
				if(vVertTri[iVert].empty())
					continue;
				if(vVertTri[iVert].size() < 3)
					puts("[MeshSimplify] error: GenericVertex had less than three associated tris!");
			}*/
			//we only want to simplify tris with very similar normals
			static const float fMinNormalAngle=0.99f;
			//compare randomly against first normal
			const Ogre::Vector3 vRefNormal=normalBuffer[aTriLists[0]->front()];

			bool bEdgeNotGood=false;

			for(unsigned int uiEdgeVert=0; uiEdgeVert<2; uiEdgeVert++)
			{
				for(auto itTri=aTriLists[uiEdgeVert]->begin(); itTri != aTriLists[uiEdgeVert]->end(); itTri++)
				{
					if(normalBuffer[*itTri].dotProduct(vRefNormal) < fMinNormalAngle)
					{
						bEdgeNotGood=true;
						break;
					}
				}
				if(bEdgeNotGood)
					break;
			}
			if(bEdgeNotGood)
				continue;

			//so now we can collapse this edge

			//need to know which triangles will be killed by the collapse
			//GenericVertex -> triangles mapping is sorted
			unsigned int uiCollapsingTris[2];
			unsigned int uiCollapsingTrisFound=0;

			auto itTriList0=aTriLists[0]->begin(),
				itTriList1=aTriLists[1]->begin();
			for( ; itTriList0!=aTriLists[0]->end(); itTriList0++)
			{
				while(*itTriList1 < *itTriList0)
				{
					if(itTriList1 != aTriLists[1]->end())
						itTriList1++;
					else
						break;
				}
				if(itTriList1 == aTriLists[1]->end())
				{
					puts("[MeshSimplify] error: didn't find two tris adjacent to edge!");
					continue;
				}
				if(*itTriList1 == *itTriList0)
				{
					uiCollapsingTris[uiCollapsingTrisFound++] = *itTriList0;
					if(uiCollapsingTrisFound==2)
						break;
				}
			}
			if(uiCollapsingTris[0] == uiCollapsingTris[1])
			{
				puts("[MeshSimplify] error: tried to remove the same tri twice!");
				continue;
			}

			for(unsigned int uiList=0; uiList<2; uiList++)
			{
				unsigned int uiLastIndex=-1;
				auto itTriList=aTriLists[uiList]->begin();
				for( ; itTriList!=aTriLists[uiList]->end(); itTriList++)
				{
					if(*itTriList == uiLastIndex)
						puts("[MeshSimplify] error: index in list twice!");
					uiLastIndex = *itTriList;
				}
			}

			if(uiCollapsingTrisFound!=2)
			{
				puts("[MeshSimplify] error: edge to collapse had only one associated tri!");
				continue;
			}

			Ogre::Vector3 avVerts[2]={
					GenericVertexBuffer[currEdge.first].position,
					GenericVertexBuffer[currEdge.second].position
				};

			//collapse GenericVertex
			const Ogre::Vector3 vCollapsed=(avVerts[0] + avVerts[1])*0.5f;
			GenericVertexBuffer[currEdge.first].position=vCollapsed;
			//invalidate removed GenericVertex
			GenericVertexBuffer[currEdge.second].position=invalidGenericVertex;

			//traverse tris of deleted vert, replace with first vert
			//also add these tris to GenericVertex -> triangle mapping
			for(auto itTriList=vVertTri[currEdge.second].begin();
					 itTriList!=vVertTri[currEdge.second].end();
					 itTriList++)
			{
				const unsigned int uiTriOffset=(*itTriList)*3;
				for(unsigned int uiTriVert=0; uiTriVert<3; uiTriVert++)
				{
					unsigned int index=uiTriOffset + uiTriVert;
					if(indexBuffer[index] == currEdge.second)
						//replace removed GenericVertex index with the collapsed one
						indexBuffer[index] = currEdge.first;
					else
					{//this is a GenericVertex which has an invalid edge in the edge set
						UIPair oldEdge=makeEdge(currEdge.second, indexBuffer[index]);
						if(oldEdge == currEdge)
							continue;

						//edge might already have been checked and is already removed
						sEdges.erase(oldEdge);

						//UIPair newEdge=makeEdge(currEdge.first, indexBuffer[index]);
						//sEdges.insert(newEdge);
					}
				}
				//add tri to mapping
				vVertTri[currEdge.first].push_back(*itTriList);
			}

			vVertTri[currEdge.second].clear();

			//invalidate tris in index buffer
			for(unsigned int uiTri=0; uiTri<2; uiTri++)
			{
				const unsigned int uiInvalidTriOffset=uiCollapsingTris[uiTri]*3;
				for(unsigned int uiTriVert=0; uiTriVert<3; uiTriVert++)
				{
					unsigned int index=uiInvalidTriOffset + uiTriVert;
					unsigned int uiGenericVertexIndex=indexBuffer[index];
					//remove deleted tris from GenericVertex -> triangle mapping

					if(uiGenericVertexIndex == uiInvalidIndex)
					{
						puts("tried to delete deleted tri again");
						continue;
					}
					vVertTri[uiGenericVertexIndex].remove(uiCollapsingTris[uiTri]);
					//invalidate index
					indexBuffer[index]=uiInvalidIndex;
				}
				vVertTri[currEdge.first].remove(uiCollapsingTris[uiTri]);
			}

			vVertTri[currEdge.first].sort();
			vVertTri[currEdge.first].unique();

			//printf("deleted tris %d, %d\n", uiCollapsingTris[0], uiCollapsingTris[1]);
			//puts("collapsed edge!");
		}

		//build new GenericVertex and index buffers
		std::vector<GenericVertex<T> > oldVB=GenericVertexBuffer;
		GenericVertexBuffer.clear();

		std::vector<unsigned int> oldIB=indexBuffer;
		indexBuffer.clear();

		std::vector<unsigned int> vbRemap;
		vbRemap.resize(oldVB.size(), uiInvalidIndex);

		for(unsigned int uiVert=0; uiVert<oldVB.size(); uiVert++)
		{
			GenericVertex<T> v=oldVB[uiVert];
			if(v.position==invalidGenericVertex)
				continue;

			GenericVertexBuffer.push_back(v);
			vbRemap[uiVert] = (unsigned int)GenericVertexBuffer.size()-1;
		}

		for(unsigned int uiIndex=0; uiIndex<oldIB.size(); uiIndex++)
		{
			unsigned int index=oldIB[uiIndex];
			if(index==uiInvalidIndex)
				continue;

			vAssert(vbRemap[index] != uiInvalidIndex);

			indexBuffer.push_back(vbRemap[index]);
		}
		printf("[MeshSimplify] removed %d%% vertices, %d%%faces\n",
				(int)((float)(oldVB.size() - GenericVertexBuffer.size())*100.0f/(float)oldVB.size()),
				(int)((float)(oldIB.size() -  indexBuffer.size())*100.0f/(float)oldIB.size())
		);
	}
};
