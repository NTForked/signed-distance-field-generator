
#pragma once

#include <list>
#include <vector>
#include <algorithm>
#include "OgreMath/OgreVector3.h"

class VertexMerger
{
	struct OctreeNode
	{
		OctreeNode(){vPos.x=0.0f;vPos.y=0.0f;vPos.z=0.0f;
			aSubNodes[0]=0;aSubNodes[1]=0;aSubNodes[2]=0;aSubNodes[3]=0;aSubNodes[4]=0;aSubNodes[5]=0;aSubNodes[6]=0;aSubNodes[7]=0;}
		Ogre::Vector3 vPos;
		OctreeNode* aSubNodes[8];
		std::list<int> liIndices;
	};
	
	struct STri
	{
		STri(){i1=-1;i2=-1;i3=-1;}
		STri(int iIndex1, int iIndex2, int iIndex3, bool bSort=true)
		{
			if(!bSort)
			{
				i1=iIndex1;
				i2=iIndex2;
				i3=iIndex3;
				return;
			}
			//rotate indices
			if(iIndex2<iIndex1)
			{
				if(iIndex3<iIndex2)
				{//index 3 is the smallest
					i1=iIndex3;
					i2=iIndex1;
					i3=iIndex2;
				}
				else
				{
					i1=iIndex2;
					i2=iIndex3;
					i3=iIndex1;
				}
			}
			else
			{
				i1=iIndex1;
				i2=iIndex2;
				i3=iIndex3;
			}
		}
		bool operator !=(STri& op){if(op.i1!=i1 || op.i2!=i2 || op.i3!=i3) return true; return false;}
		bool operator <(STri& op)
		{
			if(op.i1!=i1)
				return i1<op.i1;
			if(op.i2!=i2)
				return i2<op.i2;
			return i3<op.i3;
		}
		int i1,i2,i3;
	};

	//returns current vertex count
	static int extractOctree(OctreeNode* pNode, int iVertexCount, int* aiIndexTable, Ogre::Vector3* aNewVertices)
	{
		for(std::list<int>::const_iterator it=pNode->liIndices.begin();
			it!=pNode->liIndices.end(); it++)
			aiIndexTable[*it]=iVertexCount;
		aNewVertices[iVertexCount++]=pNode->vPos;
		for(int iSubNode=0; iSubNode<8; iSubNode++)
			if(pNode->aSubNodes[iSubNode])
			{
				iVertexCount=extractOctree(pNode->aSubNodes[iSubNode], iVertexCount, aiIndexTable, aNewVertices);
				delete pNode->aSubNodes[iSubNode];
				pNode->aSubNodes[iSubNode] = nullptr;
			}
		return iVertexCount;
	}


#define IS_IN_BOX(v1,v2,d) ((v1.x<=v2.x+d) && (v1.x>=v2.x-d) && (v1.y<=v2.y+d) && (v1.y>=v2.y-d) && (v1.z<=v2.z+d) && (v1.z>=v2.z-d))

#define EIGHTH_SPACE_INDEX(v1,v2) (((v1.x>v2.x)?4:0)+((v1.y>v2.y)?2:0)+((v1.z>v2.z)?1:0))

public:
	static void mergeVertices(std::vector<Ogre::Vector3>& vertices, std::vector<unsigned int>& indices, float fMergeDist)
	{
		//const float fMergeDist=1e-3f;

		OctreeNode root;
		root.vPos=vertices[0];
		int iVertex=0;
		int numAdded = 0;
		
		for(;iVertex<(int)vertices.size(); iVertex++)
		{
			OctreeNode* pCurrNode=&root;
			while(true)
			{
				if(IS_IN_BOX(vertices[iVertex], pCurrNode->vPos, fMergeDist))
				{
					pCurrNode->liIndices.push_back(iVertex);
					break;
				}
				else
				{//vertex is not in merge distance to this node
					int iSubNode=EIGHTH_SPACE_INDEX(pCurrNode->vPos, vertices[iVertex]);
					if(pCurrNode->aSubNodes[iSubNode])
						//proceed deeper into the tree
						pCurrNode=pCurrNode->aSubNodes[iSubNode];
					else
					{//there is no branch so make one
						pCurrNode->aSubNodes[iSubNode]=new OctreeNode;
						pCurrNode=pCurrNode->aSubNodes[iSubNode];
						pCurrNode->vPos=vertices[iVertex];
						numAdded++;
					}
				}//pCurrNode is now one level lower in the tree
			}
		}
		int* aiIndexTable=new int[vertices.size()];		//maps old indices to new 		
		Ogre::Vector3* aNewVertices=new Ogre::Vector3[vertices.size()];
		//extract indextable and vertex list
		int nNewVertices=extractOctree(&root, 0, aiIndexTable, aNewVertices);
		for (unsigned int iIndex=0; iIndex< indices.size(); ++iIndex)
		{
			assert(indices[iIndex] < (int)indices.size());
			assert(indices[iIndex] >= 0);
			indices[iIndex] = aiIndexTable[indices[iIndex]];
		}
		
		vertices.resize(nNewVertices);
		for(iVertex=0; iVertex<nNewVertices; iVertex++)
			vertices[iVertex]=aNewVertices[iVertex];
		
		delete[] aiIndexTable;
		delete[] aNewVertices;
		
		//search for duplicated and degenerate tris
		std::vector<STri> vTris;
		vTris.resize(indices.size() / 3);
		int nTrisCopied=0;
		int iTri=0;
		for(; iTri<(int)indices.size() / 3; iTri++)
		{//check if this tri is degenerate
			int index1=indices[iTri*3+0],
				index2=indices[iTri*3+1],
				index3=indices[iTri*3+2];
			if(index1==index2 || index3==index2 || index1==index3)
				//degenerate tri: two or more vertices are the same
				continue;
			vTris[nTrisCopied++]=STri(index1,index2,index3);
		}
		vTris.resize(nTrisCopied);
		std::sort(vTris.begin(), vTris.end());//sort tris to find duplicates easily
		nTrisCopied=0;
		STri lastTri;
		for(iTri=0; iTri<(int)vTris.size(); iTri++)
		{
			if(lastTri!=vTris[iTri])
			{
				indices[nTrisCopied*3+0]=vTris[iTri].i1;
				indices[nTrisCopied*3+1]=vTris[iTri].i2;
				indices[nTrisCopied*3+2]=vTris[iTri].i3;
				lastTri=vTris[iTri];
				nTrisCopied++;
			}
		}
		indices.resize(nTrisCopied*3);
	}
};