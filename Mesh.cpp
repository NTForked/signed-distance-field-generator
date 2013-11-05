
#include <unordered_map>
#include <unordered_set>
#include "Mesh.h"
#include "Surfaces.h"
#include "OgreMath/OgreVector4.h"
#include "ExportOBJ.h"

using std::vector;
using std::unordered_set;

TransformedMesh::TransformedMesh(std::shared_ptr<Mesh> mesh)
	: mMesh(mesh), mPosition(Ogre::Vector3(0,0,0)), mScale(Ogre::Vector3(1,1,1)), mRotation(Ogre::Matrix3::IDENTITY), vertexBufferVS(mesh->vertexBuffer)
{
	// create triangle objects for BVH etc
	triangleSurfaces.clear();
	triangleSurfaces.reserve(mesh->indexBuffer.size() / 3);
	for (unsigned int i = 0; i < mMesh->indexBuffer.size() / 3; i++)
		triangleSurfaces.push_back(TriangleSurface(this, i));
}

void TransformedMesh::computeCache() const
{
	Ogre::Matrix4 modelTransform, modelTransformNoScale;
	modelTransform.makeTransform(mPosition, mScale, mRotation);
	modelTransformNoScale.makeTransform(mPosition, Ogre::Vector3(1,1,1), mRotation);	// for normal transformation

	// not exactly elegant use of homogenous notation but after all it's slightly more efficient to multiply the normals with a 3x3 matrix
	Ogre::Matrix3 normalTransform;
	modelTransformNoScale.extract3x3Matrix(normalTransform);

	// transform vertices with modelViewMatrix which includes translation, rotation and scale
	vertexBufferVS.clear();
	vertexBufferVS.resize(mMesh->vertexBuffer.size());
	auto it = vertexBufferVS.begin();
	for (auto i = mMesh->vertexBuffer.begin(); i != mMesh->vertexBuffer.end(); ++i)
	{
		it->position = modelTransform * i->position;
		//apply only the rotation to the normal
		it->normal = normalTransform * i->normal;
		it->normal.normalise();
		++it;
	}

	vector<unsigned int> bullshitEdgesTriangles;

	triangleDataVS.clear();
	triangleDataVS.resize(mMesh->triangleNormals.size());

	// Maps vertices to triangles that share this vertex.
	vector<std::unordered_set<int> > verticesTotris;
	verticesTotris.resize(vertexBufferVS.size());
	const int numTriangles = triangleDataVS.size();

	int triIndex = 0;
	int numDegeneratedTris = 0;
	for (unsigned int index = 0; index < mMesh->indexBuffer.size(); index += 3)
	{
		// add edges to edge map, and do both sides
		verticesTotris[mMesh->indexBuffer[index]].insert(triIndex);
		verticesTotris[mMesh->indexBuffer[index+1]].insert(triIndex);
		verticesTotris[mMesh->indexBuffer[index+2]].insert(triIndex);

		// recompute triangle normals for maximal accuracy
		TriangleCached &triData = triangleDataVS[triIndex++];

		triData.p1 = vertexBufferVS[mMesh->indexBuffer[index]].position;
		triData.p2 = vertexBufferVS[mMesh->indexBuffer[index+1]].position;
		triData.p3 = vertexBufferVS[mMesh->indexBuffer[index+2]].position;

		Ogre::Vector3 v0 = triData.p2 - triData.p1;
		Ogre::Vector3 v1 = triData.p3 - triData.p1;

		triData.normal = v0.crossProduct(v1);
		triData.normal.normalise();

		// slightly inspired by http://software.intel.com/en-us/articles/interactive-ray-tracing
		// determine dominant axis
		int domAxis = 0;
		if (std::abs(triData.normal.y) >= std::abs(triData.normal.x) && std::abs(triData.normal.y) >= std::abs(triData.normal.z))
			domAxis = 1;
		else if (std::abs(triData.normal.z) >= std::abs(triData.normal.x) && std::abs(triData.normal.z) >= std::abs(triData.normal.y))
			domAxis = 2;
		triData.equationIndex1 = (domAxis+1) % 3;
		triData.equationIndex2 = (domAxis+2) % 3;

		double a = triData.p2[triData.equationIndex1]-triData.p1[triData.equationIndex1];
		double b = triData.p3[triData.equationIndex1]-triData.p1[triData.equationIndex1];
		double c = triData.p2[triData.equationIndex2]-triData.p1[triData.equationIndex2];
		double d = triData.p3[triData.equationIndex2]-triData.p1[triData.equationIndex2];
		double denom = a*d - b*c;
		if (std::abs(denom) < std::numeric_limits<double>::epsilon())
		{
			numDegeneratedTris++;
			denom = std::numeric_limits<double>::epsilon();
			triData.degenerated = true;
			//std::cout << "Degenerated triangle: " << p1 << " " << p2 << " " << p3 << " denom: " << denom << std::endl;
		}
		triData.aInv = static_cast<float>(d/denom);
		triData.bInv = static_cast<float>(-b/denom);
		triData.cInv = static_cast<float>(-c/denom);
		triData.dInv = static_cast<float>(a/denom);
	}
	std::cout << "Found " << numDegeneratedTris << " degenerated triangles." << std::endl;

	std::vector<Ogre::Vector3> angleWeightedVertexNormals;
	angleWeightedVertexNormals.resize(verticesTotris.size());
	for (auto i = angleWeightedVertexNormals.begin(); i != angleWeightedVertexNormals.end(); i++)
		(*i) = Ogre::Vector3(0,0,0);
	triIndex = 0;
	for (unsigned int index = 0; index < mMesh->indexBuffer.size(); index += 3)
	{
		TriangleCached &triData = triangleDataVS[triIndex++];
		Ogre::Vector3 v0 = (triData.p2 - triData.p1).normalisedCopy();
		Ogre::Vector3 v1 = (triData.p3 - triData.p1).normalisedCopy();
		Ogre::Vector3 v2 = (triData.p3 - triData.p2).normalisedCopy();
		angleWeightedVertexNormals[mMesh->indexBuffer[index]] += triData.normal * std::acosf(v0.dotProduct(v1));
		angleWeightedVertexNormals[mMesh->indexBuffer[index+1]] += triData.normal * std::acosf((-v0).dotProduct(v2));
		angleWeightedVertexNormals[mMesh->indexBuffer[index+2]] += triData.normal * std::acosf((-v2).dotProduct(-v1));
	}
	for (auto i = angleWeightedVertexNormals.begin(); i != angleWeightedVertexNormals.end(); ++i)
		i->normalise();

	// Compute edge pseudo normals as a post process
	triIndex = 0;
	bool corruptEdges = false;
	for (unsigned int index = 0; index < mMesh->indexBuffer.size(); index += 3)
	{
		TriangleCached &triData = triangleDataVS[triIndex];

		triData.vertexPseudoNormals[0] = angleWeightedVertexNormals[mMesh->indexBuffer[index]];
		triData.vertexPseudoNormals[1] = angleWeightedVertexNormals[mMesh->indexBuffer[index+1]];
		triData.vertexPseudoNormals[2] = angleWeightedVertexNormals[mMesh->indexBuffer[index+2]];

		for (int i = 0; i < 3; i++)
			triData.edgePseudoNormals[i] = triData.normal;
		int triEdgeCounter[3] = {1, 1, 1};
		for (auto i = verticesTotris[mMesh->indexBuffer[index]].begin(); i != verticesTotris[mMesh->indexBuffer[index]].end(); ++i)
		{
			if (*i != triIndex)
			{
				// check if tri also shares other vertex
				if (verticesTotris[mMesh->indexBuffer[index+1]].find(*i) != verticesTotris[mMesh->indexBuffer[index+1]].end())
				{
					triEdgeCounter[0]++;
					triData.edgePseudoNormals[0] += triangleDataVS[*i].normal;
				}
				if (verticesTotris[mMesh->indexBuffer[index+2]].find(*i) != verticesTotris[mMesh->indexBuffer[index+2]].end())
				{
					triEdgeCounter[2]++;
					triData.edgePseudoNormals[2] += triangleDataVS[*i].normal;
				}
			}
		}
		for (auto i = verticesTotris[mMesh->indexBuffer[index+1]].begin(); i != verticesTotris[mMesh->indexBuffer[index+1]].end(); ++i)
		{
			if (*i != triIndex)
			{
				// check if tri also shares other vertex
				if (verticesTotris[mMesh->indexBuffer[index+2]].find(*i) != verticesTotris[mMesh->indexBuffer[index+2]].end())
				{
					triEdgeCounter[1]++;
					triData.edgePseudoNormals[1] += triangleDataVS[*i].normal;
				}
			}
		}
		for (int i = 0; i < 3; i++)
			triData.edgePseudoNormals[1].normalise();
		for (int i = 0; i < 3; i++)
		{
			if (triEdgeCounter[i] > 2)
			{
				corruptEdges = true;
				triData.dirtyEdgeNormals[i] = true;
				bullshitEdgesTriangles.push_back(mMesh->indexBuffer[index]);
				bullshitEdgesTriangles.push_back(mMesh->indexBuffer[index+1]);
				bullshitEdgesTriangles.push_back(mMesh->indexBuffer[index+2]);
			}
		}
		triIndex++;
	}

	if (corruptEdges)
		std::cout << "Warning: Mesh contains edges that are shared between more that 2 triangles!" << std::endl;

	if (!bullshitEdgesTriangles.empty())
	{
		std::vector<GenericVertex<MaterialID> > vertexBuffer;
		vertexBuffer.resize(mMesh->vertexBuffer.size());
		for (int i = 0; i < mMesh->vertexBuffer.size(); i++)
			vertexBuffer[i].position = mMesh->vertexBuffer[i].position;
		ExportOBJ::writeMesh("BullshitEdges", vertexBuffer, mMesh->triangleNormals, bullshitEdgesTriangles);
	}

	for (auto i = triangleSurfaces.begin(); i != triangleSurfaces.end(); i++)
		i->updateBoundingVolume();
}