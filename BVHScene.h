
#pragma once

#include "Prerequisites.h"
#include "BVH.h"
#include "Mesh.h"
#include <vector>
#include "Surfaces.h"

using std::vector;
using std::shared_ptr;

class BVHScene
{
protected:
	shared_ptr<BVH<Surface> > m_BVH;
	vector<shared_ptr<TransformedMesh> > m_Meshes;

public:
	BVHScene() {}
	~BVHScene()
	{
		// do this explicitly here to ensure that the BVH is destroyed before the meshes
		if (m_BVH) m_BVH = nullptr;
	}

	void clearMeshes()
	{
		m_Meshes.clear();
	}

	void addMesh(shared_ptr<TransformedMesh> mesh)
	{
		m_Meshes.push_back(mesh);
	}

	template<class BV>
	void generateBVH()
	{
		vector<Surface*> surfaces;
		for (auto iMesh = m_Meshes.begin(); iMesh != m_Meshes.end(); ++iMesh)
		{
			for (auto iTri = (*iMesh)->triangleSurfaces.begin(); iTri != (*iMesh)->triangleSurfaces.end(); ++iTri)
				surfaces.push_back(&(*iTri));
		}
#ifdef USE_BOOST_THREADING
		const int numThreads = 16;
		m_BVH = shared_ptr<BVH<Surface> >(BVHNodeThreaded<BV, Surface>::create(surfaces, 0, (int)surfaces.size(), 0, static_cast<int>(std::log((double)numThreads) / std::log(2.0))));
#else
		m_BVH = shared_ptr<BVH<Surface> >(BVHNode<BV, Surface>::create(surfaces, 0, (int)surfaces.size(), 0));
#endif
	}

	inline BVH<Surface>* getBVH()
	{
		return m_BVH.get();
	}
	inline BVH<Surface>* getBVH() const
	{
		return m_BVH.get();
	}

};