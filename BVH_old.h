
#pragma once

#include <vector>
#include <algorithm>
#include "boost/thread.hpp"
#include "Ray.h"
#include "AABB.h"

template<class LeafType>
class BVH
{
protected:
	enum Type
	{
		PRIMITIVE,
		PRIMITIVE_BUCKET,
		NODE
	};
	Type mType;

public:
	BVH() : mType(PRIMITIVE) {}
	virtual ~BVH() {}

	/// Returns the leaf surface with the closest distance that is hit by the ray, if it exists (otherwise nullptr).
	virtual const LeafType* rayIntersectClosest(Ray::Intersection &intersection, const Ray &ray) const = 0;

	/// Returns a surface if the ray hits the surface at a closer distance than intersection.t and updates intersection.
	virtual const LeafType* rayIntersectUpdate(Ray::Intersection &intersection, const Ray &ray) const
	{
		// default implementation
		Ray::Intersection intersection2;
		const LeafType *s = rayIntersectClosest(intersection2, ray);
		if (s && intersection2.t < intersection.t)
		{
			intersection = intersection2;
			return s;
		}
		return nullptr;
	}

	/// Returns any leaf surface that is hit by the ray, if it exists (otherwise nullptr).
	virtual const LeafType* rayIntersectAny(const Ray &ray, float maxDist) const
	{
		// default implementation
		Ray::Intersection intersection;
		intersection.t = maxDist;
		return rayIntersectUpdate(intersection, ray);
	}

	__forceinline Type getType() const { return mType; }
};

template<class LeafType>
class BVHContainer : public BVH<LeafType>
{
private:
	std::vector<LeafType*> mChildren;

public:
	BVHContainer(typename std::vector<typename LeafType*>::iterator begin, const typename std::vector<typename LeafType*>::iterator &end)
	{
		mType = PRIMITIVE_BUCKET;
		mChildren.resize(end-begin);
		std::vector<LeafType*>::iterator copyBegin = mChildren.begin();
		for (; begin != end; begin++)
			*(copyBegin++) = *begin;
	}
	~BVHContainer() {}

	const LeafType* rayIntersectClosest(Ray::Intersection &intersection, const Ray &ray) const override
	{
		assert(false);
		Ray::Intersection intersection2;
		intersection.t = std::numeric_limits<float>::infinity();
		return rayIntersectUpdate(intersection, ray);
	}

	const LeafType* rayIntersectUpdate(Ray::Intersection &intersection, const Ray &ray) const override
	{
		const LeafType *closest = nullptr;
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
		{
			const LeafType *s = (*i)->rayIntersectUpdate(intersection, ray);
			if (s) closest = s;
		}
		return closest;
	}

	const LeafType* rayIntersectAny(const Ray &ray, float maxDist) const override
	{
		Ray::Intersection intersection;
		intersection.t = maxDist;
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
		{
			if (const LeafType *s = (*i)->rayIntersectUpdate(intersection, ray)) return s;
		}
		return nullptr;
	}
};

/// Inspired by http://graphics.stanford.edu/~boulos/papers/efficient_notes.pdf
template<class LeafType>
class BVHNode : public BVH<LeafType>
{
protected:
	AABB mAABB;

	BVH<LeafType> *mChildren[2];

	int mSplitAxis;

	static const int PRIMITIVES_PER_LEAF = 3;

	// node height, useful for algorithm analysis
	unsigned int mHeight;
	float mHeightAvg;

	__forceinline static void updateSplitParameters(const AABB &objAABB, const Ogre::Vector3 &objCenter, const Ogre::Vector3 &pivot, int axis, int &numLeft, float &maxLeft, float &minRight)
	{
		if (objCenter[axis] < pivot[axis])
		{
			maxLeft = std::max(maxLeft, objAABB.max[axis]);
			numLeft++;
		}
		else minRight = std::min(minRight, objAABB.min[axis]);
	}

	int partitionSurfaces(std::vector<LeafType*> &surfaces, int left, int right, int axisCounter)
	{
		int numSurfaces = right-left;
		mAABB = surfaces[left]->getAABB();
		// create bounding box big enough to contain all surfaces
		for (int i = left+1; i < right; i++)
			mAABB = AABB(mAABB, surfaces[i]->getAABB());

		// use spatial median
		Ogre::Vector3 pivot = mAABB.getCenter();

		/*
		We split the objects along the x, y or z component of pivot.
		To choose the best split axis we consider that the bounding volumes of both child nodes should not overlap too much.
		For each axis we compute a penalty and choose the axis with the lowest penalty.
		*/

		int numLeft[3] = {0, 0, 0};
		float maxLeft[3] = {-99999999.0f, -99999999.0f, -99999999.0f};
		float minRight[3] = {99999999.0f, 99999999.0f, 99999999.0f};

		for (int si = left; si < right; si++)
		{
			AABB aabb = surfaces[si]->getAABB();
			Ogre::Vector3 center = aabb.getCenter();
			for (int i = 0; i < 3; i++)
				updateSplitParameters(aabb, center, pivot, i, numLeft[i], maxLeft[i], minRight[i]);
		}

		float overlapPenalty[3];
		for (int i = 0; i < 3; i++) overlapPenalty[i] = std::max(0.0f, maxLeft[i] - minRight[i]);
		for (int i = 0; i < 3; i++) overlapPenalty[i] /= (mAABB.max[i]-mAABB.min[i]);

		mSplitAxis = 0;
		float combinedPenalty[3];
		for (int i = 0; i < 3; i++) combinedPenalty[i] = overlapPenalty[i];
		if (combinedPenalty[1] <= combinedPenalty[0] && combinedPenalty[1] <= combinedPenalty[2]) mSplitAxis = 1;
		if (combinedPenalty[2] <= combinedPenalty[0] && combinedPenalty[2] <= combinedPenalty[1]) mSplitAxis = 2;

		// ... or just round robin
		//mSplitAxis = axisCounter % 3;

		auto leftIt = surfaces.begin() + left;
		auto rightIt = surfaces.begin() + right;
		auto splitIt = std::partition(leftIt, rightIt, [&pivot, this](LeafType *s) { return s->getAABB().getCenter()[mSplitAxis] < pivot[mSplitAxis]; });
		int mid = splitIt-surfaces.begin();

		// if one list is empty, fill it some objects that fulfil the minimum criterium
		if (mid == left)
		{
			//std::cout << "mid == left" << std::endl;
			int numSwapped = 0;
			for (int i = left; i < right; i++)
			{
				if (surfaces[i]->getAABB().getCenter()[mSplitAxis] <= pivot[mSplitAxis] + std::numeric_limits<float>::epsilon())
				{
					std::swap(surfaces[i], surfaces[mid]);
					mid++;
					numSwapped++;
					if (numSwapped >= numSurfaces/2) break;
				}
			}
		}
		assert(mid > left && mid < right);
		return mid;
	}

	BVHNode() {}

public:
	virtual ~BVHNode()
	{
		if (mChildren[0]->getType() != PRIMITIVE) delete mChildren[0];
		if (mChildren[1]->getType() != PRIMITIVE) delete mChildren[1];
	}

	/// Constructs a BVH for a list of surfaces - implemented inplace
	BVHNode(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis)
	{
		assert(right-left >= 2);
		mType = NODE;
		mHeight = 0;
		mHeightAvg = 0;

		int mid = partitionSurfaces(surfaces, left, right, splitAxis);

		int heightLeft = 1, heightRight = 1;
		float heightAvgLeft = 1.0f, heightAvgRight = 1.0f;
		if (mid-left > PRIMITIVES_PER_LEAF)
		{
			mChildren[0] = new BVHNode<LeafType>(surfaces, left, mid, splitAxis+1);
			heightLeft = ((BVHNode*)mChildren[0])->mHeight;
			heightAvgLeft = ((BVHNode*)mChildren[0])->mHeightAvg;
		}
		else if (mid-left > 1) mChildren[0] = new BVHContainer<LeafType>(surfaces.begin()+left, surfaces.begin()+mid);
		else mChildren[0] = surfaces[left];

		if (right-mid > PRIMITIVES_PER_LEAF)
		{
			mChildren[1] = new BVHNode<LeafType>(surfaces, mid, right, splitAxis+1);
			heightRight = ((BVHNode*)mChildren[1])->mHeight;
			heightAvgRight = ((BVHNode*)mChildren[1])->mHeightAvg;
		}
		else if (right-mid > 1) mChildren[1] = new BVHContainer<LeafType>(surfaces.begin()+mid, surfaces.begin()+right);
		else mChildren[1] = surfaces[right-1];

		mHeight = 1 + std::max(heightLeft, heightRight);
		mHeightAvg = 1 + (heightAvgLeft + heightAvgRight) * 0.5f;
	}

	const LeafType* rayIntersectClosest(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (!ray.intersectAABB(&mAABB.min))
			return nullptr;
		intersection.t = std::numeric_limits<float>::infinity();
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray); 
		const LeafType *rightHit = mChildren[1-ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray);
		if (rightHit) return rightHit;
		return leftHit;
	}
	const LeafType* rayIntersectUpdate(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (!ray.intersectAABB(&mAABB.min, 0, intersection.t))
			return nullptr;
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray); 
		const LeafType *rightHit =  mChildren[1-ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray);
		if (rightHit) return rightHit;
		return leftHit;
	}
	const LeafType* rayIntersectAny(const Ray &ray, float maxDist) const override
	{
		if (!ray.intersectAABB(&mAABB.min, 0, maxDist))
			return nullptr;
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectAny(ray, maxDist);
		if (leftHit) return leftHit;
		const LeafType *rightHit =  mChildren[1-ray.sign[mSplitAxis]]->rayIntersectAny(ray, maxDist);
		return rightHit;
	}

	unsigned int getHeight() { return mHeight; }
	float getHeightAvg() { return mHeightAvg; }
};

template<class LeafType>
class BVHNodeThreaded : public BVHNode<LeafType>
{
private:

	struct Worker
	{
		Worker(std::vector<LeafType*> *_surfaces, int _left, int _right, int _splitAxis, BVHNodeThreaded *_node, int _maxParallelDepth)
			: surfaces(_surfaces), left(_left), right(_right), splitAxis(_splitAxis), node(_node), maxParallelDepth(_maxParallelDepth) {}
		std::vector<LeafType*> *surfaces;
		int left, right;
		int splitAxis;
		int maxParallelDepth;
		BVHNodeThreaded *node;
		void operator()()
		{
			node->initialize(*surfaces, left, right, splitAxis, maxParallelDepth);
		}
	};

public:
	~BVHNodeThreaded() {}
	BVHNodeThreaded(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth)
	{
		mType = NODE;
		mHeight = 0;
		mHeightAvg = 0;
		assert(right-left >= 2);
		initialize(surfaces, left, right, splitAxis, maxParallelDepth);
	}
	BVHNodeThreaded(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth, boost::thread_group &threads)
	{
		mType = NODE;
		mHeight = 0;
		mHeightAvg = 0;
		assert(right-left >= 2);
		threads.create_thread(Worker(&surfaces, left, right, splitAxis, this, maxParallelDepth));
	}

	void initialize(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth)
	{
		int mid = partitionSurfaces(surfaces, left, right, splitAxis);

		int heightLeft = 1, heightRight = 1;
		float heightAvgLeft = 1.0f, heightAvgRight = 1.0f;
		boost::thread_group threads;
		if (mid-left > PRIMITIVES_PER_LEAF)
		{
			if (maxParallelDepth > 0)
				mChildren[0] = new BVHNodeThreaded<LeafType>(surfaces, left, mid, splitAxis+1, maxParallelDepth-1, threads);
			else
				mChildren[0] = new BVHNode<LeafType>(surfaces, left, mid, splitAxis+1);
		}
		else if (mid-left > 1) mChildren[0] = new BVHContainer<LeafType>(surfaces.begin()+left, surfaces.begin()+mid);
		else mChildren[0] = surfaces[left];

		if (right-mid > PRIMITIVES_PER_LEAF)
		{
			if (maxParallelDepth > 0)
				mChildren[1] = new BVHNodeThreaded<LeafType>(surfaces, mid, right, splitAxis+1, maxParallelDepth-1, threads);
			else
				mChildren[1] = new BVHNode<LeafType>(surfaces, mid, right, splitAxis+1);
		}
		else if (right-mid > 1) mChildren[1] = new BVHContainer<LeafType>(surfaces.begin()+mid, surfaces.begin()+right);
		else mChildren[1] = surfaces[mid];

		threads.join_all();
		if (mChildren[0]->getType() == NODE)
		{
			heightLeft = ((BVHNode*)mChildren[0])->getHeight();
			heightAvgLeft = ((BVHNode*)mChildren[0])->getHeightAvg();
		}
		if (mChildren[1]->getType() == NODE)
		{
			heightRight = ((BVHNode*)mChildren[1])->getHeight();
			heightAvgRight = ((BVHNode*)mChildren[1])->getHeightAvg();
		}
		mHeight = 1 + std::max(heightLeft, heightRight);
		mHeightAvg = 1 + (heightAvgLeft + heightAvgRight) * 0.5f;
		
	}
};