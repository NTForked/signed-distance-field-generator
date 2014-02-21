
#pragma once

#include <vector>
#include <algorithm>
#include <queue>
#include "Ray.h"
#include "AABB.h"

#ifdef USE_BOOST_THREADING
#include "boost/thread.hpp"
#endif

template<class LeafType>
class BVH
{
public:
	enum Type
	{
		PRIMITIVE,
		PRIMITIVE_BUCKET,
		NODE
	};
protected:

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

	/// Returns all surfaces that are hit by the ray.
	virtual void rayIntersectAll(const Ray &ray, std::vector<std::pair<const LeafType*, Ray::Intersection>> &intersections) const
	{
		// default implementation
		Ray::Intersection intersection;
		const LeafType *s = rayIntersectClosest(intersection, ray);
		if (s) intersections.push_back(std::make_pair(s, intersection));
	}

	__forceinline Type getType() const { return mType; }

	/// Retrieves the squares distance for this BVH node to the given point.
	virtual float squaredDistance(const Ogre::Vector3& point) const = 0;

	struct ClosestLeafResult
	{
		ClosestLeafResult() : closestDistance(999999.0f) {}
		float closestDistance;
		Ogre::Vector3 closestPoint;
		Ogre::Vector3 normal;
	};
	/// Retrieves the closest surface to point.
	virtual const LeafType* getClosestLeaf(const Ogre::Vector3& point, ClosestLeafResult& result) const = 0;

	/// Retrieves whether the AABB intersects any surfaces.
	virtual bool intersectsAABB(const AABB& aabb) const = 0;

	virtual unsigned int getHeight() const { return 1; }
	virtual float getHeightAvg() const { return 1.0f; }
};

/// Bucket that just stores a list of leaves.
template<class LeafType>
class BVHContainer : public BVH<LeafType>
{
private:
	std::vector<LeafType*> mChildren;

public:
	BVHContainer(typename std::vector<LeafType*>::iterator begin, const typename std::vector<LeafType*>::iterator &end)
	{
		this->mType = BVH<LeafType>::PRIMITIVE_BUCKET;
		mChildren.resize(end-begin);
		typename std::vector<LeafType*>::iterator copyBegin = mChildren.begin();
		for (; begin != end; begin++)
			*(copyBegin++) = *begin;
	}
	virtual ~BVHContainer() {}

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

	void rayIntersectAll(const Ray &ray, std::vector<std::pair<const LeafType*, Ray::Intersection>> &intersections) const override
	{
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
			(*i)->rayIntersectAll(ray, intersections);
	}

	float squaredDistance(const Ogre::Vector3& point) const override
	{
		float minDist = 99999999.0f;
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
		{
			float dist = (*i)->squaredDistance(point);
			if (dist < minDist)
				minDist = dist;
		}
		return minDist;
	}

	const LeafType* getClosestLeaf(const Ogre::Vector3& point, ClosestLeafResult& result) const override
	{
		const LeafType* ret = nullptr;
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
		{
			const LeafType* surface = (*i)->getClosestLeaf(point, result);
			if (surface) ret = surface;
		}
		return ret;
	}

	bool intersectsAABB(const AABB& aabb) const override
	{
		for (auto i = mChildren.begin(); i != mChildren.end(); ++i)
		{
			if ((*i)->intersectsAABB(aabb)) return true;
		}
		return false;
	}
};

/// Inspired by http://graphics.stanford.edu/~boulos/papers/efficient_notes.pdf
template<class BoundingVolume, class LeafType>
class BVHNode : public BVH<LeafType>
{
protected:
	BoundingVolume mBoundingVolume;

	BVH<LeafType> *mChildren[2];

	int mSplitAxis;

	static const int PRIMITIVES_PER_LEAF = 6;

	__forceinline static void updateSplitParameters(const BoundingVolume &objBV, const Ogre::Vector3 &objCenter, const Ogre::Vector3 &pivot, int axis, int &numLeft, float &maxLeft, float &minRight)
	{
		if (objCenter[axis] < pivot[axis])
		{
			maxLeft = std::max(maxLeft, objBV.getMax()[axis]);
			numLeft++;
		}
		else minRight = std::min(minRight, objBV.getMin()[axis]);
	}

	struct PartitionFunctor
	{
		PartitionFunctor(const unsigned int &splitAxis, const float &pivot) : mPivot(pivot), mSplitAxis(splitAxis) {}
		const float mPivot;
		const unsigned int mSplitAxis;
		bool operator()(LeafType *s)
		{
			BoundingVolume bv;
			s->getBoundingVolume(bv);
			return bv.getCenter()[mSplitAxis] < mPivot;
		}
	};

	int partitionSurfaces(std::vector<LeafType*> &surfaces, int left, int right, int axisCounter)
	{
		int numSurfaces = right-left;

		// create bounding volume big enough to contain all surfaces
		std::vector<Ogre::Vector3> extremalPoints;
		for (int i = left; i < right; i++)
			extremalPoints.insert(
			extremalPoints.end(),
			surfaces[i]->getExtremalPoints().begin(),
			surfaces[i]->getExtremalPoints().end());
		mBoundingVolume = BoundingVolume(extremalPoints);
		
		// use spatial median
		Ogre::Vector3 pivot = mBoundingVolume.getCenter();

		/*
		We split the objects along the x, y or z component of pivot.
		To choose the best split axis we consider two goals:
		1. The tree should be balanced, this means both child nodes should optimally contain an equal amount of surface objects.
		2. The bounding volumes of both child nodes should not overlap.
		For each axis we compute two penalties that correspond to these criteria. In the end we combine both penalties in some way and choose the axis with the smallest penalty.
		*/

		int numLeft[3] = {0, 0, 0};
		float maxLeft[3] = {-99999999.0f, -99999999.0f, -99999999.0f};
		float minRight[3] = {99999999.0f, 99999999.0f, 99999999.0f};

		for (int si = left; si < right; si++)
		{
			BoundingVolume bv;
			surfaces[si]->getBoundingVolume(bv);
			Ogre::Vector3 center = bv.getCenter();
			for (int i = 0; i < 3; i++)
				updateSplitParameters(bv, center, pivot, i, numLeft[i], maxLeft[i], minRight[i]);
		}

		float penalty[3];
		// for (int i = 0; i < 3; i++) penalty[i] = (numSurfaces/2 - numLeft[i]) * (numSurfaces/2 - numLeft[i]);
		for (int i = 0; i < 3; i++) penalty[i] = std::max(0.0f, maxLeft[i] - minRight[i]);
		for (int i = 0; i < 3; i++)
			if (numLeft[i] == numSurfaces || numLeft[i] == 0) penalty[i] += 999999.0f;
		// for (int i = 0; i < 3; i++) penalty[i] /= (mBoundingVolume.getMax()[i]-mBoundingVolume.getMin()[i]);

		mSplitAxis = 0;
		if (penalty[1] <= penalty[0] && penalty[1] <= penalty[2]) mSplitAxis = 1;
		if (penalty[2] <= penalty[0] && penalty[2] <= penalty[1]) mSplitAxis = 2;

		// mSplitAxis = axisCounter % 3;

		typename std::vector<LeafType*>::iterator leftIt = surfaces.begin() + left;
		typename std::vector<LeafType*>::iterator rightIt = surfaces.begin() + right;
		typename std::vector<LeafType*>::iterator splitIt = std::partition(leftIt, rightIt, PartitionFunctor(mSplitAxis, pivot[mSplitAxis]));
		int mid = (int)(splitIt-surfaces.begin());

		// if one list is empty, fill it with some objects that fulfil the minimum criterium
		if (mid == left)
		{
			int numSwapped = 0;
			for (int i = left; i < right; i++)
			{
				std::vector<Ogre::Vector3> points = surfaces[i]->getExtremalPoints();
				for (auto ip = points.begin(); ip != points.end(); ip++)
				{
					if ((*ip)[mSplitAxis] <= pivot[mSplitAxis])
					{
						std::swap(surfaces[i], surfaces[mid]);
						mid++;
						numSwapped++;
						break;
					}
				}
				if (numSwapped >= numSurfaces/2) break;
			}
		}
		else if (mid == right)
		{
			int numSwapped = 0;
			for (int i = left; i < right; i++)
			{
				std::vector<Ogre::Vector3> points = surfaces[i]->getExtremalPoints();
				for (auto ip = points.begin(); ip != points.end(); ip++)
				{
					if ((*ip)[mSplitAxis] >= pivot[mSplitAxis])
					{
						std::swap(surfaces[i], surfaces[mid]);
						mid--;
						numSwapped++;
						break;
					}
				}
				if (numSwapped >= numSurfaces/2) break;
			}
		}
		vAssert(mid > left && mid < right);

		return mid;
	}

	BVHNode() : mSplitAxis(0)
	{}

public:
	virtual ~BVHNode()
	{
		if (mChildren[0]->getType() != BVH<LeafType>::PRIMITIVE) delete mChildren[0];
		if (mChildren[1]->getType() != BVH<LeafType>::PRIMITIVE) delete mChildren[1];
	}

	static BVH<LeafType>* create(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis)
	{
		if (right - left > PRIMITIVES_PER_LEAF)
			return new BVHNode<BoundingVolume, LeafType>(surfaces, left, right, splitAxis);
		else if (right - left > 1)
			return new BVHContainer<LeafType>(surfaces.begin() + left, surfaces.begin() + right);
		else return surfaces[left];
	}

	/// Constructs a BVH for a list of surfaces - implemented inplace
	BVHNode(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis)
	{
		assert(right-left >= 2);
		this->mType = BVH<LeafType>::NODE;

		int mid = partitionSurfaces(surfaces, left, right, splitAxis);

		mChildren[0] = create(surfaces, left, mid, splitAxis + 1);
		mChildren[1] = create(surfaces, mid, right, splitAxis + 1);
	}

	const LeafType* rayIntersectClosest(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (!mBoundingVolume.rayIntersect(ray))
			return nullptr;
		intersection.t = std::numeric_limits<float>::infinity();
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray); 
		const LeafType *rightHit = mChildren[1-ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray);
		if (rightHit) return rightHit;
		return leftHit;
	}
	const LeafType* rayIntersectUpdate(Ray::Intersection &intersection, const Ray &ray) const override
	{
		if (!mBoundingVolume.rayIntersect(ray, 0.0f, (float)intersection.t))
			return nullptr;
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray); 
		const LeafType *rightHit =  mChildren[1-ray.sign[mSplitAxis]]->rayIntersectUpdate(intersection, ray);
		if (rightHit) return rightHit;
		return leftHit;
	}
	const LeafType* rayIntersectAny(const Ray &ray, float maxDist) const override
	{
		if (!mBoundingVolume.rayIntersect(ray, 0.0f, maxDist))
			return nullptr;
		const LeafType *leftHit = mChildren[ray.sign[mSplitAxis]]->rayIntersectAny(ray, maxDist);
		if (leftHit) return leftHit;
		const LeafType *rightHit =  mChildren[1-ray.sign[mSplitAxis]]->rayIntersectAny(ray, maxDist);
		return rightHit;
	}

	void rayIntersectAll(const Ray &ray, std::vector<std::pair<const LeafType*, Ray::Intersection>> &intersections) const override
	{
		if (!mBoundingVolume.rayIntersect(ray))
			return;
		mChildren[0]->rayIntersectAll(ray, intersections); 
		mChildren[1]->rayIntersectAll(ray, intersections);
	}

	//! Finds leaf that is closest to a given point.
	const LeafType* getClosestLeaf(const Ogre::Vector3& point, ClosestLeafResult& result) const override
	{
		if (!mBoundingVolume.intersectsSphere(point, result.closestDistance)) return nullptr;
		if (mChildren[0]->squaredDistance(point) < mChildren[1]->squaredDistance(point))
		{
			const LeafType* leftHit = mChildren[0]->getClosestLeaf(point, result);
			const LeafType* rightHit = mChildren[1]->getClosestLeaf(point, result);
			if (rightHit) return rightHit;
			return leftHit;
		}
		else
		{
			const LeafType* leftHit = mChildren[1]->getClosestLeaf(point, result);
			const LeafType* rightHit = mChildren[0]->getClosestLeaf(point, result);
			if (rightHit) return rightHit;
			return leftHit;
		}
	}

	// heap traversal is twice as slow than the simple recursive method!
	/*struct CompareBVHsFunctor
	{
		Ogre::Vector3 m_Point;
		CompareBVHsFunctor(const Ogre::Vector3& point) : m_Point(point) {}
		bool operator()(const BVH* bvh1, const BVH* bvh2)
		{
			return bvh1->squaredDistance(m_Point) > bvh2->squaredDistance(m_Point);
		}
	};
	const LeafType* getClosestLeafHeap(const Ogre::Vector3& point, ClosestLeafResult& result)
	{
		CompareBVHsFunctor functor(point);
		std::priority_queue<const BVH<typename LeafType>*, std::vector<const BVH<typename LeafType>* >, CompareBVHsFunctor> heap(functor);
		heap.push(this);
		const LeafType* ret = nullptr;
		while (!heap.empty())
		{
			const BVH<LeafType>* bvh = heap.top();
			heap.pop();
			if (bvh->getType() == NODE)
			{
				BVHNode<BoundingVolume, LeafType>* bvhNode = (BVHNode<BoundingVolume, LeafType>*)bvh;
				if (bvhNode->mBoundingVolume.intersectsSphere(point, result.closestDistance))
				{
					heap.push(bvhNode->mChildren[0]);
					heap.push(bvhNode->mChildren[1]);
				}
			}
			else
			{
				const LeafType* closest = bvh->getClosestLeaf(point, result);
				if (closest) ret = closest;
			}
		}
		return ret;
	}*/

	float squaredDistance(const Ogre::Vector3& point) const override
	{
		return mBoundingVolume.squaredDistance(point);
	}

	bool intersectsAABB(const AABB& aabb) const override
	{
		if (!mBoundingVolume.intersectsAABB(aabb)) return false;
		bool leftHit = mChildren[0]->intersectsAABB(aabb);
		if (leftHit) return true;
		return  mChildren[1]->intersectsAABB(aabb);
	}

	virtual unsigned int getHeight() const override { return 1 + std::max(mChildren[0]->getHeight(), mChildren[1]->getHeight()); }
	virtual float getHeightAvg() const override { return 1.0f + (mChildren[0]->getHeight() + mChildren[1]->getHeight()) * 0.5f; }

	__forceinline BoundingVolume getBoundingVolume() const
	{
		return mBoundingVolume;
	}
};

#ifdef USE_BOOST_THREADING

template<class BoundingVolume, class LeafType>
class BVHNodeThreaded : public BVHNode<BoundingVolume, LeafType>
{
private:

	struct Worker
	{
		Worker(std::vector<LeafType*> *_surfaces, int _left, int _right, int _splitAxis, BVHNodeThreaded *_node, int _maxParallelDepth)
			: surfaces(_surfaces), left(_left), right(_right), splitAxis(_splitAxis), maxParallelDepth(_maxParallelDepth), node(_node) {}
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
	virtual ~BVHNodeThreaded() {}
	BVHNodeThreaded(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth)
	{
		this->mType = BVH<LeafType>::NODE;
		assert(right-left >= 2);
		initialize(surfaces, left, right, splitAxis, maxParallelDepth);
	}
	BVHNodeThreaded(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth, boost::thread_group &threads)
	{
		this->mType = BVH<LeafType>::NODE;
		assert(right-left >= 2);
		threads.create_thread(Worker(&surfaces, left, right, splitAxis, this, maxParallelDepth));
	}

	static BVH<LeafType>* create(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth)
	{
		if (right - left > PRIMITIVES_PER_LEAF)
		{
			if (maxParallelDepth > 0)
				return new BVHNodeThreaded<BoundingVolume, LeafType>(surfaces, left, right, splitAxis, maxParallelDepth);
			else
				return new BVHNode<BoundingVolume, LeafType>(surfaces, left, right, splitAxis);
		}
		else if (right - left > 1)
			return new BVHContainer<LeafType>(surfaces.begin() + left, surfaces.begin() + right);
		else return surfaces[left];
	}

	static BVH<LeafType>* create(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth, boost::thread_group &threads)
	{
		if (right - left > PRIMITIVES_PER_LEAF)
		{
			if (maxParallelDepth > 0)
				return new BVHNodeThreaded<BoundingVolume, LeafType>(surfaces, left, right, splitAxis, maxParallelDepth, threads);
			else
				return new BVHNode<BoundingVolume, LeafType>(surfaces, left, right, splitAxis);
		}
		else if (right - left > 1)
			return new BVHContainer<LeafType>(surfaces.begin() + left, surfaces.begin() + right);
		else return surfaces[left];
	}

	void initialize(std::vector<LeafType*> &surfaces, int left, int right, int splitAxis, int maxParallelDepth)
	{
		int mid = partitionSurfaces(surfaces, left, right, splitAxis);

		boost::thread_group threads;
		mChildren[0] = create(surfaces, left, mid, splitAxis + 1, maxParallelDepth-1, threads);
		mChildren[1] = create(surfaces, mid, right, splitAxis + 1, maxParallelDepth-1, threads);
		threads.join_all();		
	}
};

#endif
