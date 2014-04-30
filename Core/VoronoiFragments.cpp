
#include "VoronoiFragments.h"

VoronoiFragments::VoronoiFragments(const std::vector<Ogre::Vector3>& fragmentPoints)
{
    if (!fragmentPoints.empty())
    {
        m_AABB.min = fragmentPoints[0];
        m_AABB.max = fragmentPoints[0];
        m_FragmentPoints.clear();
        int index = 0;
        for (auto i = fragmentPoints.begin(); i != fragmentPoints.end(); i++)
        {
            m_FragmentPoints.push_back(new PointBVH(*i, index++));
            m_AABB.merge(AABB(*i, *i));
        }
    }
    std::vector<PointBVH*> pointsCopy;
    pointsCopy.insert(pointsCopy.end(), m_FragmentPoints.begin(), m_FragmentPoints.end());
    m_BVH = BVHNode<AABB, PointBVH>::create(pointsCopy, 0, m_FragmentPoints.size(), 0);
    m_CurrentFragmentIndex = 0;
}

VoronoiFragments::~VoronoiFragments()
{
    delete m_BVH;
    for (auto i = m_FragmentPoints.begin(); i != m_FragmentPoints.end(); i++)
        delete *i;
}

void VoronoiFragments::setFragment(int index)
{
    m_CurrentFragmentIndex = index;
    m_CutPlanes.resize(m_FragmentPoints.size());
    for (auto i = m_FragmentPoints.begin(); i != m_FragmentPoints.end(); i++)
    {
        m_CutPlanes[(*i)->getIndex()].first = (m_FragmentPoints[m_CurrentFragmentIndex]->getPoint() + m_FragmentPoints[(*i)->getIndex()]->getPoint()) * 0.5f;
        m_CutPlanes[(*i)->getIndex()].second = m_FragmentPoints[(*i)->getIndex()]->getPoint() - m_FragmentPoints[m_CurrentFragmentIndex]->getPoint();
        m_CutPlanes[(*i)->getIndex()].second.normalise();
    }
}

bool VoronoiFragments::getSign(const Ogre::Vector3& point) const
{
    BVH<PointBVH>::ClosestLeafResult result;
    return (m_BVH->getClosestLeaf(point, result) == m_FragmentPoints[m_CurrentFragmentIndex]);
}

void VoronoiFragments::getSample(const Ogre::Vector3& point, Sample& sample) const
{
    BVH<PointBVH>::ClosestLeafResult result;
    m_FragmentPoints[m_CurrentFragmentIndex]->setBlackListed(true);
    if (const PointBVH* closest = m_BVH->getClosestLeaf(point, result))
	{
        const std::pair<Ogre::Vector3, Ogre::Vector3 >& cutPlane = m_CutPlanes[closest->getIndex()];
        sample.normal = cutPlane.second;

        sample.signedDistance = (cutPlane.first - point).dotProduct(sample.normal);
		sample.closestSurfacePos = point + sample.signedDistance * sample.normal;
	}
    m_FragmentPoints[m_CurrentFragmentIndex]->setBlackListed(false);
    sample.signedDistance = std::fabsf(sample.signedDistance);
	if (result.closestPoint.squaredDistance(point) < m_FragmentPoints[m_CurrentFragmentIndex]->getPoint().squaredDistance(point))
        sample.signedDistance = -sample.signedDistance;
}

bool VoronoiFragments::intersectsSurface(const AABB& aabb) const
{
    BVH<PointBVH>::ClosestLeafResult result;
    if (const PointBVH* closest = m_BVH->getClosestLeaf(aabb.getCenter(), result))
	{
		float maxDist = std::sqrtf(aabb.getMaximumSquaredDistance(result.closestPoint));
		AABB candidateAABB = aabb;
		candidateAABB.addEpsilon(maxDist);
        if (closest != m_FragmentPoints[m_CurrentFragmentIndex])
		{
            if (!candidateAABB.containsPoint(m_FragmentPoints[m_CurrentFragmentIndex]->getPoint()))
				return false;

            const std::pair<Ogre::Vector3, Ogre::Vector3 >& cutPlane = m_CutPlanes[closest->getIndex()];
            if (aabb.intersectsPlane(cutPlane.first, cutPlane.second))
				return true;
		}

		// fetch candidate points and test them with exact aabb / plane intersection tests
        std::vector<const PointBVH*> candidates;
        m_FragmentPoints[m_CurrentFragmentIndex]->setBlackListed(true);
		closest->setBlackListed(true);
        m_BVH->getLeaves(candidateAABB, candidates);
        m_FragmentPoints[m_CurrentFragmentIndex]->setBlackListed(false);
		closest->setBlackListed(false);
        if (candidates.empty())
            return false;

        // std::cout << candidates.size() << std::endl;
		for (auto i = candidates.begin(); i != candidates.end(); ++i)
		{
            const std::pair<Ogre::Vector3, Ogre::Vector3 >& cutPlane = m_CutPlanes[(*i)->getIndex()];
            if (aabb.intersectsPlane(cutPlane.first, cutPlane.second))
                return true;
		}
	}
    return false;
}
