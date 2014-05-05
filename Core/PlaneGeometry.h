#ifndef PLANEGEOMETRY_H
#define PLANEGEOMETRY_H

#include "SolidGeometry.h"

class PlaneGeometry : public SolidGeometry
{
protected:
    Ogre::Vector3 m_Pos;
    Ogre::Vector3 m_Normal;

public:
    PlaneGeometry(const Ogre::Vector3& pos, const Ogre::Vector3& normal)
    {
        m_Pos = pos;
        m_Normal = normal;
        m_Normal.normalise();
    }

    virtual void getSample(const Ogre::Vector3& point, Sample& sample) const override
    {
        sample.signedDistance = m_Normal.dotProduct(m_Pos - point);
        sample.normal = m_Normal;
        sample.closestSurfacePos = point + m_Normal * sample.signedDistance;
    }

    virtual bool getSign(const Ogre::Vector3& point) const override
    {
        return m_Normal.dotProduct(m_Pos - point) > 0;
    }

    virtual bool intersectsSurface(const AABB& aabb) const override
    {
        unsigned char mask = 0;
        for (int i = 0; i < 8; i++)
        {
            mask |= (1 << (unsigned char)getSign(aabb.getCorner(i)));
        }
        return mask == 3;
    }

    virtual bool raycastClosest(const Ray& ray, Sample& sample) const override
    {
        Ray::Intersection intersection;
        bool backface = false;
        if (ray.intersectPlane(intersection, m_Normal, m_Pos, backface))
        {
            sample.closestSurfacePos = ray.origin + ray.direction * intersection.t;
            sample.signedDistance = intersection.t;
            sample.normal = m_Normal;
            if (!getSign(ray.origin))
                sample.signedDistance *= -1.0f;
            return true;
        }
        return false;
    }

    virtual AABB getAABB() const override
    {
        return AABB(Ogre::Vector3(-1, -1, -1), Ogre::Vector3(1, 1, 1));
    }
};

#endif // PLANEGEOMETRY_H
