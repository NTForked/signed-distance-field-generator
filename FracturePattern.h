
#pragma once

#include <memory>
#include <sstream>
#include "SignedDistanceField.h"
#include "OctreeSDF.h"
#include "TransformSDF.h"
#include "FractalNoisePlaneSDF.h"
#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreMatrix4.h"

class FracturePattern
{
protected:
	std::vector<std::shared_ptr<OctreeSDF> > m_PatternPieces;
public:
	virtual ~FracturePattern() {}
	void transformPattern(const Ogre::Matrix4& matrix)
	{
		for (auto i = m_PatternPieces.begin(); i != m_PatternPieces.end(); ++i)
		{
			TransformSDF transformedSDF(*i, matrix);
			*i = OctreeSDF::sampleSDF(&transformedSDF, (*i)->getHeight());
		}
	}

	std::vector<std::shared_ptr<SignedDistanceField3D> > fractureSDF(OctreeSDF* sdf)
	{
		std::vector<std::shared_ptr<SignedDistanceField3D> > outPieces;
		for (auto i = m_PatternPieces.begin(); i != m_PatternPieces.end(); ++i)
		{
			std::shared_ptr<OctreeSDF> pieceClone = (*i)->clone();
			pieceClone->intersect(sdf);
			sdf->subtract(pieceClone.get());
			outPieces.push_back(pieceClone);
		}
		return outPieces;
	}
};

class SphericalFracturePattern : public FracturePattern
{
protected:
	void splitRecursive(const Ogre::Vector3& pointOfImpact, int maxSplitDepth, std::shared_ptr<OctreeSDF> sdf, std::vector<std::shared_ptr<OctreeSDF> >& outPieces)
	{
		if (maxSplitDepth <= 0) return;
		Ogre::Vector3 planeVec1 = Ogre::Vector3(Ogre::Math::RangeRandom(-1.0f, 1.0f), Ogre::Math::RangeRandom(-1.0f, 1.0f), Ogre::Math::RangeRandom(-1.0f, 1.0f));
		planeVec1.normalise();
		Ogre::Vector3 planePos = sdf->getCenterOfMass();
		Ogre::Vector3 planeVec2 = (planePos - pointOfImpact).normalisedCopy();
		Ogre::Vector3 planeNormal = planeVec1.crossProduct(planeVec2);
		Ogre::Quaternion planeOrientation = Ogre::Vector3(0, 0, 1).getRotationTo(planeNormal);
		auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, planeOrientation, planePos);
		std::cout << "[SphericalFracturePattern] Processing cut plane with normal " << planeNormal << std::endl;
		// auto cutPlaneSDF = OctreeSDF::sampleSDF(fractalNoiseSDF.get(), sdf->getAABB(), sdf->getHeight());
		auto newPiece = sdf->clone();
		// newPiece->intersectAlignedOctree(cutPlaneSDF.get());
		newPiece->intersect(fractalNoiseSDF.get());
		newPiece->simplify();
		sdf->subtractAlignedOctree(newPiece.get());
		sdf->simplify();
		outPieces.push_back(newPiece);
		splitRecursive(pointOfImpact, maxSplitDepth - 1, sdf, outPieces);
		splitRecursive(pointOfImpact, maxSplitDepth - 1, newPiece, outPieces);
	}
public:
	SphericalFracturePattern(int octreeDepth, int numRecursiveSplits)
	{
		SphereSDF sdf(Ogre::Vector3(0, 0, 0), 1.0f);
		auto sphereSDF = OctreeSDF::sampleSDF(&sdf, octreeDepth);
		m_PatternPieces.push_back(sphereSDF);
		splitRecursive(Ogre::Vector3(0, 0, 0), numRecursiveSplits, sphereSDF, m_PatternPieces);

		std::cout << "[SphericalFracturePattern] Finished!" << std::endl;
		int i = 0;
		for (auto iPiece = m_PatternPieces.begin(); iPiece != m_PatternPieces.end(); iPiece++)
		{
			std::stringstream ss;
			ss << "SphericalFracturePattern" << (i++);
			SDFManager::exportSampledSDFAsMesh(ss.str(), *iPiece);
		}
	}
};