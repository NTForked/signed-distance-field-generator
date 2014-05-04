
#pragma once

#include <memory>
#include <sstream>
#include "SolidGeometry.h"
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
	void resamplePieces(const AABB& aabb, const Ogre::Matrix4& matrix)
	{
		for (auto i = m_PatternPieces.begin(); i != m_PatternPieces.end(); ++i)
		{
			TransformSDF transformedSDF(*i, matrix);
			*i = OctreeSDF::sampleSDF(&transformedSDF, aabb, (*i)->getHeight());
		}
		// for (auto iPiece = m_PatternPieces.begin(); iPiece != m_PatternPieces.end(); iPiece++)
		//	(*iPiece)->generateTriangleCache();
	}

	std::vector<std::shared_ptr<OctreeSDF> > fractureSDF(OctreeSDF* sdf, const Ogre::Matrix4& patternTransform)
	{
		std::cout << "Resampling pieces..." << std::endl;
		resamplePieces(sdf->getAABB(), patternTransform);
		exportPatternPieces("SphericalPatternMUH");
		std::cout << "Fracturing..." << std::endl;
		std::vector<std::shared_ptr<OctreeSDF> > outPieces;
		for (auto i = m_PatternPieces.begin(); i != m_PatternPieces.end(); ++i)
		{
			std::shared_ptr<OctreeSDF> pieceClone = (*i)->clone();
			pieceClone->intersectAlignedOctree(sdf);
			pieceClone->simplify();
			sdf->subtractAlignedOctree(pieceClone.get());
			sdf->simplify();
			outPieces.push_back(pieceClone);
		}
		return outPieces;
	}

	void exportPatternPieces(const std::string& patternName)
	{
		int i = 0;
		for (auto iPiece = m_PatternPieces.begin(); iPiece != m_PatternPieces.end(); iPiece++)
		{
			std::stringstream ss;
			ss << patternName << (i++);
			SDFManager::exportSampledSDFAsMesh(ss.str(), *iPiece);
		}
	}

	static void splitRecursiveRandom(int maxSplitDepth, std::shared_ptr<OctreeSDF> sdf, std::vector<std::shared_ptr<OctreeSDF> >& outPieces)
	{
		if (maxSplitDepth <= 0) return;
		Ogre::Vector3 planeNormal = Ogre::Vector3(Ogre::Math::RangeRandom(-1.0f, 1.0f), Ogre::Math::RangeRandom(-1.0f, 1.0f), Ogre::Math::RangeRandom(-1.0f, 1.0f));
		planeNormal.normalise();
		Ogre::Quaternion planeOrientation = Ogre::Vector3(0, 0, 1).getRotationTo(planeNormal);
		float planeSize = (sdf->getAABB().getMax() - sdf->getAABB().getMin()).x + 0.1f;	// Octree aabb is a cube
		auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(planeSize, 1.0f, 0.1f, planeOrientation, sdf->getCenterOfMass());
		std::cout << "[SphericalFracturePattern] Processing cut plane with size " << planeSize << " and normal " << planeNormal << std::endl;
		// auto cutPlaneSDF = OctreeSDF::sampleSDF(fractalNoiseSDF.get(), sdf->getAABB(), sdf->getHeight());
		auto newPiece = sdf->clone();
		// newPiece->intersectAlignedOctree(cutPlaneSDF.get());
		newPiece->intersect(fractalNoiseSDF.get());
		newPiece->simplify();
		sdf->subtract(fractalNoiseSDF.get());
		//sdf->subtractAlignedOctree(newPiece.get());
		sdf->simplify();
		outPieces.push_back(newPiece);
		splitRecursiveRandom(maxSplitDepth - 1, sdf, outPieces);
		splitRecursiveRandom(maxSplitDepth - 1, newPiece, outPieces);
	}

	static void splitRecursivePointOfImpact(const Ogre::Vector3& pointOfImpact, int maxSplitDepth, std::shared_ptr<OctreeSDF> sdf, std::vector<std::shared_ptr<OctreeSDF> >& outPieces)
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
		sdf->subtract(fractalNoiseSDF.get());
		//sdf->subtractAlignedOctree(newPiece.get());
		sdf->simplify();
		outPieces.push_back(newPiece);
		splitRecursivePointOfImpact(pointOfImpact, maxSplitDepth - 1, sdf, outPieces);
		splitRecursivePointOfImpact(pointOfImpact, maxSplitDepth - 1, newPiece, outPieces);
	}
};

class SphericalFracturePattern : public FracturePattern
{
public:
	SphericalFracturePattern(int octreeDepth, int numRecursiveSplits)
	{
		SphereGeometry sdf(Ogre::Vector3(0, 0, 0), 1.0f);
		auto sphereSDF = OctreeSDF::sampleSDF(&sdf, octreeDepth);
		m_PatternPieces.push_back(sphereSDF);
		splitRecursivePointOfImpact(Ogre::Vector3(0, 0, 0), numRecursiveSplits, sphereSDF, m_PatternPieces);
		for (auto iPiece = m_PatternPieces.begin(); iPiece != m_PatternPieces.end(); iPiece++)
			(*iPiece)->generateTriangleCache();
	}
};