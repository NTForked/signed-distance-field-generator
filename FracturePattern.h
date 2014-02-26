
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
public:
	SphericalFracturePattern(int numCutPlanes, int octreeDepth, int maxNumPieces)
	{
		SphereSDF sdf(Ogre::Vector3(0, 0, 0), 1.0f);
		auto sphereSDF = OctreeSDF::sampleSDF(&sdf, octreeDepth);
		m_PatternPieces.push_back(sphereSDF);

		for (float h = Ogre::Math::PI / 4 / numCutPlanes; h<Ogre::Math::PI / 2; h += Ogre::Math::PI / 2 / numCutPlanes)
		{
			for (float v = 0; v < 2 * Ogre::Math::PI; v += Ogre::Math::PI / 2 / (numCutPlanes*Ogre::Math::Cos(h)))
			{
				Ogre::Vector3 normal = Ogre::Vector3(Ogre::Math::Cos(h)*Ogre::Math::Sin(v), Ogre::Math::Sin(h), Ogre::Math::Cos(h)*Ogre::Math::Cos(v));
				normal.normalise();
				std::cout << "[SphericalFracturePattern] Processing cut plane with normal " << normal << std::endl;
				Ogre::Quaternion randOrientation = Ogre::Vector3(0, 0, 1).getRotationTo(normal);
				auto fractalNoiseSDF = SDFManager::createFractalNoiseSDF(2.0f, 1.0f, 0.1f, randOrientation);
				auto cutPlaneSDF = OctreeSDF::sampleSDF(fractalNoiseSDF.get(), sphereSDF->getAABB(), octreeDepth);
				// SDFManager::exportSampledSDFAsMesh("Cutplane", cutPlaneSDF);

				std::vector<std::shared_ptr<OctreeSDF> > newPieces;
				for (auto iPiece = m_PatternPieces.begin(); iPiece != m_PatternPieces.end(); iPiece++)
				{
					auto newPiece = (*iPiece)->clone();
					newPiece->intersectAlignedOctree(cutPlaneSDF.get());
					(*iPiece)->simplify();
					(*iPiece)->subtractAlignedOctree(newPiece.get());
					newPiece->simplify();
					newPieces.push_back(*iPiece);
					newPieces.push_back(newPiece);
				}
				m_PatternPieces = newPieces;
				if (m_PatternPieces.size() >= maxNumPieces)
					break;
			}
			if (m_PatternPieces.size() >= maxNumPieces)
				break;
		}
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