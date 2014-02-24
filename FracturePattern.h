
#pragma once

#include <memory>
#include "SignedDistanceField.h"
#include "OctreeSDF.h"
#include "TransformSDF.h"

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
			*i = OctreeSDF::sampleSDF(std::make_shared<TransformSDF>(matrix, *i), (*i)->getHeight());
		}
	}

	std::vector<std::shared_ptr<SignedDistanceField3D> > fractureSDF(std::shared_ptr<OctreeSDF> sdf)
	{
		std::vector<std::shared_ptr<SignedDistanceField3D> > outPieces;
		for (auto i = m_PatternPieces.begin(); i != m_PatternPieces.end(); ++i)
		{
			std::shared_ptr<OctreeSDF> pieceClone = (*i)->clone();
			pieceClone->intersect(sdf);
			sdf->subtract(pieceClone);
			outPieces.push_back(pieceClone);
		}
		outPieces.push_back(sdf);
		return outPieces;
	}
};

class SphericalFracturePattern : public FracturePattern
{
	SphericalFracturePattern()
	{

	}
};