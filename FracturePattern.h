
#pragma once

#include <memory>
#include "SignedDistanceField.h"
#include "OctreeSDF.h"

class FracturePattern
{
protected:
	std::vector<std::shared_ptr<SignedDistanceField3D> > m_PatternPieces;
public:
	virtual ~FracturePattern() {}
	std::vector<std::shared_ptr<SignedDistanceField3D> > fractureSDF(std::shared_ptr<OctreeSDF> sdf)
	{
		std::vector<std::shared_ptr<SignedDistanceField3D> > outPieces;
		/*for (auto i = m_ReferencePieces.begin(); i != m_ReferencePieces.end(); ++i)
		{
			std::shared_ptr<SignedDistanceField3D> pieceClone = (*i)->clone();
			pieceClone->intersect(sdf);
			sdf->subtract(pieceClone);
			outPieces.push_back(pieceClone);
		}
		outPieces.push_back(sdf);*/
		return outPieces;
	}
};