
#pragma once

#include "Vector3i.h"

template<class UserData>
class SignedDistanceField3D
{
public:
	struct Cube
	{
		Vector3i posMin;
		float signedDistances[8];
		UserData userData;
	};
	virtual std::vector<Cube> getCubesToMarch() = 0;
};