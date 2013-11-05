
#pragma once

#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreColourValue.h"

template<class T>
struct GenericVertex
{
	Ogre::Vector3 position;
	T data;
};