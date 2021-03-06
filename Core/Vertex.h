
#pragma once

#include "OgreMath/OgreVector3.h"
#include "OgreMath/OgreVector2.h"

struct Vertex
{
	Vertex() {}
	Vertex(const Ogre::Vector3 &_position, const Ogre::Vector3 &_normal) : position(_position), normal(_normal) {}
	Vertex(const Ogre::Vector3 &_position) : position(_position) {}
	Ogre::Vector3 position;
	Ogre::Vector3 normal;
	Ogre::Vector2 uv;
};