
#pragma once

#include <vector>
#include <cassert>
#include <istream>
#include <sstream>
#include "OgreMath/OgreColourValue.h"

class PPMWriter
{
public:
	static void writePPM(std::ostream &outStream, const std::vector<Ogre::ColourValue> &renderedImage, int width, int height)
	{
		assert(renderedImage.size() == width*height);

		outStream << "P6" << std::endl;
		std::stringstream strStream;
		strStream << width << " " << height << " " << "255";
		outStream << strStream.str() << std::endl;

		for (auto i = renderedImage.begin(); i != renderedImage.end(); i++)
		{
			Ogre::ColourValue clamped = i->saturateCopy();
			outStream << static_cast<unsigned char>(clamped.r*255.0f) << static_cast<unsigned char>(clamped.g*255.0f) << static_cast<unsigned char>(clamped.b*255.0f);
		}

	}
};