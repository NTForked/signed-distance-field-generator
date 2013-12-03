/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRACTALNOISEGENERATOR_H_
#define FRACTALNOISEGENERATOR_H_

#include "OgreMath/OgreMath.h"
#include <stdlib.h>

class FractalNoiseGenerator
{
public:
	static float**
	allocHeightMap(unsigned int iResolution)
	{
		float** pMap = new float*[iResolution + 1];

		for (int x = 0; x<iResolution + 1; x++)
			pMap[x] = new float[iResolution + 1];
		return pMap;
	}

	static void
	freeHeightMap(unsigned int iResolution, float** pMap)
	{
		for (int x = 0; x<iResolution + 1; x++)
			delete pMap[x];
		delete pMap;
	}

	/*
	* generate 2D fractal noise using square diamond algorithm
	* @return float[(1<<iResolution)+1][(1<<iResolution)+1] array with noise values
	*/
	static void
	generate(int iResolution, float fRoughness, float** ppfFNA)
	{
		int IMGSIZE = iResolution;

		ppfFNA[0][0]=0;
		ppfFNA[IMGSIZE][0]=0;
		ppfFNA[0][IMGSIZE]=0;
		ppfFNA[IMGSIZE][IMGSIZE]=0;

		int iStep=IMGSIZE;
		float fCurrRoughness=1;
		while(iStep>1)
		{
			fCurrRoughness*=fRoughness;
			for(int x=0; x<IMGSIZE; x+=iStep)
			{
				for(int y=0; y<IMGSIZE; y+=iStep)
				{
					//square step
					float fOffset=Ogre::Math::RangeRandom(-1.0f, 1.0f)*0.5f*fCurrRoughness*(float)iStep/(float)IMGSIZE;
					//calc average of surrounding square
					float fSum=0;
					fSum+=ppfFNA[x][y];
					fSum+=ppfFNA[x+iStep][y];
					fSum+=ppfFNA[x][y+iStep];
					fSum+=ppfFNA[x+iStep][y+iStep];
					//set center point of square
					ppfFNA[x+(iStep>>1)][y+(iStep>>1)]=fOffset+fSum/4.0f;
					//a little hack right here: since this generator is used to create a cutting plane
					//and this very point is the origin of that plane, we thereby make sure,
					//that the cutting plane will not exceed the original geometry
					if(iStep==IMGSIZE)
						ppfFNA[x+(iStep>>1)][y+(iStep>>1)]=0;

					//diamond step
					ppfFNA[x+(iStep>>1)][y]=(ppfFNA[x][y]+ppfFNA[x+iStep][y])/2.0f+
							Ogre::Math::RangeRandom(-1.0f, 1.0f)*0.5f*fCurrRoughness*(float)iStep/(float)IMGSIZE;
					ppfFNA[x][y+(iStep>>1)]=(ppfFNA[x][y]+ppfFNA[x][y+iStep])/2.0f+
							Ogre::Math::RangeRandom(-1.0f, 1.0f)*0.5f*fCurrRoughness*(float)iStep/(float)IMGSIZE;
					ppfFNA[x+(iStep>>1)][y+iStep]=(ppfFNA[x][y+iStep]+ppfFNA[x+iStep][y+iStep])/2.0f+
							Ogre::Math::RangeRandom(-1.0f, 1.0f)*0.5f*fCurrRoughness*(float)iStep/(float)IMGSIZE;
					ppfFNA[x+iStep][y+(iStep>>1)]=(ppfFNA[x+iStep][y]+ppfFNA[x+iStep][y+iStep])/2.0f+
							Ogre::Math::RangeRandom(-1.0f, 1.0f)*0.5f*fCurrRoughness*(float)iStep/(float)IMGSIZE;
				}
			}
			iStep>>=1;
		}
	}

};

#endif /* FRACTALNOISEGENERATOR_H_ */
