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

#pragma once

#include <iostream>

#define ENABLE_ASSERTIONS 1

#define USE_BOOST_THREADING

#if ENABLE_ASSERTIONS
#define vAssert(condition) {if (!(condition)) { std::cout << "assertion failed: " << #condition << std::endl; __debugbreak(); }}
#else
#define vAssert(condition) 
#endif

#define OGRE_ENDIAN OGRE_ENDIAN_BIG

struct SphereBV;
struct AABB;

namespace Ogre
{
	typedef unsigned char uint8;
	typedef unsigned int uint32;
	typedef float Real;
}