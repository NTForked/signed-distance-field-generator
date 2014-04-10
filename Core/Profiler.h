
#pragma once

#include <iostream>
#include <string>
#define NOMINMAX
#include <Windows.h>

class Profiler
{
public:
	typedef unsigned int Timestamp;

	static Timestamp timestamp()
	{
		return timeGetTime();
	}

	static void printJobDuration(std::string jobName, Timestamp timeStampJobStart)
	{
		std::cout << jobName << " - Done (took " << (timestamp() - timeStampJobStart) * 0.001f << " seconds)" << std::endl;
	}
};