
#pragma once

#include <iostream>
#include <string>
// #define NOMINMAX
// #include <Windows.h>
#include <chrono>

class Profiler
{
public:
    typedef std::chrono::system_clock::time_point Timestamp;

	static Timestamp timestamp()
	{
        return std::chrono::high_resolution_clock::now();
	}

	static void printJobDuration(std::string jobName, Timestamp timeStampJobStart)
	{
        auto diff = timestamp() - timeStampJobStart;
        std::chrono::milliseconds milis = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
        std::cout << jobName << " - Done (took " << (float)milis.count() * 0.001f << " seconds)" << std::endl;
	}
};
