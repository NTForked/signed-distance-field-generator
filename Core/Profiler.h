
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
protected:
    std::unordered_map<std::string, std::chrono::nanoseconds> m_RunningJobs;

    static std::chrono::nanoseconds getDuration(const Timestamp& timeStampJobStart)
    {
        auto diff = timestamp() - timeStampJobStart;
        std::chrono::nanoseconds nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);
        return nanos;
    }

    static float toSeconds(const std::chrono::nanoseconds& nanos)
    {
        return (float)nanos.count() * 0.000000001f;
    }

public:
	static Timestamp timestamp()
	{
        return std::chrono::high_resolution_clock::now();
	}

	static void printJobDuration(std::string jobName, Timestamp timeStampJobStart)
	{
        std::cout << jobName << " - Done (took " << toSeconds(getDuration(timeStampJobStart)) << " seconds)" << std::endl;
	}

    void createJob(const std::string& jobName)
    {
        m_RunningJobs[jobName] = std::chrono::nanoseconds::zero();
    }

    void accumulateJobDuration(const std::string& jobName, const Timestamp& checkInTimestamp)
    {
        m_RunningJobs[jobName] += getDuration(checkInTimestamp);
    }

    void printJobDuration(const std::string& jobName)
    {
        std::cout << jobName << " - Done (took " << toSeconds(m_RunningJobs[jobName]) << " seconds)" << std::endl;
    }

    static Profiler& getSingleton()
    {
        static Profiler eyeOfSauron;
        return eyeOfSauron;
    }
};
