
#pragma once

#include <filesystem>
#include <fstream>
#include <chrono>

#include "Defines.h"
#include "spdlog/spdlog.h"

namespace Vision
{

typedef Vector<char> FileBuffer;
typedef std::chrono::time_point<std::filesystem::file_time_type::clock> FileTimePoint;

class OS
{
public:
    static void Initialize();
	static void RunApplication();

    inline static Ref<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
    inline static Ref<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }

	long GetVersion();

	// Major Features
	int GetMajor();

	// Minor Features, Major Bug Fixes
	int GetMinor();

	// Minor Bug Fixes, Alterations
	int GetRevision();

	const char* GetVersionString();

	static String GetBuildDate();
	static String GetBuildTime();
	static String GetBuildType();
	static String GetOrganizationName();

	static String GetAppDataFolder();

private:
    static Ref<spdlog::logger> s_CoreLogger;
    static Ref<spdlog::logger> s_ClientLogger;

	// Main Engine Core
	const int Major = 0;

	// Minor Features, Major Updates, Breaking Compatibility Changes
	const int Minor = 1;

	// Minor Bug Fixes, Alterations, Refractors, Updates
	const int Revision = 0;

	const std::string VersionString = std::to_string(Major) + "." + std::to_string(Minor) + "." + std::to_string(Revision);

};

}

// Core log macros
#define VISION_CORE_TRACE(...)    ::Vision::OS::GetCoreLogger()->trace(__VA_ARGS__)
#define VISION_CORE_INFO(...)     ::Vision::OS::GetCoreLogger()->info(__VA_ARGS__)
#define VISION_CORE_WARN(...)     ::Vision::OS::GetCoreLogger()->warn(__VA_ARGS__)
#define VISION_CORE_ERROR(...)    ::Vision::OS::GetCoreLogger()->error(__VA_ARGS__)
#define VISION_CORE_CRITICAL(...) ::Vision::OS::GetCoreLogger()->critical(__VA_ARGS__)

// Client log macros
#define VISION_TRACE(...)    ::Vision::OS::GetClientLogger()->trace(__VA_ARGS__)
#define VISION_INFO(...)     ::Vision::OS::GetClientLogger()->info(__VA_ARGS__)
#define VISION_WARN(...)     ::Vision::OS::GetClientLogger()->warn(__VA_ARGS__)
#define VISION_ERROR(...)    ::Vision::OS::GetClientLogger()->error(__VA_ARGS__)
#define VISION_CRITICAL(...) ::Vision::OS::GetClientLogger()->critical(__VA_ARGS__)