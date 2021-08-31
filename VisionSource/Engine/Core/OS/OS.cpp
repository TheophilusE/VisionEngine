
#define _CRT_SECURE_NO_WARNINGS

#include "OS.h"

#include <Windows.h>
#include <stdio.h>

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "../Event/EventManager.h"

namespace Vision
{

Ref<spdlog::logger> OS::s_CoreLogger;
Ref<spdlog::logger> OS::s_ClientLogger;

void OS::Initialize()
{
	//RunApplication();

    std::vector<spdlog::sink_ptr> logSinks;
    logSinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    logSinks.emplace_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("Logs/VisionEngine.log", true));

    logSinks[0]->set_pattern("%^[%T] %n: %v%$");
    logSinks[1]->set_pattern("[%T] [%l] %n: %v");

    s_CoreLogger = std::make_shared<spdlog::logger>("VISION ENGINE", begin(logSinks), end(logSinks));
    spdlog::register_logger(s_CoreLogger);
    s_CoreLogger->set_level(spdlog::level::trace);
    s_CoreLogger->flush_on(spdlog::level::trace);

    s_ClientLogger = std::make_shared<spdlog::logger>("Application", begin(logSinks), end(logSinks));
    spdlog::register_logger(s_ClientLogger);
    s_ClientLogger->set_level(spdlog::level::trace);
    s_ClientLogger->flush_on(spdlog::level::trace);

}

void OS::RunApplication()
{
    FreeConsole();

    // create a separate new console window
    AllocConsole();

    // attach the new console to this application's process
    AttachConsole(GetCurrentProcessId());

    // reopen the std I/O streams to redirect I/O to the new console
    freopen("CON", "w", stdout);
    freopen("CON", "w", stderr);
    freopen("CON", "r", stdin);

}

long OS::GetVersion()
{
    return 0L;
}

int OS::GetMajor()
{
    return Major;
}
int OS::GetMinor()
{
    return Minor;
}
int OS::GetRevision()
{
    return Revision;
}
const char* OS::GetVersionString()
{
    return VersionString.c_str();
}

String OS::GetBuildDate()
{
    return __DATE__;
}

String OS::GetBuildTime()
{
    return __TIME__;
}

String OS::GetBuildType()
{
    return "Debug";
}

String OS::GetOrganizationName()
{
    return "Vision Software";
}

String OS::GetAppDataFolder()
{
    /*
	String appDataFolderString;

	PWSTR appDataFolder = nullptr;
	if (FAILED(SHGetKnownFolderPath(FOLDERID_LocalAppData, KF_FLAG_CREATE, NULL, &appDataFolder)))
	{
		VISION_CORE_WARN("Could not get or create AppData folder. Using save directory in game directory.");
		//CreateDirectoryName("save");
		//appDataFolderString = GetAbsolutePath("save").generic_string();
	}
	else
	{
		appDataFolderString = WideStringToString(std::wstring(appDataFolder));
	}
	CoTaskMemFree(appDataFolder);

	return appDataFolderString;
	*/
    return "";
}

} // namespace Vision