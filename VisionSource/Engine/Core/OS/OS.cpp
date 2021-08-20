#include "OS.h"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace Vision
{
Ref<spdlog::logger> OS::s_CoreLogger;
Ref<spdlog::logger> OS::s_ClientLogger;

void OS::Initialize()
{
	std::vector<spdlog::sink_ptr> logSinks;
	logSinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	logSinks.emplace_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("VisionEngine.log", true));

	logSinks[0]->set_pattern("%^[%T] %n: %v%$");
	logSinks[1]->set_pattern("[%T] [%l] %n: %v");

	s_CoreLogger = std::make_shared<spdlog::logger>("VISION ENGINE", begin(logSinks), end(logSinks));
	spdlog::register_logger(s_CoreLogger);
	s_CoreLogger->set_level(spdlog::level::trace);
	s_CoreLogger->flush_on(spdlog::level::trace);

	s_ClientLogger = std::make_shared<spdlog::logger>("Appication", begin(logSinks), end(logSinks));
	spdlog::register_logger(s_ClientLogger);
	s_ClientLogger->set_level(spdlog::level::trace);
	s_ClientLogger->flush_on(spdlog::level::trace);
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

}