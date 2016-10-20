#include "Logger.h"

const std::string Logger::CONFIGFILE = "EasyLogging.config";
const std::string Logger::LOGFILE = "logs/robot.log";

INITIALIZE_EASYLOGGINGPP

void Logger::Initialize()
{
	// Load configuration from file
	el::Loggers::configureFromGlobal(CONFIGFILE.c_str());

	el::Configurations defaultConf;

	defaultConf.setToDefault();
	defaultConf.set(el::Level::Global, el::ConfigurationType::Enabled, "true");

	defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
	defaultConf.setGlobally(el::ConfigurationType::ToFile, "true");
	defaultConf.setGlobally(el::ConfigurationType::Filename, LOGFILE);

	el::Loggers::reconfigureLogger("default", defaultConf);
	el::Loggers::setDefaultConfigurations(defaultConf, true);

}

el::Logger* Logger::GetLogger(const std::string& identity)
{
	return el::Loggers::getLogger(identity);
}
