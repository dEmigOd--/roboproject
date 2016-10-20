#pragma once

#ifndef ROBOT_UI_H
#define ROBOT_UI_H

#include "Configuration.h"
#include <memory>
#include <vector>

class DelayedConfigEntry
{
	void (*_callback) (int, void*);
public:
	std::shared_ptr<RobotConfig> _config;
	RobotConfig::RobotConfigName _name;
	RobotConfig::RobotConfigName _maxValueName;
	int retValue;

	DelayedConfigEntry(std::shared_ptr<RobotConfig> config, RobotConfig::RobotConfigName name,
		RobotConfig::RobotConfigName maxValueName, void(*callback) (int, void*) = 0)
		: _config(config), _name(name), _maxValueName(maxValueName), _callback(callback)
	{
	}

	void OnChange(int newValue)
	{
		_config->lookup(_name) = newValue;
	}

	void InvokeCallback(int param)
	{
		if (_callback)
		{
			_callback(param, this);
		}
	}
};


class UI
{
	std::shared_ptr<RobotConfig> _config;
	std::vector<std::unique_ptr<void>> allocatedElements;

	void CreateTrackBar(const std::string& wndName, DelayedConfigEntry* dConfigEntry);
public:
	UI(std::shared_ptr<RobotConfig> config)
		: _config(config)
	{
	}

	void CreateUI();
};
#endif
