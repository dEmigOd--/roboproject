#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	PWM class
//	implements abstraction layer to execute actual movement of the robot
//
//	inhereted from pervious project
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 17
//
//M*/


#include <memory>

#include "Common.h"
#include "RunningParameters.h"

class PWM
{
public:
  virtual ~PWM() {}
  virtual void enable(int id, bool state) = 0;
  virtual void set_freq(int id, int freq) = 0;
  virtual void set_duty_cycle(int id, int value) = 0;
  
  virtual bool CouldBeStopped() const
  {
	  return false;
  }

  typedef std::shared_ptr<PWM> pwm_ptr;
  static pwm_ptr create(const RunningParameters& params);
};
