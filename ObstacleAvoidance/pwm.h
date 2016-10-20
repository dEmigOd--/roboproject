#pragma once

#include <memory>
#include "Common.h"

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
