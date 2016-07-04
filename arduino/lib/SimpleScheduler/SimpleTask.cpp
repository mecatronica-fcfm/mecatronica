#include "SimpleTask.h"
#include "Arduino.h"


SimpleTask::SimpleTask(unsigned long period, SimpleTaskFunction function_ptr)
{
  _period = period;
  _function_ptr = function_ptr;
  _stopped = false;
  _last_activate = micros();
}

SimpleTask::~SimpleTask()
{
}

void SimpleTask::run()
{
  _last_activate = micros();
  (*_function_ptr)();
}

void SimpleTask::checkAndRun()
{
  if ( _stopped )
  {
    return;
  }
  unsigned long new_time = micros();
  unsigned long diff = new_time - _last_activate;
  if ( diff > _period )
  {
//    _last_activate = _last_activate + _period;
    _last_activate = new_time;
    (*_function_ptr)();
  }
}

void SimpleTask::stop()
{
  _stopped = true;
}
void SimpleTask::restart()
{
  _stopped = false;
  _last_activate = micros();
}


