#include "SimpleScheduler.h"
#include "SimpleTask.h"
#include "Arduino.h"

SimpleScheduler::SimpleScheduler()
{
  _tasks_registered = 0;
}

SimpleScheduler::~SimpleScheduler()
{
  for( unsigned int i=0; i< _tasks_registered ; ++i)
  {
    delete( _tasks_ptrs[i] );
  }
}

void SimpleScheduler::check()
{
  for( unsigned int i=0; i< _tasks_registered ; ++i)
  {
    _tasks_ptrs[i]->checkAndRun();
  }
}

unsigned int SimpleScheduler::addTask(unsigned long period, SimpleTaskFunction function)
{
  if ( _tasks_registered >= MAX_SIMPLETASK )
  {
    return -1;
  }
  
  _tasks_ptrs[_tasks_registered] = new SimpleTask(period,function);
  return _tasks_registered++; 
}

void SimpleScheduler::stopTask(unsigned int id)
{
  if ( id < _tasks_registered )
  {
    _tasks_ptrs[id]->stop();
  }
}

void SimpleScheduler::startTask(unsigned int id)
{
  if ( id < _tasks_registered )
  {
    _tasks_ptrs[id]->restart();
  }
}
void SimpleScheduler::restartAll()
{
  for( unsigned int i=0; i< _tasks_registered ; ++i)
  {
    _tasks_ptrs[i]->restart();
  }
}
