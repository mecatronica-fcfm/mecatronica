
#ifndef SIMPLESCHEDULER_h
#define SIMPLESCHEDULER_h
#include "SimpleTask.h"
#define MAX_SIMPLETASK 10

class SimpleScheduler
{
  public:
    SimpleScheduler();
    ~SimpleScheduler();
    
    void check();
    unsigned int addTask(unsigned long period, SimpleTaskFunction function);
    void stopTask(unsigned int id);
    void startTask(unsigned int id);
    void restartAll();

  private:
    unsigned int _tasks_registered;
    SimpleTask* _tasks_ptrs[MAX_SIMPLETASK];
};

#endif

