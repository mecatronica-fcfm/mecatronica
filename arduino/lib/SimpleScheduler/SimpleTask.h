#ifndef SIMPLETASK_h
#define SIMPLETASK_h

extern "C" 
{
  typedef void (*SimpleTaskFunction)(void);
}


class SimpleTask
{
  public:
    SimpleTask(unsigned long period, SimpleTaskFunction function_ptr);
    ~SimpleTask();
    
    void run();
    void checkAndRun();
    void stop();
    void restart();

  private:
    unsigned long _period;
    unsigned long _last_activate;
    SimpleTaskFunction _function_ptr;
    bool _stopped;
};


#endif
