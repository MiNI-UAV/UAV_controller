#include "timed_loop.hpp"
#include <stdint.h>
#include <chrono>
#include <thread>
#include "status.hpp"

#include <iostream>

TimedLoop::TimedLoop(int periodInMs, std::function<void(void)> func,Status& status):
_status{status}
{
  on_tick = func;
  period = periodInMs*one_ms_in_ns;
}

void TimedLoop::go()
{
  int64_t time_to_wait = 0;
  //next_clock = ((get_current_clock_ns() / one_ms_in_ns) * one_ms_in_ns);
  next_clock = get_current_clock_ns();
  while(_status == Status::running)
  {
    on_tick();

    // calculate the next tick time and time to wait from now until that time
    time_to_wait = calc_time_to_wait();

    // check if we're already past the 1ms time interval
    if (time_to_wait > 0)
    {
      // wait that many ns
      std::this_thread::sleep_for(std::chrono::nanoseconds(time_to_wait));
    }
    else
    {
      std::cout<< "Time exceeded!\n";
    }
  }
}

void TimedLoop::go(uint32_t loops)
{
  int64_t time_to_wait = 0;
  //next_clock = ((get_current_clock_ns() / one_ms_in_ns) * one_ms_in_ns);
  next_clock = get_current_clock_ns();
  for (uint32_t loop = 0; loop < loops; ++loop)
  {
    if(_status != Status::running) break;
    on_tick();

    // calculate the next tick time and time to wait from now until that time
    time_to_wait = calc_time_to_wait();

    // check if we're already past the 1ms time interval
    if (time_to_wait > 0)
    {
      // wait that many ns
      std::this_thread::sleep_for(std::chrono::nanoseconds(time_to_wait));
    }
    else
    {
      std::cout<< "Time exceeded!\n";
    }
  }
}

uint64_t TimedLoop::get_current_clock_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int64_t TimedLoop::calc_time_to_wait()
{
  next_clock += period;
  return next_clock - get_current_clock_ns();
}
