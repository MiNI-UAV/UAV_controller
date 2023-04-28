#pragma once
#include <stdint.h>
#include <functional>
#include "status.hpp"

//Based on https://bitbucket.org/arrizza-public/algorithms/src/master/
class TimedLoop
{
    public:
        TimedLoop(int periodInMs, std::function<void(void)> func,Status& status);
        void go();
        void go(uint32_t loops);

    private:
        uint64_t get_current_clock_ns();
        int64_t calc_time_to_wait();
        uint64_t next_clock;

        std::function<void(void)> on_tick;
        uint64_t period;
        Status& _status;

        static constexpr uint64_t one_ms_in_ns = 1000000L;
};

