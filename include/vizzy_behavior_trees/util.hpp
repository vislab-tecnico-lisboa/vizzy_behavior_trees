#ifndef UTIL_BT_H_
#define UTIL_BT_H_

#include <iostream>
#include <chrono>
#include <thread>

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#endif