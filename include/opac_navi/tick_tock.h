#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TickTock
{
  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;

  public:
    TickTock()
    {
        tick();
    }

    void tick()
    {
        start = std::chrono::system_clock::now();
    }

    double tock()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
};
