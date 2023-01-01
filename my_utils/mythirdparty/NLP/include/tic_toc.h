#pragma once

#include <iostream>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }
    void tic()
    {
        start = std::chrono::steady_clock::now();
    }
    //  返回耗时 ms
    double toc()
    {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> cost = end - start;
        //  cost.count() 返回的是 s
        return cost.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::steady_clock> start, end; 
};


//  chrono 库的总结
