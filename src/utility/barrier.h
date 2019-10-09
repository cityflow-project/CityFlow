#ifndef CITYFLOW_BARRIER_H
#define CITYFLOW_BARRIER_H
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

namespace CityFlow {
    class Barrier {
    public:
        Barrier(std::size_t nb_threads) : m_threads(nb_threads), currCounter(&counter[0]) {
            assert(0u != m_threads);
            counter[0] = m_threads;
            counter[1] = 0;
        }

        Barrier(const Barrier& barrier) = delete;

        Barrier(Barrier&& barrier) = delete;

        Barrier& operator=(const Barrier& barrier) = delete;

        Barrier& operator=(Barrier&& barrier) = delete;

        void wait();

    private:
        std::mutex m_mutex;
        std::condition_variable m_condition;
        const size_t m_threads;
        size_t counter[2], * currCounter;
    };
}



#endif //CITYFLOW_BARRIER_H
