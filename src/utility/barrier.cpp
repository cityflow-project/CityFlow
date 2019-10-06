#include "barrier.h"

namespace CityFlow {
    void Barrier::wait() {
        std::unique_lock<std::mutex> lock(m_mutex);
        assert(0u != *currCounter);
        if (!--*currCounter) {
            currCounter += currCounter == counter ? 1 : -1;
            *currCounter = m_threads;
            m_condition.notify_all();
        } else {
            size_t *currCounter_local = currCounter;
            m_condition.wait(lock, [currCounter_local] { return *currCounter_local == 0; });
        }
    }
}