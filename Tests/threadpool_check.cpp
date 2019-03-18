#if defined(SGM_MULTITHREADED) && !(defined(_MSC_VER) && (_MSC_VER < 1910))

#include <atomic>
#include <chrono>
#include <deque>
#include <vector>
#include <cmath>
#include <gtest/gtest.h>

#include "SGMThreadPool.h"

TEST(threadpool_check, future_lambda)
{
    SGM::ThreadPool pool(4);
    std::vector< std::future<int> > results;

    for(int i = 0; i < 8; ++i)
        {
        results.emplace_back(
                pool.enqueue([i] {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    return i*i;
                    })
        );
        }
    std::cout << std::endl;
    std::vector<int> values;
    for(auto && result: results)
        {
        int value = result.get();
        std::cout << value << ' ';
        values.push_back(value);
        }
    std::cout << std::endl;
}

// example of calling a function that is sometimes expensive when initial is a multiple of an odd number
#define ODD_NUMBER 5
uint64_t sum_function(uint64_t initial, uint64_t count, const std::atomic_bool& run)
{
    uint64_t number = 0;
    uint64_t i = initial;
        if (i % ODD_NUMBER == 0)
            { // do expensive operation for certain multiples of i
            while(i <= count && run)
                {
                number += 3;
                ++i;
                }
            if (!run)
                std::cout << "[STOPPED]" << std::flush;
            }
        else
            {
            number = i;
            }
    return number; // number may not have completed all the sums if run==false
}

TEST(threadpool_check, future_timeout_function)
{
    SGM::ThreadPool pool(4);
    std::vector<std::future<uint64_t>> results;
    std::deque<std::atomic_bool> run_flags;

    uint64_t count = 2000000000;

    for (uint64_t x = 1; x <= ODD_NUMBER*6; ++x) // make this many calls
        {
        // an atomic run flag for each task
        run_flags.emplace_back(true);
        // long running sum function
        results.emplace_back(pool.enqueue(std::bind(sum_function, x, count, std::ref(run_flags.back()))));
        }

    std::cout << std::endl;
    for (auto &&result: results)
        {
        bool timed_out = (result.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout);
        if (timed_out)
            {
            run_flags.front() = false; // stop this task by setting atomic run flag to false
            std::cout << "timed out, incomplete value = " << result.get() << std::endl << std::flush;
            }
        else
            {
            auto value = result.get();
            EXPECT_GT(value % ODD_NUMBER, 0);
            std::cout << "finished, task = " << value << ' ' << std::endl << std::flush;
            }
        // done with this task (will this destroy the flag so that it can no longer be seen by the task?)
        run_flags.pop_front();
        }
    std::cout << std::endl;
}

#endif // SGM_MULTITHREADED