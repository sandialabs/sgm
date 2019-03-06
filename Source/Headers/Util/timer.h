#ifndef SGM_TIMER_H
#define SGM_TIMER_H

///////////////////////////////////////////////////////////////////////////////
//
// Macros for timing.
//
// Include this header in your module, #define SGM_TIMER to activate.
// Call these four functions in order, with multiple START and STOP if
// needed.
//
//  SGM_TIMER_INITIALIZE()
//      SGM_TIMER_START("my code A")
//      SGM_TIMER_STOP()
//      SGM_TIMER_START("my code B")
//      SGM_TIMER_STOP()
//  SGM_TIMER_SUM()
//
///////////////////////////////////////////////////////////////////////////////

#ifdef SGM_TIMER

#include <iostream>
#include <chrono>

typedef std::chrono::steady_clock::time_point time_point;
typedef std::chrono::steady_clock::time_point::duration duration;

namespace SGMInternal
{

inline duration print_elapsed_time(const time_point &start)
    {
    duration diff = std::chrono::steady_clock::now() - start;
    std::cout << std::chrono::duration<double,std::milli>(diff).count() << " ms" << std::endl;
    return diff;
    }

inline void print_total_time(duration const &sum)
    {
    std::cout << "Total time: " << std::chrono::duration<double,std::milli>(sum).count() << " ms" << std::endl;
    }

} // namespace SGMInternal

#define SGM_TIMER_INITIALIZE()                      \
    time_point start;                               \
    duration sum = std::chrono::milliseconds(0);

#define SGM_TIMER_START(s)                          \
    std::cout << s << ' ';                          \
    start = std::chrono::steady_clock::now();

#define SGM_TIMER_STOP()                            \
    sum += SGMInternal::print_elapsed_time(start);

#define SGM_TIMER_SUM()                             \
    SGMInternal::print_total_time(sum);

#else

#define SGM_TIMER_INITIALIZE()
#define SGM_TIMER_START(s)
#define SGM_TIMER_STOP()
#define SGM_TIMER_SUM()

#endif // SGM_TIMER

#endif //SGM_TIMER_H
