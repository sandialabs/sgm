#ifndef SGM_TESTING_QUEUE_H
#define SGM_TESTING_QUEUE_H

///////////////////////////////////////////////////////////////////////////////
// Preliminary experimental support for testing with timeout.
//
// GOAL: Have a queue of std::async tasks running as part of unit testing.
//       The tasks are longer running chunks of testing code.
//
// * If any one of a task's EXPECT() conditions fail, the test fail.
// * If a task times out, this is also an EXPECT_TRUE(!timeout) failure.
//
// Here is a first attempt to check for timeout failure.
//
// GTEST_TIMEOUT_BEGIN
//    ...
// GTEST_TIMOUT_END(ms)
//
// Surround code block with these two macros, the 'ms' argument is a signed
// integer duration in milliseconds. We use std::async to actually run the
// code block and to then wait on the future to complete. The code block
// is placed inside a lambda function. We wait for the future to complete,
// and if it does not, the test fails... and running of tests continues.
//
///////////////////////////////////////////////////////////////////////////////
#include <future>

#define EXPECT_TIMEOUT_BEGIN auto asyncFuture = std::async(std::launch::async, [this]()->void {

#define EXPECT_TIMEOUT_END(X) return; }); \
bool timed_out = (asyncFuture.wait_for(std::chrono::milliseconds(X)) == std::future_status::timeout); \
EXPECT_TRUE(!timed_out);


#endif //SGM_TESTING_QUEUE_H
