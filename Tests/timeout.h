#ifndef SGM_TIMEOUT_H
#define SGM_TIMEOUT_H

#include <chrono>
#include <future>

///////////////////////////////////////////////////////////////////////////////
//
// Macros and functions for gtest testing
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// TIMEOUT
//
// Support for a gtest with a timeout.
//
// If what is inside the block times out you will see an EXPECT_TRUE(!timeout)
// type of gtest failure.
//
// Usage:
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

#define EXPECT_TIMEOUT_BEGIN auto asyncFuture = std::async(std::launch::async, [this]()->void {

#define EXPECT_TIMEOUT_END(X) return; }); \
bool timed_out = (asyncFuture.wait_for(std::chrono::milliseconds(X)) == std::future_status::timeout); \
EXPECT_TRUE(!timed_out);

#endif //SGM_TIMEOUT_H
