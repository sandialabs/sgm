#include <gtest/gtest.h>                // for InitGoogleTest, etc

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool result = RUN_ALL_TESTS();
  return result;
}
