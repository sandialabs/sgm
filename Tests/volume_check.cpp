#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <EntityClasses.h>

#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMMeasure.h"

namespace SGMInternal
{
    namespace Testing
    {
        // a global static pointer to the pointer to the environment shared by the ModelViewer and the gtests.
        extern thing *pThing;
    }
}

TEST(volume_check, volume_of_block)
    {
        //SGM_TEST_ACQUIRE_STATE();

    SGMInternal::thing *pThing = nullptr;

    if (SGMInternal::Testing::pThing == nullptr)
        pThing = SGM::CreateThing();
    else
        pThing = SGMInternal::Testing::pThing;
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
    SGM::Body BodyID1=SGM::CreateBlock(rResult,Pos0,Pos1);
    double dVolume1=SGM::FindVolume(rResult,BodyID1,true);
    
    EXPECT_NEAR(dVolume1, 1000, SGM_ZERO);
    //EXPECT_TRUE(false);

    if (SGMInternal::Testing::pThing == nullptr)
        SGM::DeleteThing(pThing);
    }

// this is a preliminary experimental version of test with async timeout
//
//#include "timeout.h"
//
//TEST(volume_check, volume_of_block_timeout)
//{
//    EXPECT_TIMEOUT_BEGIN
//        SGMInternal::thing *pThing=SGM::CreateThing();
//        SGM::Result rResult(pThing);
//
//        SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
//        SGM::Body BodyID1=SGM::CreateBlock(rResult,Pos0,Pos1);
//        double dVolume1=SGM::FindVolume(rResult,BodyID1,true);
//
//        EXPECT_NEAR(dVolume1, 1000, SGM_ZERO);
//
//        SGM::DeleteThing(pThing);
//    EXPECT_TIMEOUT_END(1)
//}
