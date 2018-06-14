#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMMeasure.h"

TEST(volume_check, volume_of_block) 
    {
    SGMInternal::thing *pThing=SGM::CreateThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
    SGM::Body BodyID1=SGM::CreateBlock(rResult,Pos0,Pos1);
    double dVolume1=SGM::FindVolume(rResult,BodyID1,true);
    
    EXPECT_NEAR(dVolume1, 1000, SGM_ZERO);

    SGM::DeleteThing(pThing);
    }
