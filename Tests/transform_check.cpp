#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMEntityFunctions.h"
#include "SGMTransform.h"

#include "test_utility.h"


TEST(transform_check, translate_copies)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body Block = SGM::CreateBlock(rResult, SGM::Point3D(0.,0.,0.), SGM::Point3D(.5,1.0, 1.5));

    SGM::Entity BlockCopy1 = SGM::CopyEntity(rResult, Block);

    SGM::Vector3D Translation1(.1, .2, .3);
    SGM::Transform3D Trans1(Translation1);
    SGM::TransformEntity(rResult, Trans1, BlockCopy1);

    SGM::Interval3D BlockBox = SGM::GetBoundingBox(rResult, Block);
    SGM::Interval3D Copy1Box = SGM::GetBoundingBox(rResult, BlockCopy1);
    EXPECT_TRUE(SGM::NearEqual(Copy1Box.MidPoint(), BlockBox.MidPoint() + Translation1, SGM_MIN_TOL));


    // combine two translations
    SGM::Entity BlockCopy2 = SGM::CopyEntity(rResult, Block);

    SGM::Vector3D Translation2(.3, .2, .1);
    SGM::Transform3D Trans2(Translation2);
    SGM::Transform3D Trans = Trans1 * Trans2;
    SGM::TransformEntity(rResult, Trans, BlockCopy2);

    SGM::Interval3D Copy2Box = SGM::GetBoundingBox(rResult, BlockCopy2);
    EXPECT_TRUE(SGM::NearEqual(Copy2Box.MidPoint(), BlockBox.MidPoint() + Translation1 + Translation2, SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(Copy2Box.MidPoint(), Copy1Box.MidPoint() + Translation2, SGM_MIN_TOL));

    SGM::DeleteEntity(rResult,Block);
    SGM::DeleteEntity(rResult,BlockCopy1);
    SGM::DeleteEntity(rResult,BlockCopy2);

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(transform_check, rotate)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::Body Block = SGM::CreateBlock(rResult, SGM::Point3D(0.,0.,0.), SGM::Point3D(.5,1.0, 1.5));
//    SGM::Entity BlockCopy = SGM::CopyEntity(rResult, Block);
//
//    SGM::Vector3D Translation1(.1, .2, .3);
//    SGM::Transform3D Trans1(Translation1);
//    SGM::TransformEntity(rResult, Trans1, BlockCopy);
//
//    SGM::Interval3D BlockBox = SGM::GetBoundingBox(rResult, Block);
//    SGM::Interval3D CopyBox = SGM::GetBoundingBox(rResult, BlockCopy);
//    EXPECT_TRUE(SGM::NearEqual(CopyBox.MidPoint(), BlockBox.MidPoint() + Translation1, SGM_MIN_TOL));
//
//    SGM::DeleteEntity(rResult,BlockCopy);
//
//    // combine two translations
//    BlockCopy = SGM::CopyEntity(rResult, Block);
//
//    SGM::Vector3D Translation2(.3, .2, .1);
//    SGM::Transform3D Trans2(Translation2);
//    SGM::Transform3D Trans = Trans1 * Trans2;
//    SGM::TransformEntity(rResult, Trans, BlockCopy);
//
//    CopyBox = SGM::GetBoundingBox(rResult, BlockCopy);
//    EXPECT_TRUE(SGM::NearEqual(CopyBox.MidPoint(), BlockBox.MidPoint() + Translation1 + Translation2, SGM_MIN_TOL));
//
//
////    SGM::DeleteEntity(rResult,Block);
//
//    SGMTesting::ReleaseTestThing(pThing);
//}
