#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMEntityFunctions.h"
#include "SGMEntityClasses.h"
#include "SGMTransform.h"

#include "test_utility.h"

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "cert-err58-cpp"

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

TEST(transform_check, translate_thing)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Thing ThingID;

    SGM::Body Block = SGM::CreateBlock(rResult, SGM::Point3D(0.,0.,0.), SGM::Point3D(.5,1.0, 1.5));

    SGM::Interval3D BoxBeforeTranslation = SGM::GetBoundingBox(rResult, ThingID);

    SGM::Vector3D Translation1(.1, .2, .3);
    SGM::Transform3D Trans1(Translation1);
    SGM::TransformEntity(rResult, Trans1, ThingID);

    SGM::Interval3D BoxAfter1Translation = SGM::GetBoundingBox(rResult, ThingID);
    EXPECT_TRUE(SGM::NearEqual(BoxAfter1Translation.MidPoint(),
                               BoxBeforeTranslation.MidPoint() + Translation1,
                               SGM_MIN_TOL));

    // make a second translations on top of the first
    SGM::Vector3D Translation2(.3, .2, .1);
    SGM::Transform3D Trans2(Translation2);
    SGM::TransformEntity(rResult, Trans2, ThingID);

    SGM::Interval3D BoxAfter2Translations = SGM::GetBoundingBox(rResult, ThingID);
    EXPECT_TRUE(SGM::NearEqual(BoxAfter2Translations.MidPoint(),
                               BoxBeforeTranslation.MidPoint() + Translation1 + Translation2,
                               SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(BoxAfter2Translations.MidPoint(),
                               BoxAfter1Translation.MidPoint() + Translation2,
                               SGM_MIN_TOL));

    SGM::DeleteEntity(rResult,Block);

    SGMTesting::ReleaseTestThing(pThing);
}

//#pragma clang diagnostic pop
