#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <EntityClasses.h>

#include "SGMVector.h"
#include "SGMEntityFunctions.h"
#include "SGMPrimitives.h"
#include "SGMMeasure.h"
#include "SGMTopology.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(area_check, area_of_sphere_single_face)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0, 0, 0);
    double dRadius = 1.0;
    SGM::Body UnitSphereID = SGM::CreateSphere(rResult, Center, dRadius);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,UnitSphereID,sFaces);
    EXPECT_EQ(1U, sFaces.size());

    SGM::Face FaceID=*(sFaces.begin());
    double dArea=SGM::FindArea(rResult,FaceID);
    EXPECT_TRUE(SGM::NearEqual(dArea,4*SGM_PI,SGM_MIN_TOL,false));

    SGM::DeleteEntity(rResult,UnitSphereID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    SGMTesting::ReleaseTestThing(pThing);
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif

