#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <FacetToBRep.h>

#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"

#include "test_utility.h"

TEST(create_check, create_parabola)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // y=ax^2 parabola
    // a=2

    aPoints.emplace_back(0.0, 0.0, 0.0);
    aPoints.emplace_back(1.0, 2.0, 0.0);
    aPoints.emplace_back(-1.0, 2.0, 0.0);
    aPoints.emplace_back(2.0, 8.0, 0.0);
    aPoints.emplace_back(-3.0, 18.0, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));
    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(create_check, create_hyperbola)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // x^2/a^2-y^2/b^2=1 hyperbola
    // a=2 b=3

    aPoints.clear();

    aPoints.emplace_back(2.0, 0.0, 0.0);
    aPoints.emplace_back(3.0, 3.3541019662496845446137605030969, 0.0);
    aPoints.emplace_back(6.0, 8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(6.0, -8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(3.0, -3.3541019662496845446137605030969, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));
    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(create_check, create_ellipse)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // x^2/a^2+y^2/b^2=1 ellipse
    // a=2 b=3

    aPoints.clear();

    aPoints.emplace_back(0.0, 3.0, 0.0);
    aPoints.emplace_back(2.0, 0.0, 0.0);
    aPoints.emplace_back(-2.0, 0.0, 0.0);
    aPoints.emplace_back(0.0, -3.0, 0.0);
    aPoints.emplace_back(1.0, 2.5980762113533159402911695122588, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));
    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }