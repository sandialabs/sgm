#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <FacetToBRep.h>

#include "Curve.h"

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

    SGMInternal::curve *pConic0 = SGMInternal::FindConic(rResult, aPoints, dTolerance);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultType::ResultTypeOK);
    EXPECT_TRUE(pConic0 != nullptr);
    if (pConic0)
        {
        rResult.GetThing()->DeleteEntity(pConic0);
        }
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(create_check, DISABLED_create_hyperbola)
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
    aPoints.emplace_back(-2.0, 0.0, 0.0);
    aPoints.emplace_back(6.0, 8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(6.0, -8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(-6.0, 8.4852813742385702928101323452582, 0.0);

    SGMInternal::curve *pConic1 = SGMInternal::FindConic(rResult, aPoints, dTolerance);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultType::ResultTypeOK);
    EXPECT_TRUE(pConic1 != nullptr);
    if (pConic1)
        {
        rResult.GetThing()->DeleteEntity(pConic1);
        }

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

    SGMInternal::curve *pConic2 = SGMInternal::FindConic(rResult, aPoints, dTolerance);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultType::ResultTypeOK);
    EXPECT_TRUE(pConic2 != nullptr);
    if (pConic2)
        {
        rResult.GetThing()->DeleteEntity(pConic2);
        }

    SGMTesting::ReleaseTestThing(pThing);
    }