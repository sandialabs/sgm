//#include <limits>
//#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMPrimitives.h"
//#include "SGMGeometry.h"
//#include "SGMIntersector.h"

TEST(math_check, find_least_square_plane)
{
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // points all in a x=constant plane
    aPoints.emplace_back(0,-2,0);
    aPoints.emplace_back(0,-1,0.5);
    aPoints.emplace_back(0,0.0,0.25);
    aPoints.emplace_back(0,1,0.1);
    aPoints.emplace_back(0,2,.5);

    SGM::Point3D Origin;
    SGM::UnitVector3D XVec;
    SGM::UnitVector3D YVec;
    SGM::UnitVector3D ZVec;
    bool bFound = FindLeastSquarePlane(aPoints, Origin, XVec, YVec, ZVec);

    EXPECT_TRUE(bFound);
    EXPECT_TRUE(SGM::NearEqual(fabs(ZVec%SGM::UnitVector3D(1,0,0)), 1.0, SGM_ZERO, false));

    aPoints.clear();

    // points all in a y=constant plane
    aPoints.emplace_back(-2,0,0);
    aPoints.emplace_back(-1,0,0.5);
    aPoints.emplace_back(0.0,0,0.25);
    aPoints.emplace_back(1,0,0.1);
    aPoints.emplace_back(2,0,.5);

    bFound = FindLeastSquarePlane(aPoints, Origin, XVec, YVec, ZVec);

    EXPECT_TRUE(bFound);
    EXPECT_TRUE(SGM::NearEqual(fabs(ZVec%SGM::UnitVector3D(0,1,0)), 1.0, SGM_ZERO, false));

    aPoints.clear();

    // points all in a z=constant plane
    aPoints.emplace_back(-2,0,0);
    aPoints.emplace_back(-1,0.5,0);
    aPoints.emplace_back(0.0,0.25,0);
    aPoints.emplace_back(1,0.1,0);
    aPoints.emplace_back(2,.5,0);

    bFound = FindLeastSquarePlane(aPoints, Origin, XVec, YVec, ZVec);

    EXPECT_TRUE(bFound);
    EXPECT_TRUE(SGM::NearEqual(fabs(ZVec%SGM::UnitVector3D(0,0,1)), 1.0, SGM_ZERO, false));
}

