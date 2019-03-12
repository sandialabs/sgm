#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <EntityClasses.h>
#include <SGMGeometry.h>

#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMMeasure.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(volume_check, block_approximate)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0, 0, 0), Pos1(10, 10, 10);
    SGM::Body BodyID1 = SGM::CreateBlock(rResult, Pos0, Pos1);
    double dVolume1 = SGM::FindVolume(rResult, BodyID1, true);
    EXPECT_NEAR(dVolume1, 1000, SGM_ZERO);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(volume_check, block)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0, 0, 0), Pos1(10, 10, 10);
    SGM::Body BodyID1 = SGM::CreateBlock(rResult, Pos0, Pos1);
    double dVolume1 = SGM::FindVolume(rResult,BodyID1,false);
    EXPECT_NEAR(dVolume1,1000,SGM_MIN_TOL);

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(volume_check, sphere_sheet_body)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(1,1,1);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Norm,1.0);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Surface SurfaceID=SGM::CreateSphereSurface(rResult,Center,1.0);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(EdgeID);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
    SGM::Body SheetBodyID=SGM::CreateSheetBody(rResult,SurfaceID,aEdges,aTypes);

    double dVolumeSheetBody=SGM::FindVolume(rResult,SheetBodyID,false);

    EXPECT_NEAR(dVolumeSheetBody,0.0,SGM_MIN_TOL);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(volume_check, sphere)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::Body BodyID=SGM::CreateSphere(rResult,Center,1.0);

    double dVolumeBody=SGM::FindVolume(rResult,BodyID,false);

    // volume of sphere = 4/3 * PI * r^3

    EXPECT_NEAR(dVolumeBody,4.188790204786390984616857844373,SGM_MIN_TOL);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(volume_check, torus)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(0,0,1);
    SGM::Body BodyID=SGM::CreateTorus(rResult,Center,Norm,1,3);

    double dVolumeBody=SGM::FindVolume(rResult,BodyID,false);

    // volume of torus: V = (PI r^2) (2 PI R)

    EXPECT_NEAR(dVolumeBody,59.21762640653615171300694599926,SGM_MIN_TOL);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(volume_check, body_volumes)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
    SGM::Body BodyID1=SGM::CreateBlock(rResult,Pos0,Pos1);
    double dVolume1=SGM::FindVolume(rResult,BodyID1,true);
    EXPECT_NEAR(dVolume1,1000,SGM_ZERO);

    SGM::Point3D Pos3(0,0,0),Pos4(0,0,1);
    SGM::Body BodyID2=SGM::CreateCylinder(rResult,Pos3,Pos4,1.0);
    double dVolume2=SGM::FindVolume(rResult,BodyID2,false);
    EXPECT_TRUE(SGM::NearEqual(dVolume2,3.1415926535897932384626433832795,SGM_MIN_TOL,true));

    SGMTesting::ReleaseTestThing(pThing);
    }

// this is a preliminary experimental version of a test with async timeout
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

#ifdef __clang__
#pragma clang diagnostic pop
#endif