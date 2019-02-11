#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMIntersector.h"
#include "SGMSegment.h"
#include "SGMEntityFunctions.h"
#include "SGMMeasure.h"
#include "SGMTopology.h"
#include "SGMTransform.h"
#include "SGMDisplay.h"
#include "SGMComplex.h"

#include "test_utility.h"

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "cert-err58-cpp"


bool TestIntersections(SGM::Result      &rResult,
                       SGM::Curve const &Curve1,
                       SGM::Curve const &Curve2,
                       size_t            nExpectedPoints)
    {
    bool bAnswer=true;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    size_t nPoints=SGM::IntersectCurves(rResult,Curve1,Curve2,aPoints,aTypes);

    if(nPoints!=nExpectedPoints)
        {
        bAnswer=false;
        }
    else
        {
        for(SGM::Point3D const &Pos : aPoints)
            {
            SGM::Point3D Pos1,Pos2;
            SGM::CurveInverse(rResult,Curve1,Pos,&Pos1);
            SGM::CurveInverse(rResult,Curve2,Pos,&Pos2);
            if( SGM_MIN_TOL<Pos.Distance(Pos1) ||
                SGM_MIN_TOL<Pos.Distance(Pos2))
                {
                bAnswer=false;
                break;
                }
            }
        }
    return bAnswer;
    }

bool TestIntersections(SGM::Result        &rResult,
                       SGM::Curve   const &Curve,
                       SGM::Surface const &Surface,
                       size_t              nExpectedPoints)
    {
    bool bAnswer=true;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    size_t nPoints=SGM::IntersectCurveAndSurface(rResult,Curve,Surface,aPoints,aTypes);

    if(nExpectedPoints==0)
        {
        if(aTypes.empty() || aTypes[0]!=SGM::IntersectionType::CoincidentType)
            {
            bAnswer=false;
            }
        }
    else if(nPoints!=nExpectedPoints)
        {
        bAnswer=false;
        }
    else
        {
        for(SGM::Point3D const &Pos : aPoints)
            {
            SGM::Point3D Pos1,Pos2;
            SGM::CurveInverse(rResult,Curve,Pos,&Pos1);
            SGM::SurfaceInverse(rResult,Surface,Pos,&Pos2);
            if( SGM_MIN_TOL<Pos.Distance(Pos1) ||
                SGM_MIN_TOL<Pos.Distance(Pos2))
                {
                bAnswer=false;
                break;
                }
            }
        }
    return bAnswer;
    }

bool TestIntersections(SGM::Result        &rResult,
                       SGM::Surface const &Surface1,
                       SGM::Surface const &Surface2,
                       size_t              nExpectedCurves)
    {
    bool bAnswer=true;
    std::vector<SGM::Curve> aCurves;
    size_t nCurves=SGM::IntersectSurfaces(rResult,Surface1,Surface2,aCurves);

    if(nCurves!=nExpectedCurves)
        {
        bAnswer=false;
        }
    else
        {
        size_t Index1,Index2;
        size_t nTestPoint=10;
        for(Index1=0;bAnswer && Index1<nCurves;++Index1)
            {
            SGM::Curve CurveID=aCurves[Index1];
            SGM::Interval1D Domain=SGM::GetCurveDomain(rResult,CurveID);
            double dTol=SGM_FIT;
            if(Domain.IsBounded())
                {
                dTol=SGM_FIT*Domain.Length()*5;
                }
            else if(Domain.IsBoundedAbove()==false)
                {
                Domain=SGM::Interval1D(0,1);
                }
            if(dTol<SGM_MIN_TOL)
                {
                dTol=SGM_MIN_TOL;
                }
            for(Index2=1;Index2<nTestPoint;++Index2)
                {
                double dFraction=Index2/(nTestPoint-1.0);
                double t=Domain.MidPoint(dFraction);
                SGM::Point3D Pos;
                SGM::EvaluateCurve(rResult,CurveID,t,&Pos);
                SGM::Point3D CPos1,CPos2;
                SGM::SurfaceInverse(rResult,Surface1,Pos,&CPos1);
                SGM::SurfaceInverse(rResult,Surface2,Pos,&CPos2);
                if(dTol<Pos.Distance(CPos1) && dTol<Pos.Distance(CPos2))
                    {
                    bAnswer=false;
                    break;
                    }
                }
            }
        }
    return bAnswer;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Start of G Tests
//
///////////////////////////////////////////////////////////////////////////////

//TEST(intersection_check, intersect_plane_cone2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::CreateCone(rResult,SGM::Point3D(2,-0.5,0),SGM::Point3D(2,3,0),1.1666666666666665,0); 
//    SGM::CreateDisk(rResult,SGM::Point3D(1.9910326583929372,-0.18917968498957044,0),SGM::UnitVector3D(0.094589842494785220,0.99551632919646860,0),2);
//
//    SGM::Surface ConeID1=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,-0.5,0),SGM::Point3D(2,3,0),1.1666666666666665,0);
//    SGM::Surface PlaneID1=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1.9910326583929372,-0.18917968498957044,0),SGM::UnitVector3D(0.094589842494785220,0.99551632919646860,0));
//
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,ConeID1,PlaneID1,aCurves);
//    for(auto pCurve : aCurves)
//        { 
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

//TEST(intersection_check, intersect_cone_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::CreateCone(rResult,SGM::Point3D(-3,0,2),SGM::Point3D(4,0,0),1,0);
//
//    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::Surface ConeID=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(-3,0,2),SGM::Point3D(4,0,0),1,0);
//    
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,TorusID,ConeID,aCurves);
//    for(auto pCurve : aCurves)
//        { 
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_cone_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface ConeID1=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,-4,0),SGM::Point3D(2,3,0),2.333333333333333,0);
    SGM::Surface ConeID2=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,sqrt(2)/2),SGM::Point3D(0,0,2+sqrt(2)/2),2+sqrt(2)/2,sqrt(2)/2);
    SGM::Surface ConeID3=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,-2),SGM::Point3D(0,0,2),4,0);
    SGM::Surface ConeID4=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(3,0,-2),SGM::Point3D(3,0,2),4,0);
    SGM::Surface ConeID5=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,0,-2),SGM::Point3D(2,0,2),2,0);
    SGM::Surface ConeID6=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(-4,0,0),SGM::Point3D(4,0,0),2,0);

    double dVillarceauAngle=SGM::SAFEasin(0.5);
    SGM::Surface ConeID7=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,-sin(dVillarceauAngle)*3,
                             -cos(dVillarceauAngle)*3),SGM::Point3D(1,sin(dVillarceauAngle)*3,cos(dVillarceauAngle)*3),4,0);

    SGM::Point3D Pos1(3-cos(10*SGM_PI/180.0)*6,0,1+sin(10*SGM_PI/180.0)*6);
    SGM::Point3D Pos2(3-6/cos(10*SGM_PI/180.0),0,1);
    double dRadius=Pos1.Distance(Pos2);
    SGM::Surface ConeID8=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(3-cos(10*SGM_PI/180.0)*6,0,1+sin(10*SGM_PI/180.0)*6),SGM::Point3D(3,0,1),dRadius,0);
    
    double dDist=SGM::Point3D(-3,0,1).Distance(SGM::Point3D(2,0,0));
    double dAngle=SGM::UnitVector3D(1,0,0).Angle(SGM::UnitVector3D(SGM::Point3D(2,0,0)-SGM::Point3D(-3,0,1)));
    double dR=tan(dAngle)*dDist;
    double t=0.3;
    SGM::Surface ConeID9=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2+5*t,0,-t),SGM::Point3D(-3,0,1),dR*(1+t),0);

    SGM::Surface ConeID10=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,0,3),SGM::Point3D(2,0,1),2,0);
    SGM::Surface ConeID11=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(-3,0,2),SGM::Point3D(4,0,0),1,0);
    
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,TorusID,3));     // Minor Circle and Two Curves
    EXPECT_TRUE(TestIntersections(rResult,ConeID2,TorusID,1));     // One Major Circle
    EXPECT_TRUE(TestIntersections(rResult,ConeID3,TorusID,2));     // Two Major Circles
    EXPECT_TRUE(TestIntersections(rResult,ConeID4,TorusID,1));     // One Curve C C
    EXPECT_TRUE(TestIntersections(rResult,ConeID5,TorusID,2));     // Two Curves NN CC  
    EXPECT_TRUE(TestIntersections(rResult,ConeID6,TorusID,4));     // Four Curves NNCC NNCC 
    EXPECT_TRUE(TestIntersections(rResult,ConeID7,TorusID,2));     // Villarceau Circle and Curve
    EXPECT_TRUE(TestIntersections(rResult,ConeID8,TorusID,2));     // Two Outside Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,ConeID9,TorusID,6));     // Three Inside Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,ConeID10,TorusID,1));    // Non-Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID11,TorusID,3));    // Three Curves

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(intersection_check, intersect_circle_cone2) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::CreateCone(rResult,SGM::Point3D(2,-0.5,0),SGM::Point3D(2,3,0),1.1666666666666665,0); 
    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(1.9910326583929372,-0.18917968498957044,0),
                                          SGM::UnitVector3D(0.094589842494785220,0.99551632919646860,0),1);
    SGM::CreateEdge(rResult,CircleID);

    //SGM::Surface ConeID=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,-0.5,0),SGM::Point3D(2,3,0),1.1666666666666665,0);
    //std::vector<SGM::Point3D> aPoints;
    //std::vector<SGM::IntersectionType> aTypes;
    //SGM::IntersectCurveAndSurface(rResult,CircleID,ConeID,aPoints,aTypes);
    //SGM::CreatePoints(rResult,aPoints);

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(intersection_check, intersect_circle_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateTorus(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),1,3);
//    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2);
//    SGM::CreateEdge(rResult,CircleID);
//    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),1,3);
//    
//    std::vector<SGM::Point3D> aPoints;
//    std::vector<SGM::IntersectionType> aTypes;
//    SGM::IntersectCurveAndSurface(rResult,CircleID,TorusID,aPoints,aTypes);
//    SGM::CreatePoints(rResult,aPoints);
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(intersection_check, intersect_circle_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),1,3);
    SGM::Curve CircleID1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2);

    EXPECT_TRUE(TestIntersections(rResult,CircleID1,TorusID,1));     // One Tangent Point.

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(intersection_check, intersect_circle_hermite) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::Vector3D> aVectors;
    std::vector<double> aParams;
    aPoints.push_back(SGM::Point3D(-1,-1,0));
    aPoints.push_back(SGM::Point3D(0,-1,0));
    aPoints.push_back(SGM::Point3D(0,0,0));
    aPoints.push_back(SGM::Point3D(0,1,0));
    aPoints.push_back(SGM::Point3D(3,1,0));
    aVectors.push_back(SGM::Vector3D(1,0,0));
    aVectors.push_back(SGM::Vector3D(2,0,0));
    aVectors.push_back(SGM::Vector3D(-2,0,0));
    aVectors.push_back(SGM::Vector3D(2,0,0));
    aVectors.push_back(SGM::Vector3D(1,0,0));
    aParams.push_back(0);
    aParams.push_back(3);
    aParams.push_back(4);
    aParams.push_back(5);
    aParams.push_back(8);
    SGM::Curve CurveID1=SGM::CreateHermiteCurve(rResult,aPoints,aVectors,aParams);
    SGM::CreateEdge(rResult,CurveID1);

    SGM::Curve CurveID2=SGM::CreateCircle(rResult,SGM::Point3D(1.5,0,0),SGM::UnitVector3D(0,0,1),1.5);
    SGM::CreateEdge(rResult,CurveID2);

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,CurveID1,CurveID2,aHits,aTypes);

    SGM::CreatePoints(rResult,aHits);

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(intersection_check, intersect_circle_plane2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2);
//    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,0),1);
//    SGM::CreateEdge(rResult,CircleID);
//
//    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
//    std::vector<SGM::Point3D> aPoints;
//    std::vector<SGM::IntersectionType> aTypes;
//    SGM::IntersectCurveAndSurface(rResult,CircleID,PlaneID,aPoints,aTypes);
//    SGM::CreatePoints(rResult,aPoints);
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(intersection_check, intersect_circle_plane) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Curve CircleID1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),1);
    SGM::Curve CircleID2=SGM::CreateCircle(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(1,0,0),1);
    SGM::Curve CircleID3=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);

    EXPECT_TRUE(TestIntersections(rResult,CircleID1,PlaneID,2));     // Two Points
    EXPECT_TRUE(TestIntersections(rResult,CircleID2,PlaneID,1));     // One Point
    EXPECT_TRUE(TestIntersections(rResult,CircleID3,PlaneID,0));     // Coincident

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_torus_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0.1,0),SGM::UnitVector3D(0,1,0),0.5,2);  
//
//    SGM::Surface TorusID1=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::Surface TorusID10=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0.1,0),SGM::UnitVector3D(0,1,0),0.5,2);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,TorusID1,TorusID10,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//    
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(intersection_check, intersect_torus_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface TorusID1=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID2=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,2),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID3=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,3);
    SGM::Surface TorusID4=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),0.5,1.5);
    SGM::Surface TorusID5=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),1,2);
    SGM::Surface TorusID6=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),2,3);
    SGM::Surface TorusID7=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),1,3);
    SGM::Surface TorusID8=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),1,4);
    SGM::Surface TorusID9=SGM::CreateTorusSurface(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID10=SGM::CreateTorusSurface(rResult,SGM::Point3D(5,0,1.5),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID11=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),1,3);
    SGM::Surface TorusID12=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),0.5,2);
    SGM::Surface TorusID13=SGM::CreateTorusSurface(rResult,SGM::Point3D(0.1,0,0),SGM::UnitVector3D(0,1,0),0.1,1);
    SGM::Surface TorusID14=SGM::CreateTorusSurface(rResult,SGM::Point3D(0.5,0,0),SGM::UnitVector3D(0,1,0),0.5,1.5);
    SGM::Surface TorusID15=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0.1,0),SGM::UnitVector3D(0,1,0),0.5,2);
    
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID2,1));     // One Major Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID3,2));     // Two Major Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID4,1));     // One Minor Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID5,2));     // Two Minor Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID6,3));     // Minor Circle and Two Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID3,TorusID7,8));     // Four Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID8,2));     // Two Outside Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID9,4));     // Two VCircles Two Curves Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID10,1));    // One Curve
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID11,2));    // Two Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID12,4));    // Four Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID13,3));    // Two Curves and Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID14,3));    // Three Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID1,TorusID15,4));    // Off Center Four Curves

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_cone_cone2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::CreateCone(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),2,0);
//    SGM::CreateCone(rResult,SGM::Point3D(1,0,0),SGM::Point3D(1,0,2),1,0); 
//    
//    SGM::Surface ConeID1=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),2,0);
//    SGM::Surface ConeID2=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,0),SGM::Point3D(1,0,2),1,0);
//
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,ConeID1,ConeID2,aCurves);
//    for(auto pCurve : aCurves)
//        { 
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_cone_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID1=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),2,0);
    SGM::Surface ConeID2=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,2),SGM::Point3D(0,0,0),2,0);
    SGM::Surface ConeID3=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,3),1.5,0);
    SGM::Surface ConeID4=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,4),SGM::Point3D(0,0,2),2,0);
    SGM::Surface ConeID5=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,0,2),SGM::Point3D(0,0,2),2,0);
    SGM::Surface ConeID6=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,0,0),SGM::Point3D(0,0,2),2,0);
    SGM::Surface ConeID7=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(3,0,1),SGM::Point3D(1,0,1),2,0);
    SGM::Surface ConeID8=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,3),SGM::Point3D(1,0,1),1,0);
    SGM::Surface ConeID9=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,3),SGM::Point3D(1,0,1),2,0);
    SGM::Surface ConeID10=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(2,0,2),SGM::Point3D(0,0,2),2,0);
    SGM::Transform3D Trans(SGM::Point3D(1,0,1),SGM::UnitVector3D(1,0,1),SGM_PI*0.25);
    SGM::TransformEntity(rResult,Trans,ConeID10);
    SGM::Surface ConeID11=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,0.5),SGM::Point3D(1,0,2.5),2,0);
    SGM::Surface ConeID12=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0.5,0,2.5),SGM::Point3D(0.5,0,0.5),2,0);
    SGM::Surface ConeID13=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),0.5,0);
    SGM::Surface ConeID14=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,1),SGM::Point3D(-1,0,1),0.5,0);
    SGM::Surface ConeID15=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1.5,0,1.5),SGM::Point3D(-0.5,0,1.5),2,0);
    SGM::Surface ConeID16=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,0,1),SGM::Point3D(1,0,3),2,0);
    SGM::Surface ConeID17=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0.5,-3,1),SGM::Point3D(0.5,3,1),0.5,0);
    SGM::Surface ConeID18=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,-3,1),SGM::Point3D(1,3,1),1,0);
    SGM::Surface ConeID19=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(-2,0,1),SGM::Point3D(0,0,1),0.5,0);
    SGM::Surface ConeID20=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(-2,0,1),SGM::Point3D(1,0,1),0.5,0);
    SGM::Surface ConeID21=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(0.52859547920896821,-1.8856180831641269,
                                                           0.52859547920896821),SGM::Point3D(1,2,1),0.67640791490305108,0);
    SGM::Surface ConeID22=SGM::CreateConeSurfaceFromPoints(rResult,SGM::Point3D(1,-2,2),SGM::Point3D(1,2,2),2,0);
    
    
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID2,1));     // Hourglass Circle
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID3,1));     // Witch Hat Circle
    EXPECT_TRUE(TestIntersections(rResult,ConeID3,ConeID1,1));     // Witch Hat Circle Other Order
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID4,1));     // Apex to Apex
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID5,1));     // Line From Apex
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID6,2));     // Two Lines From Apex
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID7,1));     // One Line With Apexes Not Matching
    EXPECT_TRUE(TestIntersections(rResult,ConeID7,ConeID1,1));     // One Line With Apexes Not Matching Other Order
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID8,1));     // Non-Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID8,ConeID1,1));     // Non-Tangent Point Other Order
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID9,1));     // Line Segment
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID10,1));    // Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID10,ConeID1,1));    // Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID11,1));    // Hyperbola
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID12,1));    // Ellipse
    EXPECT_TRUE(TestIntersections(rResult,ConeID13,ConeID14,2));   // Two Ellipses
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID15,2));    // Line Segment and Parabola
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID16,1));    // Inside Line from Apex
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID17,2));    // Two Curves CC NN
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID18,1));    // One Curve C C
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID19,1));    // One Curve C N
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID20,2));    // One Curve and Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID21,2));    // Two Curves Figure Eight
    EXPECT_TRUE(TestIntersections(rResult,ConeID1,ConeID22,1));    // Teardrop Curve

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_sphere_cylinder2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateCylinder(rResult,SGM::Point3D(0,0,-1),SGM::Point3D(0,0,3),1);
//    SGM::CreateSphere(rResult,SGM::Point3D(0.5,0,1),2); 
//    
//    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
//    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,0,1),2);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,CylinderID,SphereID,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//    
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(intersection_check, intersect_sphere_cylinder) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,1),1);
    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,1),2);
    SGM::Surface SphereID3=SGM::CreateSphereSurface(rResult,SGM::Point3D(1.5,0,1),0.5);
    SGM::Surface SphereID4=SGM::CreateSphereSurface(rResult,SGM::Point3D(1.5,0,1),1);
    SGM::Surface SphereID5=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,0,1),2);
    SGM::Surface SphereID6=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,0,1),2);

    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID1,1));     // One Circle
    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID2,2));     // Two Circles
    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID3,1));     // Point
    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID4,1));     // One Curve
    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID5,2));     // Figure Eight
    EXPECT_TRUE(TestIntersections(rResult,CylinderID,SphereID6,2));     // Two Curves

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_sphere_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    //double dVillarceauAngle=SGM::SAFEasin(pTorus->m_dMinorRadius/pTorus->m_dMajorRadius);
//    double dVillarceauAngle=SGM::SAFEasin(0.5);
//    
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::CreateSphere(rResult,SGM::Point3D(1,sin(dVillarceauAngle)*2,cos(dVillarceauAngle)*2),sqrt(8));
//    
//    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,sin(dVillarceauAngle)*2,cos(dVillarceauAngle)*2),sqrt(8));
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,SphereID,TorusID,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_sphere_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),2);
    SGM::Surface SphereID3=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),3);
    SGM::Surface SphereID4=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,2),sqrt(2)*2-1);
    SGM::Surface SphereID5=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,2),sqrt(2)*2);
    SGM::Surface SphereID6=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,1,0),2);
    SGM::Surface SphereID7=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,2,0),sqrt(2));
    SGM::Surface SphereID8=SGM::CreateSphereSurface(rResult,SGM::Point3D(2,0,0),1);
    SGM::Surface SphereID9=SGM::CreateSphereSurface(rResult,SGM::Point3D(3,0,0),2);
    SGM::Surface SphereID10=SGM::CreateSphereSurface(rResult,SGM::Point3D(2,0,3),4);
    double dVillarceauAngle=SGM::SAFEasin(0.5);
    SGM::Surface SphereID11=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,sin(dVillarceauAngle)*2,cos(dVillarceauAngle)*2),sqrt(8));
    
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID1,1));   // Sphere at Torus Center One Inside Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID2,2));   // Sphere at Torus Center Two Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID3,1));   // Sphere at Torus Center One Outside Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID4,1));   // Sphere on Axis One Tangent Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID5,2));   // Sphere on Axis Two Tangent Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID6,2));   // Villarceau Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID7,2));   // Two Minor Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID8,1));   // One Minor Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID9,2));   // Figure Eight
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID10,2));  // Two Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID,SphereID11,2));  // Villarceau Circles, Sphere Center off Plane

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_plane_sphere2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
//    SGM::CreateDisk(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(0,0,1),2);
//    
//    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
//    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(0,0,1));
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,SphereID,PlaneID,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_plane_sphere) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface PlaneID1=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,0.5),SGM::UnitVector3D(0,0,1));
    SGM::Surface PlaneID2=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(0,0,1));

    EXPECT_TRUE(TestIntersections(rResult,SphereID,PlaneID1,1));  // Circle
    EXPECT_TRUE(TestIntersections(rResult,SphereID,PlaneID2,1));  // Point

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(intersection_check, intersect_conics2) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Curve Curve1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2);
    SGM::Curve Curve2=SGM::CreateParabola(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),SGM::UnitVector3D(1,0,0),1);
    SGM::CreateEdge(rResult,Curve1);
    SGM::Interval1D Domain(-1.5,1.5);
    SGM::CreateEdge(rResult,Curve2,&Domain);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,Curve1,Curve2,aPoints,aTypes);
    SGM::CreatePoints(rResult,aPoints);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(intersection_check, intersect_conics) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve Ellipse1=SGM::CreateEllipse(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),2,1);
    SGM::Curve Ellipse2=SGM::CreateEllipse(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,2);
    SGM::Curve Ellipse3=SGM::CreateEllipse(rResult,SGM::Point3D(0,1,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,2);
    SGM::Curve Ellipse4=SGM::CreateEllipse(rResult,SGM::Point3D(0,2,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,2);
    SGM::Curve Ellipse5=SGM::CreateEllipse(rResult,SGM::Point3D(0,3,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,2);
    SGM::Curve Circle1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve Hyperbola1=SGM::CreateHyperbola(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),SGM::UnitVector3D(1,0,0),1,1);
    SGM::Curve Parabola1=SGM::CreateParabola(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1);
    SGM::Curve Parabola2=SGM::CreateParabola(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),SGM::UnitVector3D(1,0,0),1);
    
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Ellipse2,4));    // Four points.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Ellipse3,3));    // Three points.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Ellipse4,2));    // Two points.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Ellipse5,1));    // One point.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Circle1,2));     // Two points.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Hyperbola1,2));  // Two points.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Parabola1,2));   // Two points y=x^2.
    EXPECT_TRUE(TestIntersections(rResult,Ellipse1,Parabola2,2));   // Two points x^2=y.

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_cylinder_cylinder2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateCylinder(rResult,SGM::Point3D(0,0,-2),SGM::Point3D(0,0,2),1);
//    SGM::CreateCylinder(rResult,SGM::Point3D(0.5,-2,0),SGM::Point3D(0.5,2,0),0.5);
//  
//    SGM::Surface CylinderID1=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
//    SGM::Surface CylinderID2=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0.5,-2,0),SGM::Point3D(0.5,2,0),0.5);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,CylinderID1,CylinderID2,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_cylinder_cylinder) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID1=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
    SGM::Surface CylinderID2=SGM::CreateCylinderSurface(rResult,SGM::Point3D(1,0,0),SGM::Point3D(1,0,2),1);
    SGM::Surface CylinderID3=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2,0,0),SGM::Point3D(2,0,2),1);
    SGM::Surface CylinderID4=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(2,0,0),1);
    SGM::Surface CylinderID5=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2,0,0),SGM::Point3D(2,2,0),1);
    SGM::Surface CylinderID6=SGM::CreateCylinderSurface(rResult,SGM::Point3D(1,0,0),SGM::Point3D(1,2,0),1);
    SGM::Surface CylinderID7=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,-2,0),SGM::Point3D(0,2,0),0.5);
    
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID2,2));  // Two lines
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID3,1));  // One line
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID4,2));  // Two Ellipses
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID5,1));  // Point
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID6,1));  // One Hermite
    EXPECT_TRUE(TestIntersections(rResult,CylinderID1,CylinderID7,2));  // Two Hermites

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_plane_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,2);
//    SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),5);
//  
//    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,2);
//    SGM::Surface PlaneID=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0));
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,TorusID,PlaneID,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_plane_torus)
{ 
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dAlpha=SGM::SAFEasin(1.0/2.0);
    SGM::UnitVector3D PlaneNormal(sin(dAlpha),0.0,cos(dAlpha));

    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID2=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,2); // Pinched Torus
    SGM::Surface PlaneID1=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),PlaneNormal);
    SGM::Surface PlaneID2=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Surface PlaneID3=SGM::CreatePlane(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(0,0,1));
    SGM::Surface PlaneID4=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0.5),SGM::UnitVector3D(0,0,1));
    SGM::Surface PlaneID5=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,1,0));
    SGM::Surface PlaneID6=SGM::CreatePlane(rResult,SGM::Point3D(3,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface PlaneID7=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,2));
    SGM::Surface PlaneID8=SGM::CreatePlane(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface PlaneID9=SGM::CreatePlane(rResult,SGM::Point3D(0.9,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface PlaneID10=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0));
    SGM::Surface PlaneID11=SGM::CreatePlane(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(1,0,0));

    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID1,2));     // Villarceau Circles 
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID2,2));     // Two Major Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID3,1));     // One Major Circle
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID4,2));     // Two Major Circles not at Equator
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID5,2));     // Two Minor Circles
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID6,1));     // Outside Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID7,2));     // Two Major Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID8,1));     // One Curve Spiric Section, Cassini oval
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID9,2));     // Two Curves Spiric Section
    EXPECT_TRUE(TestIntersections(rResult,TorusID2,PlaneID10,2));   // Two Touching Circles.
    EXPECT_TRUE(TestIntersections(rResult,TorusID,PlaneID11,2));    // Bernoulli’s lemniscate

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(intersection_check, intersect_sphere_cone2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateCone(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),2,0);
//    SGM::CreateSphere(rResult,SGM::Point3D(sqrt(2)/2,0,2+sqrt(2)/2),1);
//  
//    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,SGM_HALF_PI*0.5);
//    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(sqrt(2)/2,0,2+sqrt(2)/2),1);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,ConeID,SphereID,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 


TEST(intersection_check, intersect_sphere_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,SGM_HALF_PI*0.5);
    SGM::Surface ConeID2=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM::SAFEatan2(1,2));
    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,2),1);
    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,1),1);
    SGM::Surface SphereID3=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1.5);
    SGM::Surface SphereID4=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,3),1);
    SGM::Surface SphereID5=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,0,2),1);
    SGM::Surface SphereID6=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,0,1),1);
    SGM::Surface SphereID7=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,1,1),1);
    SGM::Surface SphereID8=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,0,1.5),1);
    SGM::Surface SphereID9=SGM::CreateSphereSurface(rResult,SGM::Point3D(2+sqrt(2)/2,0,sqrt(2)/2),1);
    SGM::Surface SphereID10=SGM::CreateSphereSurface(rResult,SGM::Point3D(2-sqrt(2)*2,0,-sqrt(2)*2),4);
    SGM::Surface SphereID11=SGM::CreateSphereSurface(rResult,SGM::Point3D(sqrt(2)/2,0,2+sqrt(2)/2),1);

    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID1,1));     // One Circle 
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID2,2));     // One Circle and a Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID3,2));     // Two Circles
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID4,1));     // One Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID5,1));     // Teardrop Curve
    EXPECT_TRUE(TestIntersections(rResult,ConeID2,SphereID6,2));    // Two Closed Curves
    EXPECT_TRUE(TestIntersections(rResult,ConeID2,SphereID7,1));    // One Closed Curve Cone C
    EXPECT_TRUE(TestIntersections(rResult,ConeID2,SphereID8,1));    // One Closed Curve Cone N
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID9,1));     // Tangent Point Sphere Outside
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID10,2));    // Two Curve in a Figure Eight
    EXPECT_TRUE(TestIntersections(rResult,ConeID,SphereID11,1));    // Apex on on Center Axis

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_cylinder_torus2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::CreateCylinder(rResult,SGM::Point3D(-4,1,0),SGM::Point3D(4,1,0),0.5);
//    
//    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
//    SGM::Surface CylinderID4=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,1,0),SGM::Point3D(4,1,0),0.5);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,TorusID,CylinderID4,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_cylinder_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Surface TorusID2=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,3);
    SGM::Surface CylinderID1=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,-2),SGM::Point3D(0,0,2),1);
    SGM::Surface CylinderID2=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,-2),SGM::Point3D(0,0,2),2);
    SGM::Surface CylinderID3=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,-0.5),SGM::Point3D(0,0,0.5),3);
    SGM::Surface CylinderID4=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-3,2,0),SGM::Point3D(3,2,0),1);
    SGM::Surface CylinderID5=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0.5*3,1,0.866025403784439*3),SGM::Point3D(-0.5*3,1,-0.866025403784439*3),2);
    SGM::Surface CylinderID6=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,0,2),SGM::Point3D(4,0,2),1);
    SGM::Surface CylinderID7=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,2,2),SGM::Point3D(4,2,2),1);
    SGM::Surface CylinderID8=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,0,0),SGM::Point3D(4,0,0),1);
    SGM::Surface CylinderID9=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-5,0,0),SGM::Point3D(5,0,0),1);
    SGM::Surface CylinderID10=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-5,1,0),SGM::Point3D(5,1,0),1);
    SGM::Surface CylinderID11=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2.75,0,2.1650635094610964),SGM::Point3D(-3.25,0,-1.2990381056766580),0.5);
    SGM::Surface CylinderID12=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,0,0),SGM::Point3D(4,0,0),0.5);
    SGM::Surface CylinderID13=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,2,0),SGM::Point3D(4,2,0),0.5);
    SGM::Surface CylinderID14=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,3,0),SGM::Point3D(4,3,0),0.5);
    SGM::Surface CylinderID15=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,1,0),SGM::Point3D(4,1,0),0.5);

    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID1,1));     // One Circle Inside
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID2,2));     // Two Circle 
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID3,1));     // One Circle Outside
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID4,3));     // Minor Circle Two Hermits
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID5,3));     // Villarceau Circle Two Hermits
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID6,2));     // Two Tangent Points on Top
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID7,1));     // One Tangent Point on Top
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID8,12));    // Six Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID2,CylinderID9,8));    // Four Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID2,CylinderID10,10));  // Five Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID11,3));    // Two Tangent Points
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID12,4));    // Four Closed Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID13,2));    // Two Closed Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID14,1));    // One Closed Curves
    EXPECT_TRUE(TestIntersections(rResult,TorusID,CylinderID15,3));    // Three Closed Curves

    // Add tests for one and two point tangents
    // Check on axis tests for the general case.

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_cylinder_cone2) 
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::CreateCone(rResult,SGM::Point3D(0,0,-4),SGM::Point3D(0,0,2),6,0);
//    SGM::Body BodyID=SGM::CreateCylinder(rResult,SGM::Point3D(2-sqrt(2.0)/2,-4,-sqrt(2.0)/2),SGM::Point3D(2-sqrt(2.0)/2,4,-sqrt(2.0)/2),1);
//  
//    SGM::Transform3D Trans(SGM::Point3D(2,0,0),SGM::UnitVector3D(1,0,1),SGM_HALF_PI*0.5);
//    SGM::TransformEntity(rResult,Trans,BodyID);
//
//    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,SGM_HALF_PI*0.5);
//    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2-sqrt(2.0)/2,-4,-sqrt(2.0)/2),SGM::Point3D(2-sqrt(2.0)/2,4,-sqrt(2.0)/2),1);
//    SGM::TransformEntity(rResult,Trans,CylinderID);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,ConeID,CylinderID,aCurves);
//    SGM::CreateEdge(rResult,aCurves[0]);
//    SGM::CreateEdge(rResult,aCurves[1]);
//
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_cylinder_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2,SGM_HALF_PI*0.5);
    SGM::Surface CylinderID1=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
    SGM::Surface CylinderID2=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2+sqrt(2.0),0,0),SGM::Point3D(0,0,2+sqrt(2.0)),1);
    SGM::Surface CylinderID3=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2+sqrt(2.0)/2,-3,sqrt(2.0)/2),SGM::Point3D(2+sqrt(2.0)/2,3,sqrt(2.0)/2),1);
    SGM::Transform3D Trans(SGM::Point3D(2,0,0),SGM::UnitVector3D(1,0,1),SGM_HALF_PI*0.5);
    SGM::TransformEntity(rResult,Trans,CylinderID3);
    SGM::Surface CylinderID4=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,3),SGM::Point3D(1,0,3),1);
    SGM::Surface CylinderID5=SGM::CreateCylinderSurface(rResult,SGM::Point3D(4-sqrt(2.0),0,-2),SGM::Point3D(-1,0,3-sqrt(2.0)),1);
    SGM::Surface CylinderID6=SGM::CreateCylinderSurface(rResult,SGM::Point3D(5-sqrt(2.0),0,-2),SGM::Point3D(-1,0,4-sqrt(2.0)),1);
    SGM::Surface CylinderID7=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-3,0,1),SGM::Point3D(3,0,1),1);
    SGM::Surface CylinderID8=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-3,1,2),SGM::Point3D(3,1,2),1);
    SGM::Surface CylinderID9=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-0.5,0.5,-2),SGM::Point3D(-0.5,0.5,3),1.5);
    SGM::Surface CylinderID10=SGM::CreateCylinderSurface(rResult,SGM::Point3D(4,1,-2),SGM::Point3D(0,1,2),1); 
    SGM::Surface CylinderID11=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,-3,1),SGM::Point3D(0,3,1),sqrt(2.0)/2.0); 
    SGM::Surface CylinderID12=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2-sqrt(2.0)/2,-4,-sqrt(2.0)/2),SGM::Point3D(2-sqrt(2.0)/2,4,-sqrt(2.0)/2),1);
    SGM::TransformEntity(rResult,Trans,CylinderID12);
    SGM::Surface CylinderID13=SGM::CreateCylinderSurface(rResult,SGM::Point3D(1.5,0,2),SGM::Point3D(1.5,0,-2),1);
    SGM::Surface CylinderID14=SGM::CreateCylinderSurface(rResult,SGM::Point3D(1,0,3),SGM::Point3D(1,0,-2),1);
    SGM::Surface CylinderID15=SGM::CreateCylinderSurface(rResult,SGM::Point3D(2,-3,1),SGM::Point3D(2,3,1),1);
    SGM::Surface CylinderID16=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-4,0,0),SGM::Point3D(4,0,0),1);
    SGM::Surface CylinderID17=SGM::CreateCylinderSurface(rResult,SGM::Point3D(-6,0,-1),SGM::Point3D(6,0,-1),2.5);

    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID1,1));   // Circle
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID2,1));   // Line Outside
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID3,1));   // Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID4,1));   // Non Tangent Point Outside
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID5,2));   // Line and Ellipse
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID6,1));   // Two Infinite ends.
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID7,2));   // Point and Pringle Case
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID8,1));   // Teardrop Case, Contractable on Cylinder
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID9,1));   // Cone N Cylinder N
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID10,2));  // One Infinite end and line
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID11,2));  // Two Ellipses
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID12,2));  // Two Teardrops and Tangent Point
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID13,1));  // Cone C Cylinder N
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID14,1));  // Cone C Cylinder N Teardrop
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID15,1));  // Cone C Cylinder C
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID16,2));  // Cone 2C Cylinder 2N
    EXPECT_TRUE(TestIntersections(rResult,ConeID,CylinderID17,2));  // Cone 2N Cylinder 2C

    SGMTesting::ReleaseTestThing(pThing);
} 

//TEST(intersection_check, intersect_sphere_sphere2) 
//{
//    //SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    //SGM::Result rResult(pThing);
//    //
//    //SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),4);
//    //SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),4);
//    //
//    //SGM::Surface SphereID1=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
//    //SGM::Surface SphereID2=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
//    //std::vector<SGM::Curve> aCurves;
//    //SGM::IntersectSurfaces(rResult,SphereID1,SphereID2,aCurves);
//    //for(auto pCurve : aCurves)
//    //    {
//    //    SGM::Interval1D Domain(-5,5);
//    //    SGM::CreateEdge(rResult,pCurve,&Domain);
//    //    }
//    //
//    //SGMTesting::ReleaseTestThing(pThing);
//
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    
//    SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
//    SGM::CreateSphere(rResult,SGM::Point3D(2,0,0),1);
//    
//    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
//    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(2,0,0),1);
//    std::vector<SGM::Curve> aCurves;
//    SGM::IntersectSurfaces(rResult,SphereID1,SphereID2,aCurves);
//    for(auto pCurve : aCurves)
//        {
//        SGM::CreateEdge(rResult,pCurve);
//        }
//    
//    SGMTesting::ReleaseTestThing(pThing);
//} 

TEST(intersection_check, intersect_sphere_sphere) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,0,0),1);
    SGM::Surface SphereID3=SGM::CreateSphereSurface(rResult,SGM::Point3D(2,0,0),1);
    SGM::Surface SphereID4=SGM::CreateSphereSurface(rResult,SGM::Point3D(0.5,0,0),0.5);

    EXPECT_TRUE(TestIntersections(rResult,SphereID1,SphereID2,1));  // Circle
    EXPECT_TRUE(TestIntersections(rResult,SphereID1,SphereID3,1));  // Point Outside
    EXPECT_TRUE(TestIntersections(rResult,SphereID1,SphereID4,1));  // Point Inside
    EXPECT_TRUE(TestIntersections(rResult,SphereID4,SphereID1,1));  // Testing other direction
    
    SGM::GetSurfaceType(rResult,SphereID1);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(intersection_check, plane_cone_intersection_circle)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0),SGM::Point3D(0,1,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::CircleType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_point)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlane(rResult,SGM::Point3D(0,0,1),SGM::Point3D(1,0,1),SGM::Point3D(0,1,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::PointCurveType);

    SGM::SaveSGM(rResult,"GTest_point_curve_test.sgm",aCurves[0],SGM::TranslatorOptions());

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_two_lines)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_EQ(aCurves.size(),2);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::LineType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_one_lines)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(1,0,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_EQ(aCurves.size(),1);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::LineType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_one_hyperbola)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::HyperbolaType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_one_parabola)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(-1,0,1),SGM::UnitVector3D(1,0,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::ParabolaType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cone_intersection_one_ellipse)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0.1,0,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,ConeID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::EllipseType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cylinder_intersection_ellipse)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0.1,0,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,CylinderID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::EllipseType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cylinder_intersection_circle)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,CylinderID,aCurves);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::CircleType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cylinder_intersection_one_line)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,CylinderID,aCurves);
    EXPECT_EQ(aCurves.size(),1);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::LineType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, plane_cylinder_intersection_two_line)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,PlaneID,CylinderID,aCurves);
    EXPECT_EQ(aCurves.size(),2);
    EXPECT_TRUE(SGM::GetCurveType(rResult,aCurves[0])==SGM::EntityType::LineType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, cylinder_sphere_intersect)
    {
    // Test sphere cylinder intersections

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(0,0,-10),Top(0,0,10),Pos0(1,0,0),Pos1(3,0,0),Pos2(0,0,0),Pos3(2,0,0),Pos4(4,0,0);
    double dRadius=2.0;

    rResult.SetLog(true);
    /*SGM::Body CylinderID=*/SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    std::vector<SGM::Entity> const &aLog=rResult.GetLogEntities();
    SGM::Face CylinderFace(aLog[0].m_ID);
    SGM::Surface CylinderSurf=SGM::GetSurfaceOfFace(rResult,CylinderFace);

    /*SGM::Body SphereID1=*/SGM::CreateSphere(rResult,Pos2,1.0);
    SGM::Face SphereFace1(aLog[3].m_ID);
    SGM::Surface SphereSurf1=SGM::GetSurfaceOfFace(rResult,SphereFace1);

    /*SGM::Body SphereID2=*/SGM::CreateSphere(rResult,Pos2,2.0);
    SGM::Face SphereFace2(aLog[4].m_ID);
    SGM::Surface SphereSurf2=SGM::GetSurfaceOfFace(rResult,SphereFace2);

    /*SGM::Body SphereID3=*/SGM::CreateSphere(rResult,Pos2,3.0);
    SGM::Face SphereFace3(aLog[5].m_ID);
    SGM::Surface SphereSurf3=SGM::GetSurfaceOfFace(rResult,SphereFace3);

    /*SGM::Body SphereID4=*/SGM::CreateSphere(rResult,Pos0,1.0);
    SGM::Face SphereFace4(aLog[6].m_ID);
    SGM::Surface SphereSurf4=SGM::GetSurfaceOfFace(rResult,SphereFace4);

    /*SGM::Body SphereID5=*/SGM::CreateSphere(rResult,Pos1,1.0);
    SGM::Face SphereFace5(aLog[7].m_ID);
    SGM::Surface SphereSurf5=SGM::GetSurfaceOfFace(rResult,SphereFace5);

    /*SGM::Body SphereID6=*/SGM::CreateSphere(rResult,Pos3,2.0);
    SGM::Face SphereFace6(aLog[8].m_ID);
    SGM::Surface SphereSurf6=SGM::GetSurfaceOfFace(rResult,SphereFace6);

    /*SGM::Body SphereID7=*/SGM::CreateSphere(rResult,Pos3,4.0);
    SGM::Face SphereFace7(aLog[9].m_ID);
    SGM::Surface SphereSurf7=SGM::GetSurfaceOfFace(rResult,SphereFace7);

    /*SGM::Body SphereID8=*/SGM::CreateSphere(rResult,Pos3,6.0);
    SGM::Face SphereFace8(aLog[10].m_ID);
    SGM::Surface SphereSurf8=SGM::GetSurfaceOfFace(rResult,SphereFace8);

    /*SGM::Body SphereID9=*/SGM::CreateSphere(rResult,Pos4,1.0);
    SGM::Face SphereFace9(aLog[11].m_ID);
    SGM::Surface SphereSurf9=SGM::GetSurfaceOfFace(rResult,SphereFace9);

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf1,0));    // Empty Inside

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf2,1));    // One Circle

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf3,2));    // Two Circles

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf4,1));    // Inside Point

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf5,1));    // Outside Point

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf6,1));    // Potato chip curve

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf7,2));    // Figure eight

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf8,2));    // Two Hermite

    EXPECT_TRUE(TestIntersections(rResult,CylinderSurf,SphereSurf9,0));    // Empty Outside

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(intersection_check, DISABLED_cylinder_circle_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Curve CircleID1=SGM::CreateCircle(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve CircleID2=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve CircleID3=SGM::CreateCircle(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,0,1),1);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    EXPECT_EQ( SGM::IntersectCurveAndSurface(rResult,CircleID1,CylinderID,aPoints,aTypes), 2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndSurface(rResult,CircleID2,CylinderID,aPoints,aTypes);
    EXPECT_EQ(aTypes[0],SGM::IntersectionType::CoincidentType);

    aPoints.clear();
    aTypes.clear();
    EXPECT_EQ( SGM::IntersectCurveAndSurface(rResult,CircleID3,CylinderID,aPoints,aTypes), 1);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(intersection_check, plane_circle_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Surface PlaneID1=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface PlaneID2=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Surface PlaneID3=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(1,0,0));

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    EXPECT_EQ( SGM::IntersectCurveAndSurface(rResult,CircleID,PlaneID1,aPoints,aTypes), 2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndSurface(rResult,CircleID,PlaneID2,aPoints,aTypes);
    EXPECT_EQ(aTypes[0],SGM::IntersectionType::CoincidentType);

    aPoints.clear();
    aTypes.clear();
    EXPECT_EQ( SGM::IntersectCurveAndSurface(rResult,CircleID,PlaneID3,aPoints,aTypes), 1);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(intersection_check, three_surface_intersect )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID1=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Surface SurfID2=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0));
    SGM::Surface SurfID3=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    SGM::IntersectThreeSurfaces(rResult,SurfID1,SurfID2,SurfID3,aPoints);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, tangent_spheres_intersect )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface SurfID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(2,0,0),1);
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SurfID1,SurfID2,aCurves);
    EXPECT_EQ(aCurves.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, DISABLED_circle_circle_tangent_intersections )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve CircleID1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve CircleID2=SGM::CreateCircle(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve CircleID3=SGM::CreateCircle(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(1,0,0),1);

    std::vector<SGM::Point3D> aPoints1,aPoints2,aPoints3,aPoints4;
    std::vector<SGM::IntersectionType> aTypes1,aTypes2,aTypes3,aTypes4;
    SGM::IntersectCurves(rResult,CircleID1,CircleID2,aPoints1,aTypes1);
    SGM::IntersectCurves(rResult,CircleID1,CircleID3,aPoints2,aTypes2);
    EXPECT_EQ(aPoints1.size(),1);
    EXPECT_EQ(aPoints2.size(),1);

    SGM::Point3D Pos(1,0,0);
    SGM::Curve PointID=SGM::CreatePointCurve(rResult,Pos);
    SGM::IntersectCurves(rResult,CircleID1,PointID,aPoints3,aTypes3);
    EXPECT_EQ(aPoints3.size(),1);
    SGM::IntersectCurves(rResult,PointID,CircleID1,aPoints4,aTypes4);
    EXPECT_EQ(aPoints4.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, bounded_parabola_plane_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Interval1D Domain(-1,1);
    SGM::Curve CurveID=SGM::CreateParabola(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,&Domain);
    SGM::Surface SurfID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndSurface(rResult,CurveID,SurfID,aPoints,aTypes);
    
    SGMTesting::ReleaseTestThing(pThing);
}


TEST(intersection_check, circle_and_line_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,CircleID,LineID,aPoints,aTypes);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, point_curve_surface_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve CurveID=SGM::CreatePointCurve(rResult,SGM::Point3D(0,0,0));
    SGM::CurveInverse(rResult,CurveID,SGM::Point3D(0,0,0));
    SGM::Surface PlaneID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndSurface(rResult,CurveID,PlaneID,aPoints,aTypes);
    
    SGMTesting::ReleaseTestThing(pThing);
}


TEST(intersection_check, intersect_ellipse_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(1,2,3);
    SGM::UnitVector3D XAxis(1,0,0);
    SGM::UnitVector3D YAxis(0,1,0);
    double dXRadius = 5;
    double dYRadius = 7;
    SGM::Curve EllipseCurve = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // coincident plane
    SGM::Point3D PlaneOrigin(1,2,3);
    SGM::UnitVector3D PlaneNormal(0,0,1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint = Center + 5*(XAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // plane rotated about the y axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(1,0,2);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint1 = Center - dYRadius*(YAxis);
        SGM::Point3D ExpectedPoint2 = Center + dYRadius*(YAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // ellipse rotated about z
    XAxis = SGM::UnitVector3D(1,2,0);
    YAxis = SGM::UnitVector3D(2,-1,0);
    SGM::Curve EllipseCurve2 = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // plane rotated about x axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(2,-1,1);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint1 = Center + dXRadius*(XAxis);
        SGM::Point3D ExpectedPoint2 = Center - dXRadius*(XAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // plane tangent to the ellipse
    SGM::Point3D PosOnEllipse{};
    SGM::Vector3D Tangent;
    double dUvalue = SGM_PI/5.0;
    SGM::EvaluateCurve(rResult, EllipseCurve2, dUvalue, &PosOnEllipse, &Tangent);

    SGM::UnitVector3D EllipseNormal = XAxis*YAxis;
    PlaneNormal = Tangent*EllipseNormal;
    PlaneOrigin = PosOnEllipse + EllipseNormal*11.0;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // plane misses ellipse by slightly more than tolerance
    PlaneOrigin = PlaneOrigin + (dTolerance*10.0)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // plane misses ellipse by slightly less than tolerance
    PlaneOrigin = PosOnEllipse + (dTolerance*0.1)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // simple case for easier debugging
    SGM::Point3D Center3(0,0,0);
    SGM::UnitVector3D XAxis3(1,1,0);
    SGM::UnitVector3D YAxis3(-1,1,0);
    double dXRadius3 = 2;
    double dYRadius3 = 1;
    SGM::Curve EllipseCurve3 = SGM::CreateEllipse(rResult, Center3, XAxis3, YAxis3, dXRadius3, dYRadius3);

    // tangent plane that just misses 
    SGM::UnitVector3D PlaneNormal3(1,1,0);
    SGM::Point3D PlaneOrigin3(sqrt(2.0), sqrt(2.0), 0);
    PlaneOrigin3 = PlaneOrigin3 + (dTolerance*10.0)*PlaneNormal3;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve3, PlaneOrigin3, PlaneNormal3, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_line_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos(-1,2,-3);
    SGM::UnitVector3D Direction(1,2,3);
    SGM::Curve LineID = SGM::CreateLine(rResult, Pos, Direction);

    SGM::Point3D PlaneOrigin(0,0,0);
    SGM::UnitVector3D PlaneNorm(-1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    SGM::Point3D ExpectedPos(0,4,0);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // coincident intersection
    PlaneOrigin = Pos + 0.5*SGM::Vector3D(1,2,3);
    PlaneNorm = Direction.Orthogonal();

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // parallel plane offset by more than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // parallel plane offset by less than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_circle_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(7,6,-5);
    SGM::UnitVector3D Normal(1,2,1);
    double dRadius=5.5;
    SGM::Curve CircleID = SGM::CreateCircle(rResult, Center, Normal, dRadius);

    // coincident plane
    SGM::Point3D PlaneOrigin = Center + 5*Normal.Orthogonal();
    SGM::UnitVector3D PlaneNorm(-1,-2,-1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // two intersection points
    PlaneOrigin = Center + SGM::Vector3D(1,0,0);
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    for (SGM::Point3D TestPos : aPoints)
    {
        EXPECT_NEAR(Center.Distance(TestPos), 5.5, dTolerance);
    }

    // tangent intersection
    SGM::Point3D PosOnCircle;
    SGM::UnitVector3D Tangent;
    SGM::EvaluateCurve(rResult, CircleID, SGM_PI*0.7, &PosOnCircle, &Tangent);
    PlaneOrigin = PosOnCircle + 3*Tangent;
    PlaneNorm = Tangent*Normal + Normal; // 45 degrees to plane of circle

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnCircle, aPoints[0], dTolerance));

    // plane misses by just more than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // plane misses by just less than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnCircle, aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_parabola_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(-3,2,1);
    SGM::UnitVector3D XAxis(1,0,2);
    SGM::UnitVector3D YAxis(2,0,-1);
    double dA = -0.3;
    SGM::Curve ParabolaID = SGM::CreateParabola(rResult, Center, XAxis, YAxis, dA);

    // coincident plane
    SGM::Point3D PlaneOrigin(6.5,2,16.2);
    SGM::UnitVector3D PlaneNorm(0,1,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
      EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
      EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // single clean intersection point
    PlaneNorm = XAxis;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // single tangent intersection point
    SGM::Point3D Pos;
    SGM::Vector3D Tangent;
    SGM::UnitVector3D Normal = (XAxis*YAxis);
    SGM::EvaluateCurve(rResult, ParabolaID, 1.4, &Pos, &Tangent);

    PlaneNorm = Normal*Tangent + 2*Normal;  // not perpendicular
    PlaneOrigin = Pos + 20.6*Tangent;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(aPoints[0], Pos, dTolerance));

    // two intersection points
    PlaneNorm = SGM::Vector3D(-1,0,3);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // parallel to tangent plane, but just outside of tolerance
    PlaneNorm = Normal*Tangent;
    PlaneOrigin = Pos + 7.7*Normal + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // tangent just within tolerance
    PlaneOrigin = Pos + 7.7*Normal + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // tangent just within tolerance - tilt the plane
    SGM::UnitVector3D TiltedNormal = PlaneNorm + 0.4*Normal;
    PlaneOrigin = Pos + 7.7*Tangent;
    PlaneOrigin = PlaneOrigin + (dTolerance*0.1)*TiltedNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);


    SGM::Point3D Center2(0.0,-1.0, -1.0);
    SGM::UnitVector3D XAxis2(0.62457719407916179, 0.78096307763952644, 0.0);
    SGM::UnitVector3D YAxis2(0.0, 0.0, -1.0);
    double dA2 = 0.63263978149317612;
    SGM::Curve ParabolaID2 = SGM::CreateParabola(rResult, Center2, XAxis2, YAxis2, dA2);

    //SGM::Interval1D Domain(-10.0, 10.0);
    //SGM::CreateEdge(rResult, ParabolaID2, &Domain);

    SGM::Point3D PlaneOrigin2(10.0, 0.0, -1.0);
    SGM::UnitVector3D PlaneNorm2(0.0, 0.0, 1.0);

    //SGM::CreateDisk(rResult, PlaneOrigin2, PlaneNorm2, 10.0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID2, PlaneOrigin2, PlaneNorm2, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(Center2, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_hyperbola_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0);
    SGM::UnitVector3D YAxis(0,1,0);
    double dA = 2.0;
    double dB = 1.0;
    SGM::Curve HyperbolaID = SGM::CreateHyperbola(rResult, Center, XAxis, YAxis, dA, dB);

    // coincident plane
    SGM::Point3D PlaneOrigin(0,10,0);
    SGM::UnitVector3D PlaneNorm(0,0,1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    
    // intersects two points
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(4.898979, 10, 0), aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-4.898979, 10, 0), aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // intersects one point
    PlaneOrigin = SGM::Point3D(-10,0,0);
    PlaneNorm = SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-10, 20.099751, 0), aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // hyperbola in y-z plane, not aligned with axes
    SGM::Point3D Center2(3,-4,-5);
    SGM::UnitVector3D XAxis2(0,2,1);
    SGM::UnitVector3D YAxis2(0,1,-2);
    double dA2 = .7;
    double dB2 = 1.1;
    SGM::Curve HyperbolaID2 = SGM::CreateHyperbola(rResult, Center2, XAxis2, YAxis2, dA2, dB2);

    PlaneOrigin = SGM::Point3D(0,0,-10);
    PlaneNorm = SGM::UnitVector3D(0,.75,-2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // tangent plane
    SGM::Point3D PosOnHyperbola;
    SGM::Vector3D Tangent;
    SGM::EvaluateCurve(rResult, HyperbolaID2, 1.5, &PosOnHyperbola, &Tangent);

    PlaneOrigin = PosOnHyperbola + 3*Tangent;
    PlaneNorm = Tangent * (XAxis2*YAxis2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnHyperbola, aPoints[0], dTolerance));

    // tangent offset less than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(PosOnHyperbola, aPoints[0], dTolerance));

    // tangent offset more than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_segment_and_segment)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D PosA(1,0,0);
    SGM::Point3D PosB(2,0,0);
    SGM::Point3D PosC(1.5,-1,0);
    SGM::Point3D PosD(1.5,1,0);

    SGM::Segment3D SegmentAB(PosA, PosB);
    SGM::Segment3D SegmentCD(PosC, PosD);

    SGM::Point3D PosAB;
    SGM::Point3D PosCD;
    double sAB;
    double tCD;
    SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    double dTolerance = SGM_MIN_TOL;
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosAB, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosCD, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.5, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // flipped order should give same results
    bool bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosCD, PosAB, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosAB, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosCD, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.5, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // parallel segments
    PosC = SGM::Point3D(1,1,0);
    PosD = SGM::Point3D(2,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel just overlaps at start points
    PosC = SGM::Point3D(1,1,0);
    PosD = SGM::Point3D(-1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel just overlaps at one start and one end point
    PosC = SGM::Point3D(0,1,0);
    PosD = SGM::Point3D(1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // parallel just overlaps at one start and one end point - reverse order
    PosC = SGM::Point3D(0,1,0);
    PosD = SGM::Point3D(1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentCD.Intersect(SegmentAB, PosCD, PosAB, &tCD, &sAB);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SGM::Point3D(0,0,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, -1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel no overlap
    PosC = SGM::Point3D(-1.5,1,0);
    PosD = SGM::Point3D(0.5,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SGM::Point3D(1,1,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.25, dTolerance, false));

    // parallel no overlap - again
    PosC = SGM::Point3D(2.5,1,0);
    PosD = SGM::Point3D(4,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SGM::Point3D(1,1,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, -1.0, dTolerance, false));

    // collinear no overlap
    PosC = SGM::Point3D(2.5,0,0);
    PosD = SGM::Point3D(4,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, -1.0, dTolerance, false));

    // collinear no overlap - reverse one
    PosC = SGM::Point3D(4,0,0);
    PosD = SGM::Point3D(2.5,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 2.0, dTolerance, false));

    // collinear - overlap at single point
    PosC = SGM::Point3D(2,0,0);
    PosD = SGM::Point3D(5,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // collinear - overlap at single point - reverse second curve
    PosC = SGM::Point3D(5,0,0);
    PosD = SGM::Point3D(2,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // collinear - overlap at single point - reverse first and second curve
    PosC = SGM::Point3D(5,0,0);
    PosD = SGM::Point3D(2,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;

    SGM::Segment3D SegmentABreversed(SegmentAB.m_End, SegmentAB.m_Start);
    
    bSegmentsIntersect = SegmentABreversed.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentABreversed.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // collinear - overlap
    PosC = SGM::Point3D(1.75,0,0);
    PosD = SGM::Point3D(3.75,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.125, dTolerance, false));

    // collinear - first within second
    PosC = SGM::Point3D(-0.25,0,0);
    PosD = SGM::Point3D(2.25,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // collinear - second within first
    bSegmentsIntersect = SegmentCD.Intersect(SegmentAB, PosCD, PosAB, &tCD, &sAB);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_planar_NUBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aInterpolate;
    aInterpolate.reserve(5);

    aInterpolate.emplace_back(0,-2,0);
    aInterpolate.emplace_back(0,-1,0.5);
    aInterpolate.emplace_back(0,0.0,0.25);
    aInterpolate.emplace_back(0,1,0.1);
    aInterpolate.emplace_back(0,2,.5);
    
    SGM::Curve NUBcurveID = SGM::CreateNUBCurve(rResult, aInterpolate);

    // plane with single intersection
    SGM::Point3D PlaneOrigin(1,1,0);
    SGM::UnitVector3D PlaneNormal(0,1,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(0,1,0.1), aPoints[0], dTolerance));

    // coincident plane
    PlaneOrigin = SGM::Point3D(0,21,0);
    PlaneNormal = SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    SGM::Point3D NUBstart;
    SGM::Point3D NUBend;
    SGM::EvaluateCurve(rResult, NUBcurveID, SGM::GetCurveDomain(rResult, NUBcurveID).m_dMin,&NUBstart);
    SGM::EvaluateCurve(rResult, NUBcurveID, SGM::GetCurveDomain(rResult, NUBcurveID).m_dMax,&NUBend);
    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(NUBstart, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(NUBend, aPoints[1], dTolerance));
    }

    // multiple intersection points
    PlaneOrigin = SGM::Point3D(1,-2,0);
    PlaneNormal = SGM::UnitVector3D(0,-1,4);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (!aPoints.empty())
    {
        // just checking the first point
        EXPECT_TRUE(SGM::NearEqual(aInterpolate[0], aPoints[0], dTolerance));
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_nonplanar_NUBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aInterpolate;
    aInterpolate.reserve(6);

    aInterpolate.emplace_back(-2,3.01,0.01);
    aInterpolate.emplace_back(-1,4,0);
    aInterpolate.emplace_back(0,5,0);
    aInterpolate.emplace_back(1,6,0);
    aInterpolate.emplace_back(2,7,0);
    aInterpolate.emplace_back(3,8,0);
    
    SGM::Curve NUBcurveID = SGM::CreateNUBCurve(rResult, aInterpolate);

    SGM::Point3D PlaneOrigin(1,1,0);
    SGM::UnitVector3D PlaneNormal(1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,6,0), aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // tangent intersection - touches without crossing plane
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,-2,-3);
    aInterpolate.emplace_back(-1,-2,-2);
    aInterpolate.emplace_back(0,-1,-1);
    aInterpolate.emplace_back(1,0,-2);
    aInterpolate.emplace_back(2,0,-3);

    SGM::Curve NUBcurveID2 = SGM::CreateNUBCurve(rResult, aInterpolate);
    //SGM::CreateEdge(rResult, NUBcurveID2);

    PlaneOrigin = SGM::Point3D(10,0,-1);
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(0,-1,-1), aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // single intersection point
    SGM::Point3D CurvePos;
    SGM::Vector3D CurveTangent;
    SGM::EvaluateCurve(rResult, NUBcurveID2, 0.3, &CurvePos, &CurveTangent);
    SGM::Vector3D Curvature = SGM::CurveCurvature(rResult, NUBcurveID2, 0.3);
    double dA = Curvature.Magnitude() * 0.5;

    /* SGM::Curve ParabolaID = */ SGM::CreateParabola(rResult, CurvePos, CurveTangent, Curvature, dA);
    //SGM::Interval1D Domain(-5,5);
    //SGM::CreateEdge(rResult, ParabolaID, &Domain);

    PlaneOrigin = SGM::Point3D(CurvePos.m_x,0,0);
    PlaneNormal = SGM::UnitVector3D(-1,0,0);

    //SGM::CreateDisk(rResult, PlaneOrigin, PlaneNormal, 5.0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // two intersection points
    PlaneOrigin = SGM::Point3D(11,4,CurvePos.m_z);
    PlaneNormal = SGM::UnitVector3D(0,0,-1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
        // expected 2nd intersection point
        // x is symmetric about 0 so change sign
        // y is symmetric about -1 so compute difference and add back twice
        // z is equal
        SGM::Point3D ExpectedPos(-CurvePos.m_x,CurvePos.m_y+2*(-1-CurvePos.m_y),CurvePos.m_z);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // 1 tangent intersection and one point intersection
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,2,-2);
    aInterpolate.emplace_back(-1.3,2.5,-2);
    aInterpolate.emplace_back(-1.3,2,-2);
    aInterpolate.emplace_back(-1.3,1.5,-2);
    aInterpolate.emplace_back(-0.6,2,-1);
    SGM::Curve NUBcurveID3 = SGM::CreateNUBCurve(rResult, aInterpolate);

    SGM::Vector3D CurveD1, CurveD2;
    SGM::EvaluateCurve(rResult, NUBcurveID3, 0.35, &CurvePos, &CurveD1, &CurveD2);
    PlaneOrigin = CurvePos+.5*CurveD1;
    PlaneNormal = (CurveD1*CurveD2)*CurveD1;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID3, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }


    // tangent intersection - crossing plane
    aInterpolate.clear();
    aInterpolate.emplace_back(-5 ,3,21.2);
    aInterpolate.emplace_back(0  ,4,21);
    aInterpolate.emplace_back(4.5,4,21);
    aInterpolate.emplace_back(5.5,2,21);
    aInterpolate.emplace_back(10 ,2,21);
    aInterpolate.emplace_back(15 ,3,20.8);
    SGM::Curve NUBcurveID4 = SGM::CreateNUBCurve(rResult, aInterpolate);

    //SGM::CreateEdge(rResult, NUBcurveID4);

    SGM::EvaluateCurve(rResult, NUBcurveID4, 0.5, &CurvePos, &CurveD1, &CurveD2);
    PlaneOrigin = CurvePos + CurveD1;
    PlaneNormal = CurveD1.Orthogonal();

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID4, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // near miss - outside tolerance
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,-2,-3);
    aInterpolate.emplace_back(-1.8,-2,-2);
    aInterpolate.emplace_back(-1.6,-1.9,-1);
    aInterpolate.emplace_back(-1.4,-1.8,-2);
    aInterpolate.emplace_back(-1.2,-1.8,-3);
    SGM::Curve NUBcurveID5 = SGM::CreateNUBCurve(rResult, aInterpolate);

    PlaneOrigin = SGM::Point3D(10,0,-1+(5*dTolerance));
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID5, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // near duplicate, but distinct
    PlaneOrigin = SGM::Point3D(10,0,-1.0000005);
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID5, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_pointcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos(55,123,42);
    SGM::Curve PointCurveID = SGM::CreatePointCurve(rResult, Pos);

    SGM::Point3D Origin(-1,1,-1);
    SGM::Point3D PlaneOrigin = Origin;
    SGM::UnitVector3D PlaneNorm = (Pos-Origin).Orthogonal();

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));

    PlaneOrigin = Origin + 0.1*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    PlaneOrigin = Origin - (0.2e-6)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_planar_NURBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(1,1,-1,1);
    aControlPoints.emplace_back(1,1,-2,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,-2,1);
    aControlPoints.emplace_back(1,-1,-2,sqrt(2)/2);
    aControlPoints.emplace_back(1,-1,-3,1);
        
    std::vector<double> aKnots;
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);

    SGM::Curve NURBcurve = SGM::CreateNURBCurve(rResult, aControlPoints, aKnots);

    // coincident plane
    SGM::Point3D PlaneOrigin(1,0,0);
    SGM::UnitVector3D PlaneNorm (-1,0,0);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    if (aPoints.size() > 1)
    {
      SGM::Point3D ExpectedPos1, ExpectedPos2;
      SGM::Interval1D Domain = SGM::GetDomainOfCurve(rResult, NURBcurve);
      SGM::EvaluateCurve(rResult, NURBcurve, Domain.m_dMin, &ExpectedPos1);
      SGM::EvaluateCurve(rResult, NURBcurve, Domain.m_dMax, &ExpectedPos2);
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos1, aPoints[0], 1E-4));
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos2, aPoints[1], 1E-4));
    }

    // single intersection point
    PlaneOrigin = SGM::Point3D(0, 0, (sqrt(2)/2.0-1));
    PlaneNorm = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    if (!aPoints.empty())
    {
      SGM::Point3D ExpectedPos(1, sqrt(2)/2.0, (sqrt(2)/2.0-1.0));
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], 1E-4));
    }

    // multiple intersection points
    PlaneOrigin = SGM::Point3D(1,-0.1,-2);
    PlaneNorm = SGM::UnitVector3D(0,-1,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes);

    EXPECT_EQ(aPoints.size(), 3);
    EXPECT_EQ(aTypes.size(), 3);
    for (auto const &Type : aTypes)
        EXPECT_EQ(Type, SGM::IntersectionType::PointType);

    // tangent intersection point
    PlaneOrigin = SGM::Point3D(5, sqrt(2)/2.0, (sqrt(2)/2.0-1.0));
    PlaneNorm = SGM::UnitVector3D(0,1,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, 1E-5);

    SGM::Point3D ClosePos, Pos;
    SGM::Vector3D Tangent;
    double dNURBt = SGM::CurveInverse(rResult, NURBcurve, SGM::Point3D(0, sqrt(2)/2.0, (sqrt(2)/2.0-1.0)), &ClosePos);
    SGM::EvaluateCurve(rResult, NURBcurve, dNURBt, &Pos, &Tangent);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (!aPoints.empty())
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1, sqrt(2)/2.0, (sqrt(2)/2.0-1.0)), aPoints[0], 1E-4));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_nonplanar_NURBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,1,-1,1);
    aControlPoints.emplace_back(1,1,-2,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,0,-2,1);
    aControlPoints.emplace_back(1,-1,-2,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,-1,-3,1);
    aControlPoints.emplace_back(1,-1,-4,sqrt(2)/2.0);
    aControlPoints.emplace_back(2,-1,-4,1);
    //aControlPoints.emplace_back(0,1,0,1);
    //aControlPoints.emplace_back(-1,1,0,sqrt(2)/2);
    //aControlPoints.emplace_back(-1,1,-1,1);
    //aControlPoints.emplace_back(-1,1,-2,sqrt(2)/2);
    //aControlPoints.emplace_back(-1,0,-2,1);
        
    std::vector<double> aKnots;
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_TWO_PI);
    aKnots.push_back(SGM_TWO_PI);
    aKnots.push_back(SGM_TWO_PI);

    SGM::Curve NURBcurve = SGM::CreateNURBCurve(rResult, aControlPoints, aKnots);

    // one intersection point
    SGM::Point3D PlaneOrigin(1.5,0,-1.5);
    SGM::UnitVector3D PlaneNorm (-1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (!aTypes.empty())
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    }
    if (!aPoints.empty())
    {
        EXPECT_TRUE(SGM::NearEqual(aPoints[0].m_x, 1.5, dTolerance, false));
        EXPECT_TRUE(SGM::NearEqual(aPoints[0].m_y, -1.0, dTolerance, false));
    }

    // two intersection points
    PlaneOrigin = SGM::Point3D(0,0,-1.5);
    PlaneNorm = SGM::UnitVector3D(0,-1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (aPoints.size() > 1)
    {
      EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,0,0), aPoints[0], dTolerance));
      EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,0,-2), aPoints[1], dTolerance));
    }

    // two intersections, one tangent
    SGM::Point3D CurvePos;
    SGM::Vector3D CurveTangent;
    SGM::EvaluateCurve(rResult, NURBcurve, SGM_PI*0.8, &CurvePos, &CurveTangent);
    PlaneOrigin = CurvePos + SGM::Vector3D(-5,0,0);
    PlaneNorm = CurveTangent * SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (aPoints.size() > 1)
    {
      EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    }

    //SGM::Interval1D domain = SGM::GetDomainOfCurve(rResult, NURBcurve);
    //SGM::CreateEdge(rResult, NURBcurve, &domain);
    //SGM::Curve curve = SGM::CreatePointCurve(rResult, aPoints[0]);
    //SGM::CreateEdge(rResult, curve);
    //curve = SGM::CreatePointCurve(rResult, aPoints[1]);
    //SGM::CreateEdge(rResult, curve);

    //SGM::CreateDisk(rResult, aPoints[0], PlaneNorm, 3);

    SGMTesting::ReleaseTestThing(pThing); 
}


TEST(intersection_check, intersect_line_and_cylinder)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(0,0,-5),Top(0,0,5);
    double dRadius=7.0;
    SGM::Body BodyID = CreateCylinder(rResult, Bottom, Top, dRadius);
    
    SGM::Point3D Origin(4,-4,0);
    SGM::UnitVector3D Axis(1,-1,0);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
      
    double dTolerance = SGM_MIN_TOL;
    const bool bUseWholeLine = false;

    RayFire(rResult,Origin,Axis, BodyID, aPoints, aTypes, dTolerance, bUseWholeLine);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);

    SGM::Point3D ExpectedPos(sqrt(49./2.),-sqrt(49./2.),0);

    EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    /* SGM::Curve LineID = */ SGM::CreateLine(rResult, Origin, Axis);
    //SGM::Interval1D LineInterval(0.0, 10.0); 
    //SGM::CreateEdge(rResult, LineID, &LineInterval);

    /* SGM::Curve PointCurveID = */ SGM::CreatePointCurve(rResult, aPoints[0]);
    //SGM::CreateEdge(rResult, PointCurveID);

    SGMTesting::ReleaseTestThing(pThing); 
}

//#pragma clang diagnostic pop
