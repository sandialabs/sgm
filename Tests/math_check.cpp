#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"

#include "test_utility.h"

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

TEST(math_check, quartic_equation)
{
    // Test the quartic equation.

    bool bAnswer=true;

    // 2*(x-1)(x-2)(x-3)(x-4) -> 2*x^4-20*x^3+70*x^2-100*x+48 Four roots

    std::vector<double> aRoots;
    size_t nRoots=SGM::Quartic(2,-20,70,-100,48,aRoots,SGM_MIN_TOL);
    if( nRoots!=4 || 
        SGM_ZERO<fabs(aRoots[0]-1) ||
        SGM_ZERO<fabs(aRoots[1]-2) ||
        SGM_ZERO<fabs(aRoots[2]-3) ||
        SGM_ZERO<fabs(aRoots[3]-4))
        {
        bAnswer=false;
        }

    // (x-1)(x-2)(x-3)(x-3) -> x^4-9*x^3+29*x^2-39*x+18 Three roots, one double

    aRoots.clear();
    nRoots=SGM::Quartic(1,-9,29,-39,18,aRoots,SGM_MIN_TOL);
    if( nRoots!=3 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2) ||
        SGM_ZERO<fabs(aRoots[2]-3))
        {
        bAnswer=false;
        }

    // (x-1)(x-2)(x-2)(x-2) -> x^4-7*x^3+18*x^2-20*x+8 Two roots, one triple

    aRoots.clear();
    nRoots=SGM::Quartic(1,-7,18,-20,8,aRoots,SGM_MIN_TOL);
    if( nRoots!=2 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2))
        {
        bAnswer=false;
        }

    // (x-1)(x-1)(x-2)(x-2) -> x^4-6*x^3+13*x^2-12*x+4 Two double roots

    aRoots.clear();
    nRoots=SGM::Quartic(1,-6,13,-12,4,aRoots,SGM_MIN_TOL);
    if( nRoots!=2 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2))
        {
        bAnswer=false;
        }

    // (x-1)(x-2)(x^2+1) -> x^4-3*x^3+3*x^2-3*x+2 Two roots

    aRoots.clear();
    nRoots=SGM::Quartic(1,-3,3,-3,2,aRoots,SGM_MIN_TOL);
    if( nRoots!=2 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2))
        {
        bAnswer=false;
        }

    // (x-1)(x-1)(x^2+1) -> x^4-2*x^3+2*x^2-2*x+1 One double root.

    aRoots.clear();
    nRoots=SGM::Quartic(1,-2,2,-2,1,aRoots,SGM_MIN_TOL);
    if( nRoots!=1 || 
        SGM_ZERO<fabs(aRoots[0]-1))
        {
        bAnswer=false;
        }

    // (x-1)(x-1)(x-1)(x-1) -> x^4-4*x^3+6*x^2-4*x+1 One quadruple root.

    aRoots.clear();
    nRoots=SGM::Quartic(1,-4,6,-4,1,aRoots,SGM_MIN_TOL);
    if( nRoots!=1 || 
        SGM_ZERO<fabs(aRoots[0]-1))
        {
        bAnswer=false;
        }

    // (x^2+1)(x^2+1) -> x^4+2*x^2+1 No roots.

    aRoots.clear();
    nRoots=SGM::Quartic(1,0,2,0,1,aRoots,SGM_MIN_TOL);
    if( nRoots!=0 )
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);
}

TEST(math_check, cubic_equation)
    {
    // Test the cubic equation.

    bool bAnswer=true;

    // 2*(x-1)*(x-2)*(x-3)=0 -> 2*x^3-12*x^2+22*x-12=0 Three roots

    std::vector<double> aRoots;
    size_t nRoots=SGM::Cubic(2,-12,22,-12,aRoots);
    if( nRoots!=3 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2) ||
        SGM_ZERO<fabs(aRoots[2]-3))
        {
        bAnswer=false;
        }

    // (x-1)*(x-2)*(x-2)=0 -> x^3-5*x^2+8*x-4=0 Two roots, one double

    aRoots.clear();
    nRoots=SGM::Cubic(1,-5,8,-4,aRoots);
    if( nRoots!=2 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2))
        {
        bAnswer=false;
        }

    // (x-1)*(x^2+1)=0 -> x^3-x^2+x-1=0 One root

    aRoots.clear();
    nRoots=SGM::Cubic(1,-1,1,-1,aRoots);
    if( nRoots!=1 || 
        SGM_ZERO<fabs(aRoots[0]-1))
        {
        bAnswer=false;
        }

    // (x-1)*(x-1)*(x-1)=0 -> x^3-x^2+x-1=0 One triple root

    aRoots.clear();
    nRoots=SGM::Cubic(1,-3,3,-1,aRoots);
    if( nRoots!=1 || 
        SGM_ZERO<fabs(aRoots[0]-1))
        {
        bAnswer=false;
        }

    // (x-1)*(x-2)=0 -> x^2-3*x+2=0 Two roots and degenerate

    aRoots.clear();
    nRoots=SGM::Cubic(0,1,-3,2,aRoots);
    if( nRoots!=2 || 
        SGM_ZERO<fabs(aRoots[0]-1) || 
        SGM_ZERO<fabs(aRoots[1]-2))
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);  
    }

TEST(surface_check, plane)
    {
    // Test plane inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D XAxis(1,2,3);
    SGM::UnitVector3D YAxis=XAxis.Orthogonal();
    SGM::Surface PlaneID=SGM::CreatePlane(rResult,Origin,Origin+XAxis,Origin+YAxis);

    bool bAnswer=SGM::TestSurface(rResult,PlaneID,SGM::Point2D(0.5,0.2));
    SGM::DeleteEntity(rResult,PlaneID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(surface_check, sphere)
    {
    // Test sphere inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D XAxis(1,2,3);
    SGM::UnitVector3D YAxis=XAxis.Orthogonal();
    double dRadius=2.5;
    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,Origin,dRadius,&XAxis,&YAxis);

    bool bAnswer=SGM::TestSurface(rResult,SphereID,SGM::Point2D(0.5,0.2));
    SGM::DeleteEntity(rResult,SphereID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(surface_check, cylinder)
    {
    // Test cylinder inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(10,11,12),Top(13,14,15);
    double dRadius=2.5;
    SGM::Surface SphereID=SGM::CreateCylinderSurface(rResult,Bottom,Top,dRadius);

    bool bAnswer=SGM::TestSurface(rResult,SphereID,SGM::Point2D(0.5,0.2));
    SGM::DeleteEntity(rResult,SphereID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(surface_check, torus)
    {
    // Test torus inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(0,0,0);
    SGM::UnitVector3D ZAxis(0,0,1);
    SGM::Surface SphereID=SGM::CreateTorusSurface(rResult,Origin,ZAxis,2,5,true);

    bool bAnswer=SGM::TestSurface(rResult,SphereID,SGM::Point2D(0.5,0.2));
    SGM::DeleteEntity(rResult,SphereID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(surface_check, cone)
    {
    // Test cone inverse.
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D ZAxis(1,2,3);
    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,Origin,ZAxis,2,0.4);

    bool bAnswer=SGM::TestSurface(rResult,ConeID,SGM::Point2D(0.5,0.2));
    SGM::DeleteEntity(rResult,ConeID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }


TEST(curve_check, NUB)
    {
    // Test NUB Curve inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    std::vector<double> aKnots={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints;
    aControlPoints.emplace_back(1,1,0);
    aControlPoints.emplace_back(1.166666666666666,1.166666666666666,0);
    aControlPoints.emplace_back(2,2.8333333333333333,0);
    aControlPoints.emplace_back(2.8333333333333333,1.166666666666666,0);
    aControlPoints.emplace_back(3,1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints,aKnots);

    bool bAnswer=SGM::TestCurve(rResult,NUBID,0.45);
    SGM::DeleteEntity(rResult,NUBID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(curve_check, line)
    {
    // Test Line

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos0(1,2,3),Pos1(4,5,6);
    SGM::UnitVector3D Axis(7,8,9);

    SGM::Curve LineID=SGM::CreateLine(rResult,Pos0,Axis);
    SGM::Curve CopyID=SGM::Curve(SGM::CopyEntity(rResult,LineID).m_ID);

    SGM::EvaluateCurve(rResult,LineID,0.45,&Pos0);
    SGM::EvaluateCurve(rResult,CopyID,0.45,&Pos1);

    EXPECT_TRUE(SGM::NearEqual(Pos0,Pos1,SGM_ZERO));

    bool bAnswer=SGM::TestCurve(rResult,LineID,0.5);
    SGM::DeleteEntity(rResult,LineID);
    SGM::DeleteEntity(rResult,CopyID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(curve_check, circle)
    {
    // Test Circle

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(1,2,3);
    SGM::UnitVector3D Normal(4,5,6);
    double dRadius=2.1;

    SGM::Curve CircleID=SGM::CreateCircle(rResult,Center,Normal,dRadius);
    SGM::Curve CopyID=SGM::Curve(SGM::CopyEntity(rResult,CircleID).m_ID);

    SGM::Point3D Pos0,Pos1;
    SGM::EvaluateCurve(rResult,CircleID,0.45,&Pos0);
    SGM::EvaluateCurve(rResult,CopyID,0.45,&Pos1);

    EXPECT_TRUE(SGM::NearEqual(Pos0,Pos1,SGM_ZERO));

    bool bAnswer=SGM::TestCurve(rResult,CircleID,0.45);
    SGM::DeleteEntity(rResult,CircleID);
    SGM::DeleteEntity(rResult,CopyID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(surface_check, principle_curvature)
    {
    // Test Principle Curvature

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    SGM::Point3D Center(0.0,0.0,0.0);
    SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,Center,ZAxis,2,5,true);
    SGM::Point2D uv(0.0,0.0);
    SGM::UnitVector3D Vec1,Vec2;
    double k1,k2;
    SGM::PrincipleCurvature(rResult,TorusID,uv,Vec1,Vec2,k1,k2);

    if(SGM::NearEqual(Vec1,SGM::UnitVector3D(0.0,1.0,0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec2,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(k1,-7.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(k2,-2.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }

    SGM::DeleteEntity(rResult,TorusID);
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }