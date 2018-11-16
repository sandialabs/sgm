#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"
#include "SGMGraph.h"
#include "SGMComplex.h"
#include "SGMTransform.h"
#include "SGMIntersector.h"
#include "SGMMeasure.h"
#include "SGMInterrogate.h"
#include "SGMTopology.h"
#include "SGMModify.h"
#include "SGMDisplay.h"

#define SGM_TIMER 
#include "Timer.h"

#include "test_utility.h"

TEST(math_check, save_step_primitives)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    SGM::CreateCylinder(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::CreateCone(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1,2);
    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,3);

    SGM::TranslatorOptions TranslatorOpts;
    SGM::SaveSTEP(rResult,"primitives.stp",SGM::Thing(),TranslatorOpts);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(math_check, revolve_surface_save_step)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1;
    aPoints1.emplace_back(-2,.5,0);
    aPoints1.emplace_back(-1,1.5,0);
    aPoints1.emplace_back(0,1,0);
    aPoints1.emplace_back(1,1.5,0);
    aPoints1.emplace_back(2,2,0);

    // simple case
    //aPoints1.push_back(SGM::Point3D(-2,.5,0));
    //aPoints1.push_back(SGM::Point3D(-1,1.5,0));
    //aPoints1.push_back(SGM::Point3D(0,.5,0));
    //aPoints1.push_back(SGM::Point3D(1,.5,0));
    //aPoints1.push_back(SGM::Point3D(2,.5,0));

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Origin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Surface RevolveID=SGM::CreateRevolveSurface(rResult,Origin,Axis,CurveID);

    std::vector<SGM::Edge> aEdges;
    std::vector<SGM::EdgeSideType> aTypes;
    SGM::Body BodyID=SGM::CreateSheetBody(rResult,RevolveID,aEdges,aTypes);
    
    SGM::TranslatorOptions TranslatorOpts;
    SGM::SaveSTEP(rResult,"revolve_sheet.stp",SGM::Thing(),TranslatorOpts);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, revolve_surface_test)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Revolve Surface

    bool bAnswer=true;

    // relatively easy testcase to debug
    // NUB curve is a straight line parallel to the axis
    // both have a slope of 2:1, and distance between is sqrt(5.0)
    std::vector<double> aKnots1={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints1;
    aControlPoints1.emplace_back(3.5,1,0);
    aControlPoints1.emplace_back(3.75,1.5,0);
    aControlPoints1.emplace_back(4,2,0);
    aControlPoints1.emplace_back(4.25,2.5,0);
    aControlPoints1.emplace_back(4.5,3,0);

    SGM::Curve NUBID1=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints1,aKnots1);

    SGM::Point3D Origin1(1.0,1.0,0.0);
    SGM::UnitVector3D Axis1(1.0,2.0,0.0);
    SGM::Surface RevolveID1=SGM::CreateRevolveSurface(rResult,Origin1,Axis1,NUBID1);

    SGM::Point3D Pos;
    SGM::Vector3D Du, Dv;
    SGM::EvaluateSurface(rResult,RevolveID1,SGM::Point2D(SGM_PI/2.0, 0.5), &Pos, &Du, &Dv);

    bool bAnswer1 = true;
    double dDistance = sqrt(5.0);
    if(SGM::NearEqual(Pos, SGM::Point3D(2.0,3.0,-dDistance), SGM_ZERO)==false)
        {
        bAnswer1=false;
        }
    SGM::UnitVector3D uDuDirection(-2,1,0);
    SGM::UnitVector3D UnitDu = Du;
    if(SGM::NearEqual(uDuDirection,UnitDu,SGM_ZERO)==false)
        {
        bAnswer1=false;
        }
    SGM::UnitVector3D uDvDirection(1,2,0);
    SGM::UnitVector3D UnitDv = Dv;
    if(SGM::NearEqual(uDvDirection,UnitDv,SGM_ZERO)==false)
        {
        bAnswer1=false;
        }

    // create a more general NUB to revolve
    std::vector<double> aKnots2={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints2;
    aControlPoints2.emplace_back(1,1,0);
    aControlPoints2.emplace_back(1.166666666666666,1.166666666666666,0);
    aControlPoints2.emplace_back(2,2.8333333333333333,0);
    aControlPoints2.emplace_back(2.8333333333333333,2.8333333333333333,0);
    aControlPoints2.emplace_back(3,3,0);

    SGM::Curve NUBID2=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints2,aKnots2);

    SGM::Point3D Origin2(1.0,3.0,0.0);
    SGM::UnitVector3D Axis2(1.0,2.0,0.0);
    SGM::Surface RevolveID2=SGM::CreateRevolveSurface(rResult,Origin2,Axis2,NUBID2);

    bool bAnswer2 = SGM::TestSurface(rResult,RevolveID2, SGM::Point2D(0.5,0.2));

    bAnswer = (bAnswer1 && bAnswer2);

    SGM::DeleteEntity(rResult,RevolveID1);
    SGM::DeleteEntity(rResult,RevolveID2);
    SGM::DeleteEntity(rResult,NUBID1);
    SGM::DeleteEntity(rResult,NUBID2);

    EXPECT_TRUE(bAnswer);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, extrude_hermit)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::Vector3D> aVectors;
    std::vector<double> aParams;
    aPoints.push_back(SGM::Point3D(0,0,0));
    aPoints.push_back(SGM::Point3D(10,0,0));
    aVectors.push_back(SGM::Vector3D(1,1,0));
    aVectors.push_back(SGM::Vector3D(1,1,0));
    aParams.push_back(0);
    aParams.push_back(12);
    SGM::Curve CurveID=SGM::CreateHermitCurve(rResult,aPoints,aVectors,aParams);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,6));
    SGM::UnitVector3D Axis(0,0,1);
    SGM::Surface SurfID=SGM::CreateExtrudeSurface(rResult,Axis,CurveID);
    EXPECT_TRUE(SGM::TestSurface(rResult,SurfID,SGM::Point2D(6,1)));

    SGM::DeleteEntity(rResult,SurfID);
    SGM::DeleteEntity(rResult,CurveID);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, closest_point)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateCylinder(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,2),1);
    SGM::Point3D Pos(1,0,3);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    auto iter=sFaces.begin();
    SGM::Face FaceID1=*iter;
    ++iter;
    SGM::Face FaceID2=*iter;
    ++iter;
    SGM::Face FaceID3=*iter;

    SGM::Point3D CPos1,CPos2,CPos3;
    SGM::Entity Ent1,Ent2,Ent3;
    SGM::FindClosestPointOnEntity(rResult,Pos,FaceID1,CPos1,Ent1);
    SGM::FindClosestPointOnEntity(rResult,Pos,FaceID2,CPos2,Ent2);
    SGM::FindClosestPointOnEntity(rResult,Pos,FaceID3,CPos3,Ent3);

    EXPECT_TRUE(SGM::NearEqual(CPos1,SGM::Point3D(1,0,2),SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(CPos2,SGM::Point3D(1,0,0),SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(CPos3,SGM::Point3D(1,0,2),SGM_MIN_TOL));
      
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, rectangle)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point2D Pos0(0,0),Pos1(1,1);
    SGM::Complex RectangleID=SGM::CreateRectangle(rResult,Pos0,Pos1,false);
    double dLength=SGM::FindComplexLength(rResult,RectangleID);
    bool bTest1=SGM::NearEqual(dLength,4.0,SGM_ZERO,false);
    EXPECT_TRUE(bTest1);

    EXPECT_TRUE(SGM::IsCycle(rResult,RectangleID));
    SGM::Point3D Origin;
    SGM::UnitVector3D Normal;
    EXPECT_TRUE(SGM::IsPlanar(rResult,RectangleID,Origin,Normal,SGM_MIN_TOL));
    EXPECT_TRUE(SGM::IsConnected(rResult,RectangleID));
    EXPECT_FALSE(SGM::IsLinear(rResult,RectangleID));

    SGM::Complex FilledID=SGM::CreateRectangle(rResult,Pos0,Pos1,true);
    double dArea=SGM::FindComplexArea(rResult,FilledID);
    bool bTest2=SGM::NearEqual(dArea,1.0,SGM_ZERO,false);
    EXPECT_TRUE(bTest2);

    SGM::FindDegenerateTriangles(rResult,FilledID);
    SGM::GetBoundingBox(rResult,FilledID);
    SGM::GetBoundingBox(rResult,RectangleID);

    SGM::Body IDBox=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Complex IDComplexBox=SGM::CreateComplex(rResult,IDBox);
    SGM::FindSharpEdges(rResult,IDComplexBox,0.1);

    std::vector<SGM::Point3D> aPoints1,aPoints2,aPoints3;
    std::vector<SGM::IntersectionType> aTypes1,aTypes2;
    SGM::Point3D Root(5,5,5);
    SGM::UnitVector3D Axis(0,0,1);
    SGM::RayFire(rResult,Root,Axis,IDComplexBox,aPoints1,aTypes1);
    SGM::RayFire(rResult,Root,Axis,IDComplexBox,aPoints2,aTypes2,SGM_MIN_TOL,true);
    EXPECT_EQ(aPoints1.size(),1);
    EXPECT_EQ(aPoints2.size(),2);

    SGM::Segment3D Seg(Root,SGM::Point3D(5,5,15));
    SGM::IntersectSegment(rResult,Seg,IDBox,aPoints3);
    EXPECT_EQ(aPoints3.size(),1);
        
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, cylinder_copy_transform)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(0,0,1);
    SGM::Body CylinderID=SGM::CreateCylinder(rResult,Pos0,Pos1,1.0);
    SGM::Transform3D Trans(SGM::Vector3D(1,2,3));
    SGM::TransformEntity(rResult,Trans,CylinderID);
    SGM::CopyEntity(rResult,CylinderID);

    SGM::Surface SurfID=SGM::CreateCylinderSurface(rResult,Pos0,Pos1,1.0);
    SGM::Point2D uvGuess1(0,0);
    SGM::Point2D uv1=SGM::SurfaceInverse(rResult,SurfID,SGM::Point3D(1,0,0),nullptr,&uvGuess1);
    bool bTest1=SGM::NearEqual(uv1.m_u,0,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest1);
    SGM::Point2D uvGuess2(SGM_TWO_PI,0);
    SGM::Point2D uv2=SGM::SurfaceInverse(rResult,SurfID,SGM::Point3D(1,0,0),nullptr,&uvGuess2);
    bool bTest2=SGM::NearEqual(uv2.m_u,SGM_TWO_PI,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, cone_copy_transform)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(0,0,1);
    SGM::Body ConeID=SGM::CreateCone(rResult,Pos0,Pos1,1.0,2.0);
    SGM::Transform3D Trans(SGM::Vector3D(1,2,3));
    SGM::TransformEntity(rResult,Trans,ConeID);
    SGM::CopyEntity(rResult,ConeID);

    SGM::Surface SurfID=SGM::CreateConeSurface(rResult,Pos0,Pos1-Pos0,1.0,2.0);
    SGM::Point2D uvGuess1(0,0);
    SGM::Point2D uv1=SGM::SurfaceInverse(rResult,SurfID,SGM::Point3D(1,0,0),nullptr,&uvGuess1);
    bool bTest1=SGM::NearEqual(uv1.m_u,0,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest1);
    SGM::Point2D uvGuess2(SGM_TWO_PI,0);
    SGM::Point2D uv2=SGM::SurfaceInverse(rResult,SurfID,SGM::Point3D(1,0,0),nullptr,&uvGuess2);
    bool bTest2=SGM::NearEqual(uv2.m_u,SGM_TWO_PI,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest2);

    SGMTesting::ReleaseTestThing(pThing);
}

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

TEST(math_check, timer)
{
    SGM_TIMER_INITIALIZE();
    SGM_TIMER_START("test");
    SGM_TIMER_STOP();
    SGM_TIMER_SUM();
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

TEST(math_check, triangulate_overlapping_polygon)
    {
    // Tests to see if an overlapping triangle can be triangluated.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point2D> aPoints;
    aPoints.push_back(SGM::Point2D(0,0));
    aPoints.push_back(SGM::Point2D(1,1));
    aPoints.push_back(SGM::Point2D(0,1));
    aPoints.push_back(SGM::Point2D(1,0));
    std::vector<unsigned int> aPolygon;
    aPolygon.push_back(0);
    aPolygon.push_back(1);
    aPolygon.push_back(2);
    aPolygon.push_back(3);
    std::vector<unsigned int> aTriangles;
    bool bAnswer=SGM::TriangulatePolygon(rResult,aPoints,aPolygon,aTriangles);
    EXPECT_EQ(aTriangles.size(),6);
    
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, min_cycles_odd)
    {
    // Tests to find min cycles with an odd number of edges.

    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    
    sVertices.insert(0);
    sVertices.insert(1);
    sVertices.insert(2);
    sVertices.insert(3);
    sVertices.insert(4);
    sVertices.insert(5);
    sVertices.insert(6);
    sVertices.insert(7);
    sVertices.insert(8);

    sEdges.insert(SGM::GraphEdge(0,1,0));
    sEdges.insert(SGM::GraphEdge(0,2,1));
    sEdges.insert(SGM::GraphEdge(3,1,2));
    sEdges.insert(SGM::GraphEdge(2,4,3));
    sEdges.insert(SGM::GraphEdge(3,5,4));
    sEdges.insert(SGM::GraphEdge(4,6,5));
    sEdges.insert(SGM::GraphEdge(5,6,6));
    sEdges.insert(SGM::GraphEdge(5,7,7));
    sEdges.insert(SGM::GraphEdge(6,8,8));
    sEdges.insert(SGM::GraphEdge(7,8,9));

    SGM::Graph graph(sVertices,sEdges);

    SGM::GraphEdge GE(0,1,0);
    SGM::Graph GLoop=graph.FindMinCycle(GE);
    bool bAnswer=true;
    if(GLoop.GetVertices().size()!=7)
        {
        bAnswer=false;
        }
    if(GLoop.GetEdges().size()!=7)
        {
        bAnswer=false;
        }
    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, min_cycles_even)
    {
    // Tests to find min cycles with an even number of edges.

    std::set<size_t> sVertices;
    std::set<SGM::GraphEdge> sEdges;
    
    sVertices.insert(0);
    sVertices.insert(1);
    sVertices.insert(2);
    sVertices.insert(3);
    sVertices.insert(4);
    sVertices.insert(5);
    sVertices.insert(6);

    sEdges.insert(SGM::GraphEdge(0,1,0));
    sEdges.insert(SGM::GraphEdge(0,2,1));
    sEdges.insert(SGM::GraphEdge(3,1,2));
    sEdges.insert(SGM::GraphEdge(2,4,3));
    sEdges.insert(SGM::GraphEdge(3,5,4));
    sEdges.insert(SGM::GraphEdge(4,5,5));
    sEdges.insert(SGM::GraphEdge(4,6,6));
    sEdges.insert(SGM::GraphEdge(6,6,7));

    SGM::Graph graph(sVertices,sEdges);

    SGM::GraphEdge GE(0,1,0);
    SGM::Graph GLoop=graph.FindMinCycle(GE);
    bool bAnswer=true;
    if(GLoop.GetVertices().size()!=6)
        {
        bAnswer=false;
        }
    if(GLoop.GetEdges().size()!=6)
        {
        bAnswer=false;
        }
    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, triangulate_polygon_with_holes)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point2D> aPoints2D;
    aPoints2D.push_back(SGM::Point2D(0,0));
    aPoints2D.push_back(SGM::Point2D(1,0));
    aPoints2D.push_back(SGM::Point2D(0,1));
    aPoints2D.push_back(SGM::Point2D(0.25,0.25));
    std::vector<std::vector<unsigned int> > aaPolygons;
    std::vector<unsigned int> aPolygon1,aPolygon2;
    aPolygon1.push_back(0);
    aPolygon1.push_back(1);
    aPolygon1.push_back(2);
    aPolygon2.push_back(3);
    aaPolygons.push_back(aPolygon1);
    aaPolygons.push_back(aPolygon2);
    std::vector<unsigned int> aTriangles,aAdjacencies;
    SGM::TriangulatePolygonWithHoles(rResult,aPoints2D,aaPolygons,aTriangles,aAdjacencies);
    EXPECT_EQ(aTriangles.size(),9);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, triangulate_polygon)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Testing polygon trianglulation.

    std::vector<SGM::Point2D> aPoints2D;
    aPoints2D.push_back(SGM::Point2D(2 , 0));
    aPoints2D.push_back(SGM::Point2D(1 , 0));
    aPoints2D.push_back(SGM::Point2D(0 ,-1));
    aPoints2D.push_back(SGM::Point2D(-1, 0));
    aPoints2D.push_back(SGM::Point2D(0 , 1));
    aPoints2D.push_back(SGM::Point2D(1 , 0));
    aPoints2D.push_back(SGM::Point2D(2 , 0));
    aPoints2D.push_back(SGM::Point2D(0 , 2));
    aPoints2D.push_back(SGM::Point2D(-2, 0));
    aPoints2D.push_back(SGM::Point2D(0 ,-2));

    std::vector<SGM::Point3D> aPoints3D;
    aPoints3D.push_back(SGM::Point3D(2 , 0, 0));
    aPoints3D.push_back(SGM::Point3D(1 , 0, 0));
    aPoints3D.push_back(SGM::Point3D(0 ,-1, 0));
    aPoints3D.push_back(SGM::Point3D(-1, 0, 0));
    aPoints3D.push_back(SGM::Point3D(0 , 1, 0));
    aPoints3D.push_back(SGM::Point3D(1 , 0, 0));
    aPoints3D.push_back(SGM::Point3D(2 , 0, 0));
    aPoints3D.push_back(SGM::Point3D(0 , 2, 0));
    aPoints3D.push_back(SGM::Point3D(-2, 0, 0));
    aPoints3D.push_back(SGM::Point3D(0 ,-2, 0));

    std::vector<unsigned int> aTriangles,aSegments;
    std::vector<unsigned int> aPolygon={0,1,2,3,4,5,6,7,8,9};
    SGM::TriangulatePolygon(rResult,aPoints2D,aPolygon,aTriangles);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);
 
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, octahedron)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned int> aSegments,aTriangles;
    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D ZAxis(0,0,1),XAxis(1,0,0);
    SGM::CreateOctahedron(1.0,Center,ZAxis,XAxis,aPoints3D,aTriangles,3);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, icosahedron)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned int> aSegments,aTriangles;
    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D ZAxis(0,0,1),XAxis(1,0,0);
    SGM::CreateIcosahedron(1.0,Center,ZAxis,XAxis,aPoints3D,aTriangles,3);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_cration_and_point_removal)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test grid creation and point imprinting.

    std::vector<double> aValues;
    aValues.push_back(0);
    aValues.push_back(1);
    aValues.push_back(2);
    aValues.push_back(3);
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<unsigned int> aTriangles,aSegments;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point3D> aGridPoints3D;
    for(auto Pos : aPoints2D)
        {
        aGridPoints3D.push_back(SGM::Point3D(Pos.m_u,Pos.m_v,0.0));
        }
    
    // Point (1,1,0) is at index 5.
    
    std::vector<unsigned int> aRemovedOrChanged,aReplacedTriangles;
    SGM::RemovePointFromTriangles(rResult,5,aPoints2D,aTriangles,aRemovedOrChanged,aReplacedTriangles);
    SGM::CreateComplex(rResult,aGridPoints3D,aSegments,aTriangles);


    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_cration_and_imprinting_polygon)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aValues;
    aValues.push_back(0);
    aValues.push_back(1);
    aValues.push_back(2);
    aValues.push_back(3);
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<unsigned int> aTriangles,aSegments;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point2D> aPolyPoints2D;
    std::vector<SGM::Point3D> aPolyPoints3D;
    
    aPolyPoints2D.push_back(SGM::Point2D(0,0));
    aPolyPoints3D.push_back(SGM::Point3D(0,0,0));
    aPolyPoints2D.push_back(SGM::Point2D(3,0));
    aPolyPoints3D.push_back(SGM::Point3D(3,0,0));
    aPolyPoints2D.push_back(SGM::Point2D(3,3));
    aPolyPoints3D.push_back(SGM::Point3D(3,3,0));
    aSegments.push_back(0);
    aSegments.push_back(1);
    aSegments.push_back(1);
    aSegments.push_back(2);
    aSegments.push_back(2);
    aSegments.push_back(0);
    
    std::vector<unsigned int> aPolygonIndices;
    SGM::InsertPolygon(rResult,aPolyPoints2D,aPoints2D,aTriangles,aPolygonIndices);
    std::vector<SGM::Point3D> aPoints3D;
    for(auto Pos : aPoints2D)
        {
        aPoints3D.push_back(SGM::Point3D(Pos.m_u,Pos.m_v,0.0));
        }
    aSegments.clear();
    SGM::ReduceToUsedPoints(aPoints2D,aTriangles,&aPoints3D,nullptr);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_cration_and_imprinting_circle)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aValues;
    aValues.push_back(0);
    aValues.push_back(1);
    aValues.push_back(2);
    aValues.push_back(3);
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned int> aTriangles,aSegments;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point3D> aGridPoints3D;
    for(auto Pos : aPoints2D)
        {
        aGridPoints3D.push_back(SGM::Point3D(Pos.m_u,Pos.m_v,0.0));
        }

    std::vector<SGM::Point2D> aPolyPoints2D;
    std::vector<SGM::Point3D> aPolyPoints3D;
    SGM::Interval1D Domain(0,SGM_TWO_PI);
    size_t Index1,nPoints=12;
    double dRadius=1.5;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        double t=Domain.MidPoint(Index1/(double)nPoints);
        double x=cos(t)*dRadius+1.5;
        double y=sin(t)*dRadius+1.5;
        aPolyPoints2D.push_back(SGM::Point2D(x,y));
        aPolyPoints3D.push_back(SGM::Point3D(x,y,0));
        aSegments.push_back((unsigned int)Index1);
        aSegments.push_back((unsigned int)((Index1+1)%nPoints));
        }

    std::vector<unsigned int> aPolygonIndices;
    SGM::InsertPolygon(rResult,aPolyPoints2D,aPoints2D,aTriangles,aPolygonIndices);
    for(auto Pos : aPoints2D)
        {
        aPoints3D.push_back(SGM::Point3D(Pos.m_u,Pos.m_v,0.0));
        }
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, spliting_complex_at_points)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints;
    aPoints.push_back(SGM::Point3D(0,0,0));
    aPoints.push_back(SGM::Point3D(10,0,0));
    aPoints.push_back(SGM::Point3D(10,10,0));
    std::vector<unsigned int> aSegments,aTriangles;
    aSegments.push_back(0);
    aSegments.push_back(1);
    aSegments.push_back(1);
    aSegments.push_back(2);

    std::vector<SGM::Point3D> aNewPoints;
    aNewPoints.push_back(SGM::Point3D(2,0,0));
    aNewPoints.push_back(SGM::Point3D(7,0,0));
    aNewPoints.push_back(SGM::Point3D(10,5,0));

    SGM::Complex ComplexID=SGM::CreateComplex(rResult,aPoints,aSegments,aTriangles);
    SGM::SplitComplexAtPoints(rResult,ComplexID,aNewPoints,SGM_MIN_TOL);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, transform_block)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
    SGM::CreateBlock(rResult,Pos0,Pos1);
    SGM::Body BodyID2=SGM::CreateBlock(rResult,Pos0,Pos1);
    SGM::Transform3D Trans(SGM::Vector3D(20,0,0));
    SGM::TransformEntity(rResult,Trans,BodyID2);
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, intersect_line_and_revolve)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    std::vector<SGM::Point3D> aNUBPoints;
    aNUBPoints.emplace_back(-2,.5,0);
    aNUBPoints.emplace_back(-1,1.5,0);
    aNUBPoints.emplace_back(0,1,0);
    aNUBPoints.emplace_back(1,1.5,0);
    aNUBPoints.emplace_back(2,2,0);
    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aNUBPoints);

    SGM::Point3D AxisOrigin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Surface RevolveID = SGM::CreateRevolveSurface(rResult, AxisOrigin, Axis, CurveID);

    SGM::Point3D LineOrigin1(1,0,0);
    SGM::UnitVector3D uDirection1(0,0,1);
    SGM::Curve Line1ID = SGM::CreateLine(rResult, LineOrigin1, uDirection1);

    std::vector<SGM::Point3D> aPoints1;
    std::vector<SGM::IntersectionType> aTypes1;
    double dTolerance = SGM_MIN_TOL;
    SGM::IntersectCurveAndSurface(rResult, Line1ID, RevolveID, aPoints1, aTypes1, nullptr, nullptr, dTolerance);

    if (aPoints1.size() != 2)
        {
        bAnswer=false;
        }

    for (SGM::IntersectionType IType : aTypes1 )
        {
        if (IType != SGM::PointType)
            {
            bAnswer=false;
            }
        }

    std::vector<SGM::Point3D> aExpected1(2);
    aExpected1[0] = SGM::Point3D(1,0,1.5);
    aExpected1[1] = SGM::Point3D(1,0,-1.5);
    int found1=0;
    for (SGM::Point3D PosExpected : aExpected1)
        {
        for (SGM::Point3D PosComputed : aPoints1)
            if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                found1++;
        }

    SGM::Point3D LineOrigin2(1,1,-1.2);
    SGM::UnitVector3D uDirection2(0,-1,0);
    SGM::Curve Line2ID = SGM::CreateLine(rResult, LineOrigin2, uDirection2);

    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    SGM::IntersectCurveAndSurface(rResult, Line2ID, RevolveID, aPoints2, aTypes2, nullptr, nullptr, dTolerance);

    if (aPoints2.size() != 2)
        {
        bAnswer=false;
        }

    for (SGM::IntersectionType IType : aTypes2 )
        {
        if (IType != SGM::PointType)
            {
            bAnswer=false;
            }
        }

    std::vector<SGM::Point3D> aExpected2(2);
    aExpected2[0] = SGM::Point3D(1,0.9,-1.2);
    aExpected2[1] = SGM::Point3D(1,-0.9,-1.2);

    int found2=0;
    for (SGM::Point3D PosExpected : aExpected2)
        {
        for (SGM::Point3D PosComputed : aPoints2)
            if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                found2++;
        }

    bAnswer = ((found1 == 2) && (found2 == 2));
    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, intersect_nubcurve_and_plane)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints1;
    aPoints1.emplace_back(-2,.5,0);
    aPoints1.emplace_back(-1,1.5,0);
    aPoints1.emplace_back(0,1,0);
    aPoints1.emplace_back(1,1.5,0);
    aPoints1.emplace_back(2,2,0);

    // simple case
    //aPoints1.emplace_back(-2,.5,0);
    //aPoints1.emplace_back(-1,1.5,0);
    //aPoints1.emplace_back(0,.5,0);
    //aPoints1.emplace_back(1,.5,0);
    //aPoints1.emplace_back(2,.5,0);

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Origin(1,0,0);
    SGM::Point3D XPos(1,0,-1);
    SGM::Point3D YPos(1,1,0);

    SGM::Surface PlaneID = SGM::CreatePlane(rResult, Origin, XPos, YPos);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    double dTolerance = SGM_MIN_TOL;
    SGM::IntersectCurveAndSurface(rResult, CurveID, PlaneID, aPoints, aTypes, nullptr, nullptr, dTolerance);

    if (aPoints.size() != 1)
        {
        bAnswer=false;
        }
    else
        {
        SGM::Point3D Expected(1, 1.5, 0);
        bAnswer = SGM::NearEqual(aPoints[0], Expected, dTolerance);
        }

    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, DISABLED_body_volumes)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    SGM::Point3D Pos0(0,0,0),Pos1(10,10,10);
    SGM::Body BodyID1=SGM::CreateBlock(rResult,Pos0,Pos1);
    double dVolume1=SGM::FindVolume(rResult,BodyID1,true);
    if(SGM::NearEqual(dVolume1,1000,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    
    SGM::Point3D Pos3(0,0,0),Pos4(0,0,1);
    SGM::Body BodyID2=SGM::CreateCylinder(rResult,Pos3,Pos4,1.0);
    double dVolume2=SGM::FindVolume(rResult,BodyID2,false);
    if(SGM::NearEqual(dVolume2,3.1415926535897932384626433832795,SGM_MIN_TOL,true)==false)
        {
        bAnswer=false;
        }
    
    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, DISABLED_point_in_body)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Point in Body

    bool bAnswer=true;

    SGM::Point3D Bottom(0,0,0),Top(0,0,2);
    double dRadius=1.0;
    SGM::Body BodyID=SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    SGM::Point3D Pos1(-2,0,1);
    bool bInBody1=SGM::PointInEntity(rResult,Pos1,BodyID);
    SGM::Point3D Pos2(0,0,1);
    bool bInBody2=SGM::PointInEntity(rResult,Pos2,BodyID);

    if(bInBody1==false)
        {
        bAnswer=false;
        }
    if(bInBody2==true)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, ray_fire)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    SGM::Point3D Bottom(0,0,0),Top(0,0,2);
    double dRadius=1.0;
    SGM::Body BodyID=SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    SGM::Point3D Origin(-2,0,1);
    SGM::UnitVector3D Axis(1,0,0);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::RayFire(rResult,Origin,Axis,BodyID,aPoints,aTypes,SGM_MIN_TOL);

    if(aPoints.size()==2)
        {
        if(SGM::NearEqual(aPoints[0],SGM::Point3D(-1,0,1),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(SGM::NearEqual(aPoints[1],SGM::Point3D(1,0,1),SGM_ZERO)==false)
            {
            bAnswer=false;
            }
        if(aTypes[0]!=SGM::IntersectionType::PointType)
            {
            bAnswer=false;
            }
        if(aTypes[1]!=SGM::IntersectionType::PointType)
            {
            bAnswer=false;
            }
        }
    else
        {
        bAnswer=false;
        }
    
    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NURB_surface)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test of NURB Surface.

    std::vector<std::vector<SGM::Point4D> > aaControlPoints;
    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,1,0,1);
    aControlPoints.emplace_back(-1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(-1,0,0,1);
    aControlPoints.emplace_back(-1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,-1,0,1);
    aControlPoints.emplace_back(1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,0,1);
    aaControlPoints.push_back(aControlPoints);
    aControlPoints.clear();
    aControlPoints.emplace_back(1,0,1,1);
    aControlPoints.emplace_back(1,1,1,sqrt(2)/2);
    aControlPoints.emplace_back(0,1,1,1);
    aControlPoints.emplace_back(-1,1,1,sqrt(2)/2);
    aControlPoints.emplace_back(-1,0,1,1);
    aControlPoints.emplace_back(-1,-1,1,sqrt(2)/2);
    aControlPoints.emplace_back(0,-1,1,1);
    aControlPoints.emplace_back(1,-1,1,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,1,1);
    aaControlPoints.push_back(aControlPoints);
    
    std::vector<double> aUKnots;
    aUKnots.push_back(0);
    aUKnots.push_back(0);
    aUKnots.push_back(1);
    aUKnots.push_back(1);

    std::vector<double> aVKnots;
    aVKnots.push_back(0);
    aVKnots.push_back(0);
    aVKnots.push_back(0);
    aVKnots.push_back(SGM_HALF_PI);
    aVKnots.push_back(SGM_HALF_PI);
    aVKnots.push_back(SGM_PI);
    aVKnots.push_back(SGM_PI);
    aVKnots.push_back(SGM_PI*1.5);
    aVKnots.push_back(SGM_PI*1.5);
    aVKnots.push_back(SGM_TWO_PI);
    aVKnots.push_back(SGM_TWO_PI);
    aVKnots.push_back(SGM_TWO_PI);

    SGM::Surface SurfID=SGM::CreateNURBSurface(rResult,std::move(aaControlPoints),std::move(aUKnots),std::move(aVKnots));
    bool bAnswer=SGM::TestSurface(rResult,SurfID,SGM::Point2D(0.245,0.678));

    SGM::Point3D Pos0,Pos1,Pos2;
    SGM::EvaluateSurface(rResult,SurfID,SGM::Point2D(0.145,0.578),&Pos0);
    SGM::EvaluateSurface(rResult,SurfID,SGM::Point2D(0.245,0.678),&Pos1);
    SGM::EvaluateSurface(rResult,SurfID,SGM::Point2D(0.345,0.778),&Pos2);
    Pos0.m_z=0;
    Pos1.m_z=0;
    Pos2.m_z=0;
    double dDist0=(Pos0-SGM::Point3D(0,0,0)).Magnitude();
    double dDist1=(Pos1-SGM::Point3D(0,0,0)).Magnitude();
    double dDist2=(Pos2-SGM::Point3D(0,0,0)).Magnitude();

    if(SGM::NearEqual(dDist0,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(dDist1,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(dDist2,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NURB_curve)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test of NURB Curves.

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,1,0,1);
    aControlPoints.emplace_back(-1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(-1,0,0,1);
    aControlPoints.emplace_back(-1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,-1,0,1);
    aControlPoints.emplace_back(1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,0,1);
    
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

    SGM::Curve CurveID=SGM::CreateNURBCurve(rResult,aControlPoints,aKnots);
    bool bAnswer=SGM::TestCurve(rResult,CurveID,1.234);

    SGM::Point3D Pos0,Pos1,Pos2;
    SGM::EvaluateCurve(rResult,CurveID,1,&Pos0);
    SGM::EvaluateCurve(rResult,CurveID,2,&Pos1);
    SGM::EvaluateCurve(rResult,CurveID,3,&Pos2);
    double dDist0=(Pos0-SGM::Point3D(0,0,0)).Magnitude();
    double dDist1=(Pos1-SGM::Point3D(0,0,0)).Magnitude();
    double dDist2=(Pos2-SGM::Point3D(0,0,0)).Magnitude();

    if(SGM::NearEqual(dDist0,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(dDist1,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(dDist2,1.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, torus_area)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(0,0,1);
    SGM::Body BodyID=SGM::CreateTorus(rResult,Center,Norm,1,3);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    double dArea=SGM::FindArea(rResult,FaceID);
    if(SGM::NearEqual(dArea,118.43525281307230342601389199851,SGM_MIN_TOL,false)==false)
        {
        bAnswer=false;
        }

    SGM::DeleteEntity(rResult,BodyID);

    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, sphere_area)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(1,1,1);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Norm,1.0);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Surface SurfaceID=SGM::CreateSphereSurface(rResult,Center,1.0);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(EdgeID);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
    SGM::Body BodyID=SGM::CreateSheetBody(rResult,SurfaceID,aEdges,aTypes);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    double dArea=SGM::FindArea(rResult,FaceID);
    if(SGM::NearEqual(dArea,6.283185307179586476925286766559,SGM_MIN_TOL,true)==false)
        {
        bAnswer=false;
        }
    SGM::DeleteEntity(rResult,BodyID);

    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
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
            SGM::Interval1D const &Domain=SGM::GetCurveDomain(rResult,CurveID);
            double dLength=SGM::FindCurveLength(rResult,Domain,CurveID);
            double dTol=SGM_FIT*dLength;
            for(Index2=1;Index2<nTestPoint;++Index2)
                {
                double dFraction=Index2/(nTestPoint-1.0);
                double t=Domain.MidPoint(dFraction);
                SGM::Point3D Pos;
                SGM::EvaluateCurve(rResult,CurveID,t,&Pos);
                SGM::Point3D CPos1,CPos2;
                SGM::SurfaceInverse(rResult,Surface1,Pos,&CPos1);
                SGM::SurfaceInverse(rResult,Surface1,Pos,&CPos2);
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

TEST(math_check, cylinder_sphere_intersect)
    {
    // Test sphere cylinder intersections

    bool bAnswer=true;

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(0,0,-10),Top(0,0,10),Pos0(1,0,0),Pos1(3,0,0),Pos2(0,0,0),Pos3(2,0,0),Pos4(4,0,0);
    double dRadius=2.0;

    rResult.SetLog(true);
    SGM::Body CylinderID=SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    std::vector<SGM::Entity> const &aLog=rResult.GetLogEntities();
    SGM::Face CylinderFace(aLog[0].m_ID);
    SGM::Surface CylinderSurf=SGM::GetSurfaceOfFace(rResult,CylinderFace);

    SGM::Body SphereID1=SGM::CreateSphere(rResult,Pos2,1.0);
    SGM::Face SphereFace1(aLog[3].m_ID);
    SGM::Surface SphereSurf1=SGM::GetSurfaceOfFace(rResult,SphereFace1);

    SGM::Body SphereID2=SGM::CreateSphere(rResult,Pos2,2.0);
    SGM::Face SphereFace2(aLog[4].m_ID);
    SGM::Surface SphereSurf2=SGM::GetSurfaceOfFace(rResult,SphereFace2);

    SGM::Body SphereID3=SGM::CreateSphere(rResult,Pos2,3.0);
    SGM::Face SphereFace3(aLog[5].m_ID);
    SGM::Surface SphereSurf3=SGM::GetSurfaceOfFace(rResult,SphereFace3);

    SGM::Body SphereID4=SGM::CreateSphere(rResult,Pos0,1.0);
    SGM::Face SphereFace4(aLog[6].m_ID);
    SGM::Surface SphereSurf4=SGM::GetSurfaceOfFace(rResult,SphereFace4);

    SGM::Body SphereID5=SGM::CreateSphere(rResult,Pos1,1.0);
    SGM::Face SphereFace5(aLog[7].m_ID);
    SGM::Surface SphereSurf5=SGM::GetSurfaceOfFace(rResult,SphereFace5);

    SGM::Body SphereID6=SGM::CreateSphere(rResult,Pos3,2.0);
    SGM::Face SphereFace6(aLog[8].m_ID);
    SGM::Surface SphereSurf6=SGM::GetSurfaceOfFace(rResult,SphereFace6);

    SGM::Body SphereID7=SGM::CreateSphere(rResult,Pos3,4.0);
    SGM::Face SphereFace7(aLog[9].m_ID);
    SGM::Surface SphereSurf7=SGM::GetSurfaceOfFace(rResult,SphereFace7);

    SGM::Body SphereID8=SGM::CreateSphere(rResult,Pos3,6.0);
    SGM::Face SphereFace8(aLog[10].m_ID);
    SGM::Surface SphereSurf8=SGM::GetSurfaceOfFace(rResult,SphereFace8);

    SGM::Body SphereID9=SGM::CreateSphere(rResult,Pos4,1.0);
    SGM::Face SphereFace9(aLog[11].m_ID);
    SGM::Surface SphereSurf9=SGM::GetSurfaceOfFace(rResult,SphereFace9);

    if(!TestIntersections(rResult,CylinderSurf,SphereSurf1,0))    // Empty Inside
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf2,1))    // One Circle
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf3,2))    // Two Circles
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf4,1))    // Inside Point
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf5,1))    // Outside Point
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf6,1))    // Potato chip curve
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf7,2))    // Figure eight
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf8,2))    // Two Hermites
        {
        bAnswer=false;
        }
    if(!TestIntersections(rResult,CylinderSurf,SphereSurf9,0))    // Empty Outside
        {
        bAnswer=false;
        }
    
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, line_nub_curve_intersect)
    {
    bool bAnswer=true;

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints;
    size_t Index1;
    aPoints.emplace_back(SGM::Point3D(0,0,0));
    aPoints.emplace_back(SGM::Point3D(1,1,0));
    aPoints.emplace_back(SGM::Point3D(2,0,0));
    SGM::Curve NUBCurveID=SGM::CreateNUBCurve(rResult,aPoints);

    // Test with two hits.

    SGM::Point3D Pos0(0,0.5,0),Pos1(2,0.5,0);
    SGM::Curve LineID1=SGM::CreateLine(rResult,Pos0,Pos1-Pos0);
    std::vector<SGM::Point3D> aHits1;
    std::vector<SGM::IntersectionType> aTypes1;
    SGM::IntersectCurves(rResult,LineID1,NUBCurveID,aHits1,aTypes1);

    size_t nHits1=aHits1.size();
    if(nHits1!=2)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits1;++Index1)
        {
        SGM::Point3D const &Pos=aHits1[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID1,Pos,&CPos1);
        SGM::CurveInverse(rResult,NUBCurveID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID1);

    // Test with one tangent hit.

    SGM::Point3D Pos2(0,1,0),Pos3(2,1,0);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);
    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    SGM::IntersectCurves(rResult,LineID2,NUBCurveID,aHits2,aTypes2);

    size_t nHits2=aHits2.size();
    if(nHits2!=1)
        {
        bAnswer=false;
        }
    else if(aTypes2[0]!=SGM::IntersectionType::TangentType)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits2;++Index1)
        {
        SGM::Point3D const &Pos=aHits2[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::CurveInverse(rResult,NUBCurveID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID2);
    SGM::DeleteEntity(rResult,NUBCurveID);

    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }


TEST(math_check, DISABLED_line_nub_surface_intersect)
    {
    bool bAnswer=true;

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots,aVKnots;
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aVKnots=aUKnots;
    std::vector<std::vector<SGM::Point3D> > aaPoints;
    std::vector<SGM::Point3D> aPoints;
    aPoints.assign(3,SGM::Point3D(0,0,0));
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints[0][0]=SGM::Point3D(0.0,0.0,1.0);
    aaPoints[0][1]=SGM::Point3D(0.0,1.0,0.0);
    aaPoints[0][2]=SGM::Point3D(0.0,2.0,-1.0);
    aaPoints[1][0]=SGM::Point3D(1.0,0.0,0.0);
    aaPoints[1][1]=SGM::Point3D(1.0,1.0,0.0);
    aaPoints[1][2]=SGM::Point3D(1.0,2.0,0.0);
    aaPoints[2][0]=SGM::Point3D(2.0,0.0,-1.0);
    aaPoints[2][1]=SGM::Point3D(2.0,1.0,0.0);
    aaPoints[2][2]=SGM::Point3D(2.0,2.0,1.0);
    SGM::Surface NUBSurfaceID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    // Test with a line that hits the saddle point.

    SGM::Point3D Pos0(0,0,0.0),Pos1(2,2,0.0);
    SGM::Curve LineID1=SGM::CreateLine(rResult,Pos0,Pos1-Pos0);

    std::vector<SGM::Point3D> aHits1;
    std::vector<SGM::IntersectionType> aTypes1;
    size_t nHits1=SGM::IntersectCurveAndSurface(rResult,LineID1,NUBSurfaceID,aHits1,aTypes1);

    if(nHits1!=1)
        {
        bAnswer=false;
        }
    else if(aTypes1[0]!=SGM::IntersectionType::TangentType)
        {
        bAnswer=false;
        }
    size_t Index1;
    for(Index1=0;Index1<nHits1;++Index1)
        {
        SGM::Point3D const &Pos=aHits1[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID1,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID1);

    // Test with a line that hits two points.

    SGM::Point3D Pos2(0,0,0.5),Pos3(2,2,0.5);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);

    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    size_t nHits2=SGM::IntersectCurveAndSurface(rResult,LineID2,NUBSurfaceID,aHits2,aTypes2);

    if(nHits2!=2)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits2;++Index1)
        {
        SGM::Point3D const &Pos=aHits2[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID2);;

    // Test with a line that just misses the saddle but within tolernace.

    SGM::Point3D Pos4(2,0,0.0001),Pos5(0,2,0.0001);
    SGM::Curve LineID3=SGM::CreateLine(rResult,Pos4,Pos5-Pos4);

    std::vector<SGM::Point3D> aHits3;
    std::vector<SGM::IntersectionType> aTypes3;
    double dTestTol=0.001;
    size_t nHits3=SGM::IntersectCurveAndSurface(rResult,LineID3,NUBSurfaceID,aHits3,aTypes3,nullptr,nullptr,dTestTol);

    if(nHits3!=1)
        {
        bAnswer=false;
        }
    else if(aTypes3[0]!=SGM::IntersectionType::TangentType)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits3;++Index1)
        {
        SGM::Point3D const &Pos=aHits3[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID3,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(dTestTol<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID3);
    SGM::DeleteEntity(rResult,NUBSurfaceID);
    
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }


TEST(math_check, cover_stl)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    std::string file_path = get_models_file_path("STL Files/kelvin.stl");
    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<SGM::Complex> *aComplexes = (std::vector<SGM::Complex> *) &entities;
    SGM::Complex ComplexID = aComplexes->front();
    if (aComplexes->size() > 1)
        {
        ComplexID = SGM::MergeComplexes(rResult, *aComplexes);
        }
    SGM::Complex CoverID = SGM::CoverComplex(rResult, ComplexID);
    std::vector<SGM::Complex> aParts;
    aParts.push_back(ComplexID);
    aParts.push_back(CoverID);
    SGM::Complex AnswerID=SGM::MergeComplexes(rResult, aParts);
    SGM::DeleteEntity(rResult,CoverID); 
    SGM::DeleteEntity(rResult,ComplexID); 

    std::string OutputSTLFile = get_models_file_path("STL Files/kelvin_output.stl");
    SGM::TranslatorOptions Options;
    SGM::SaveSTL(rResult, OutputSTLFile, AnswerID, Options);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, find_holes_stl) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    std::string file_path = get_models_file_path("STL Files/SNL-2024-T3-IMP1.stl");
    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<SGM::Complex> *aComplexes = (std::vector<SGM::Complex> *) &entities;
    SGM::Complex ComplexID = aComplexes->front();
    if (aComplexes->size() > 1)
        {
        ComplexID = SGM::MergeComplexes(rResult, *aComplexes);
        }
    std::vector<SGM::Complex> aHoles;
    SGM::Complex HolesID=SGM::FindHoles(rResult,ComplexID,aHoles);
    SGM::DeleteEntity(rResult,ComplexID); 

    std::string OutputSGMLFile("STL Files/SNL-2024-T3-IMP1_Output.stl");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, OutputSGMLFile, SGM::Thing() , Options);    

    SGMTesting::ReleaseTestThing(pThing);
}
/*
TEST(math_check, unite_spheres) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center1(0,0,0),Center2(1.5,0,0);
    SGM::Body SphereID1=SGM::CreateSphere(rResult,Center1,1.0);
    SGM::Body SphereID2=SGM::CreateSphere(rResult,Center2,1.0);
    SGM::UniteBodies(rResult,SphereID1,SphereID2);

    SGMTesting::ReleaseTestThing(pThing);
}
*/
TEST(math_check, NUB_curve_inverse) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    aPoints.emplace_back(-2,0,0);
    aPoints.emplace_back(-1,0.1,0);
    aPoints.emplace_back(0.0,0.0,0);
    aPoints.emplace_back(1,0.1,0);
    aPoints.emplace_back(2,0,0);

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    size_t Index1;
    for(Index1=0;Index1<5;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }

    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, banded_matrix_solve) 
    {
    bool bAnswer=true;

    // if x=1,y=2,z=3,w=4,v=5,r=6 then
    //
    // 1x+2y+3z+0w+0v+0r=14
    // 2x+1y+1z+1w+0v+0r=11
    // 1x+2y+1z+1w+0v+0r=12
    // 0x+1y+2z-1w+1v+0r=9
    // 0x+0y-1z+1w+2v+1r=17
    // 0x+0y+0z-1w-1v+2r=3

    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(5);
    std::vector<double> aRow;
    aRow.reserve(6);
    aRow.push_back(0);
    aRow.push_back(0);
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(3);
    aRow.push_back(14);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(0);
    aRow.push_back(2);
    aRow.push_back(1);
    aRow.push_back(1);
    aRow.push_back(1);
    aRow.push_back(11);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(1);
    aRow.push_back(1);
    aRow.push_back(0);
    aRow.push_back(12);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(-1);
    aRow.push_back(1);
    aRow.push_back(0);
    aRow.push_back(9);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(-1);
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(1);
    aRow.push_back(0);
    aRow.push_back(17);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(-1);
    aRow.push_back(-1);
    aRow.push_back(2);
    aRow.push_back(0);
    aRow.push_back(0);
    aRow.push_back(3);
    aaMatrix.push_back(aRow);

    if(SGM::BandedSolve(aaMatrix)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[0].back(),1,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[1].back(),2,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[2].back(),3,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[3].back(),4,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[4].back(),5,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);
    }

TEST(math_check, matrix_solve) 
    {
    bool bAnswer=true;

    // if x=1,y=2,z=3,w=4, then
    //
    // 1x+2y+0z+1w= 9
    // 2x+2y+2z+0w=12
    // 0x+2y-1z+3w=13
    // 1x+1y+2z-1w= 5

    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(4);
    std::vector<double> aRow;
    aRow.reserve(4);
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(0);
    aRow.push_back(1);
    aRow.push_back(9);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(2);
    aRow.push_back(2);
    aRow.push_back(2);
    aRow.push_back(0);
    aRow.push_back(12);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(0);
    aRow.push_back(2);
    aRow.push_back(-1);
    aRow.push_back(3);
    aRow.push_back(13);
    aaMatrix.push_back(aRow);
    aRow.clear();
    aRow.push_back(1);
    aRow.push_back(1);
    aRow.push_back(2);
    aRow.push_back(-1);
    aRow.push_back(5);
    aaMatrix.push_back(aRow);

    if(SGM::LinearSolve(aaMatrix)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[0].back(),1,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[1].back(),2,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[2].back(),3,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(aaMatrix[3].back(),4,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer); 
    }

TEST(math_check, NUB_curve_through_three_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to three points with end vectors.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(3);
    aPoints.emplace_back(1,1,0);
    aPoints.emplace_back(2,2,0);
    aPoints.emplace_back(3,1,0);

    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    size_t Index1;
    for(Index1=0;Index1<3;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer);  

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_points_and_end_vecs)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to four points with end vectors.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(4);
    aPoints.emplace_back(1,1,0);
    aPoints.emplace_back(2,2,0);
    aPoints.emplace_back(3,1,0);
    aPoints.emplace_back(5,0,0);

    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    size_t Index1;
    for(Index1=0;Index1<4;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_two_points_and_end_vecs) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    // Fitting a NUB curve to two points with end vectors.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(2);
    aPoints.emplace_back(1,1,0);
    aPoints.emplace_back(3,1,0);

    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    size_t Index1;
    for(Index1=0;Index1<2;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    if(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }

TEST(math_check, NUB_curve_to_three_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to three points.
    // Which requires it be degree two.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(3);
    aPoints.emplace_back(1,1,0);
    aPoints.emplace_back(2,2,0);
    aPoints.emplace_back(3,1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    size_t Index1;
    for(Index1=0;Index1<3;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }
    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }

TEST(math_check, NUB_curve_to_two_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to two points.
    // Which requires it be degree one.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(2);
    aPoints.emplace_back(1,1,0);
    aPoints.emplace_back(3,1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    size_t Index1;
    for(Index1=0;Index1<2;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        if(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL)==false)
            {
            bAnswer=false;
            }
        }
    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }

TEST(math_check, create_nub_surface) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test NUB surface.

    std::vector<double> aUKnots,aVKnots;
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aVKnots=aUKnots;
    std::vector<std::vector<SGM::Point3D> > aaPoints;
    std::vector<SGM::Point3D> aPoints;
    aPoints.assign(3,SGM::Point3D(0,0,0));
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints[0][0]=SGM::Point3D(0.0,0.0,1.0);
    aaPoints[0][1]=SGM::Point3D(0.0,1.0,0.0);
    aaPoints[0][2]=SGM::Point3D(0.0,2.0,-1.0);
    aaPoints[1][0]=SGM::Point3D(1.0,0.0,0.0);
    aaPoints[1][1]=SGM::Point3D(1.0,1.0,0.0);
    aaPoints[1][2]=SGM::Point3D(1.0,2.0,0.0);
    aaPoints[2][0]=SGM::Point3D(2.0,0.0,-1.0);
    aaPoints[2][1]=SGM::Point3D(2.0,1.0,0.0);
    aaPoints[2][2]=SGM::Point3D(2.0,2.0,1.0);
    SGM::Surface NUBSurfID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    bool bAnswer=SGM::TestSurface(rResult,NUBSurfID,SGM::Point2D(0.3,0.2));
    
    SGM::UnitVector3D Vec1,Vec2;
    double k1,k2;
    SGM::Point2D uv(0.5,0.5);
    SGM::PrincipleCurvature(rResult,NUBSurfID,uv,Vec1,Vec2,k1,k2);

    if(SGM::NearEqual(Vec1,SGM::UnitVector3D(1.0,-1.0,0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec2,SGM::UnitVector3D(1.0,1.0,0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(k1,-4.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(k2,4.0,SGM_ZERO,false)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }


TEST(math_check, line_torus_intersection) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Line Torus Intersections.

    bool bAnswer=true;

    SGM::Point3D Origin(-20,0,0);
    SGM::UnitVector3D Axis(1.0,0.0,0.0);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;

    SGM::Point3D Center(0.0,0.0,0.0);
    SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,Center,ZAxis,1.0,3.0,false);
    
    SGM::Curve Line1=SGM::CreateLine(rResult,Origin,Axis);
    size_t nHits=SGM::IntersectCurveAndSurface(rResult,Line1,TorusID,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
    if( nHits!=4 ||
        SGM::NearEqual(aPoints[0],SGM::Point3D(-4.0,0.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[1],SGM::Point3D(-2.0,0.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[2],SGM::Point3D( 2.0,0.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[3],SGM::Point3D( 4.0,0.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=2.0;
    SGM::Curve Line2=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line2,TorusID,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
    if( nHits!=3 ||
        SGM::NearEqual(aPoints[0],SGM::Point3D(-3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[1],SGM::Point3D(0.0,2.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[2],SGM::Point3D(3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=3.0;
    SGM::Curve Line3=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line3,TorusID,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
    if( nHits!=2 ||
        SGM::NearEqual(aPoints[0],SGM::Point3D(-2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO)==false ||
        SGM::NearEqual(aPoints[1],SGM::Point3D(2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=4.0;
    SGM::Curve Line4=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line4,TorusID,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
    if( nHits!=1 ||
        SGM::NearEqual(aPoints[0],SGM::Point3D(0.0,4.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=5.0;
    SGM::Curve Line5=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line5,TorusID,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
    if(nHits!=0)
        {
        bAnswer=false;
        }

    SGM::DeleteEntity(rResult,TorusID);
    SGM::DeleteEntity(rResult,Line1);
    SGM::DeleteEntity(rResult,Line2);
    SGM::DeleteEntity(rResult,Line3);
    SGM::DeleteEntity(rResult,Line4);
    SGM::DeleteEntity(rResult,Line5);

    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }


TEST(math_check, test_transforms) 
    {
    // Test Transforms

    bool bAnswer=true;

    SGM::UnitVector3D XAxis(2.0,3.0,4.0);
    SGM::UnitVector3D YAxis=XAxis.Orthogonal();
    SGM::UnitVector3D ZAxis=XAxis*YAxis;
    SGM::Vector3D Offset(5.0,6.0,7.0);
    SGM::Transform3D Trans1(XAxis,YAxis,ZAxis,Offset);
    SGM::Transform3D Trans2;
    Trans1.Inverse(Trans2);
    SGM::Transform3D Trans3=Trans1*Trans2;

    SGM::Point3D Pos(0.0,0.0,0.0);
    SGM::Vector3D Vec(1.0,0.0,0.0);
    SGM::UnitVector3D UVec(1.0,0.0,0.0);
    SGM::Point3D Pos1=Trans1*Pos;
    SGM::Point3D Pos2=Trans2*Pos1;
    SGM::Point3D Pos3=Trans3*Pos;
    SGM::Vector3D Vec1=Trans1*Vec;
    SGM::Vector3D Vec2=Trans2*Vec1;
    SGM::Vector3D Vec3=Trans3*Vec;
    SGM::UnitVector3D UVec1=Trans1*UVec;
    SGM::UnitVector3D UVec2=Trans2*UVec1;
    SGM::UnitVector3D UVec3=Trans3*UVec;

    if(SGM::NearEqual(Pos,Pos2,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec,Vec2,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(UVec,UVec2,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Pos,Pos3,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Vec,Vec3,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(UVec,UVec3,SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    
    EXPECT_TRUE(bAnswer); 
    }


TEST(math_check, DISABLED_least_squares_plane) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Least Squared Plane Fitting.

    bool bAnswer=true;

    std::vector<SGM::Point3D> aPoints;
    aPoints.emplace_back(0.0,0.0,0.0);
    aPoints.emplace_back(2.0,0.0,0.0);
    aPoints.emplace_back(2.0,1.0,0.0);
    aPoints.emplace_back(0.0,1.0,0.0);
    aPoints.emplace_back(1.0,0.5,0.4);

    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    if(SGM::NearEqual(Origin,SGM::Point3D(1.0,0.5,0.08),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aPoints.emplace_back(1.0,0.0,0.0);
    aPoints.emplace_back(0.0,1.0,0.0);
    aPoints.emplace_back(3.0,2.0,0.0);
    aPoints.emplace_back(2.0,3.0,0.0);

    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    if(SGM::NearEqual(Origin,SGM::Point3D(1.5,1.5,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(YVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,-1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aPoints.emplace_back(1.0,0.0,0.0);
    aPoints.emplace_back(0.0,1.0,0.0);
    aPoints.emplace_back(1.0,1.0,0.0);
    aPoints.emplace_back(0.0,0.0,0.0);
    aPoints.emplace_back(1.0,0.0,1.0);
    aPoints.emplace_back(0.0,1.0,1.0);
    aPoints.emplace_back(1.0,1.0,1.0);
    aPoints.emplace_back(0.0,0.0,1.0);

    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    if(SGM::NearEqual(Origin,SGM::Point3D(0.5,0.5,0.5),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aPoints.emplace_back(0.0,0.0,0.0);
    aPoints.emplace_back(8.0,8.0,0.0);
    aPoints.emplace_back(4.0,4.0,0.0);

    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    if(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(YVec,SGM::UnitVector3D(-1.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    aPoints.clear();
    aPoints.emplace_back(0.0,0.0,0.0);
    aPoints.emplace_back(8.0,8.0,0.0);
    aPoints.emplace_back(4.0,4.0,0.1);

    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    if(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0333333333333333),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(ZVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO)==false)
        {
        bAnswer=false;
        }

    EXPECT_TRUE(bAnswer); 
    SGMTesting::ReleaseTestThing(pThing); 
    }
/*
TEST(math_check, DISABLED_unite_squares_island) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos1(0,0,0),Pos2(10,10,0),Pos3(2,5,-4),Pos4(8,5,4);
    SGM::Body SheetID1=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::Body SheetID2=SGM::CreateBlock(rResult,Pos3,Pos4);
    SGM::UniteBodies(rResult,SheetID1,SheetID2);

    SGMTesting::ReleaseTestThing(pThing);
}
*/
TEST(math_check, sgm_save_and_read_block) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::ChangeColor(rResult,BlockID,255,0,0);
    int nRed,nGreen,nBlue;
    SGM::GetColor(rResult,BlockID,nRed,nGreen,nBlue);

    EXPECT_EQ(nRed,255);
    EXPECT_EQ(nGreen,0);
    EXPECT_EQ(nBlue,0);

    SGM::RemoveColor(rResult,BlockID);

    EXPECT_FALSE(SGM::GetColor(rResult,BlockID,nRed,nGreen,nBlue));

    SGM::ChangeColor(rResult,BlockID,255,0,0);

    std::string file_path=get_models_file_path("block.sgm");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,BlockID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, step_save_and_read_block) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    std::string file_path=get_models_file_path("block.step");
    SGM::TranslatorOptions Options;
    SGM::SaveSTEP(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,BlockID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, delete_face_from_block) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::DeleteEntity(rResult,FaceID);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, copy_a_complex_and_block) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BlockID);
    SGM::CopyEntity(rResult,ComplexID);
    SGM::CopyEntity(rResult,BlockID);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, transform_complex) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BlockID);
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,ComplexID);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, test_complex_props) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BlockID);

    double dMaxLength;
    double dAverageLength=SGM::FindAverageEdgeLength(rResult,ComplexID,&dMaxLength);

    EXPECT_TRUE(SGM::NearEqual(dMaxLength,14.142135623730950488016887242097,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(dAverageLength,11.380711874576983,SGM_ZERO,false));

    size_t nGenus=SGM::FindGenus(rResult,ComplexID);

    EXPECT_EQ(nGenus,0);
    
    bool bLinear=SGM::IsLinear(rResult,ComplexID);

    EXPECT_FALSE(bLinear);

    bool bCycle=SGM::IsCycle(rResult,ComplexID);

    EXPECT_FALSE(bCycle);

    bool bOrigined=SGM::IsOriented(rResult,ComplexID);

    EXPECT_TRUE(bOrigined);

    bool bManifold=SGM::IsManifold(rResult,ComplexID);

    EXPECT_TRUE(bManifold);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sheet_body_check) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,0);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    EXPECT_TRUE(SGM::IsSheetBody(rResult,BlockID));
    EXPECT_FALSE(SGM::IsWireBody(rResult,BlockID));

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, topology_traversal) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body BlockID=SGM::CreateBlock(rResult,Pos1,Pos2);
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BlockID);
    
    // Top level checkes.

    SGM::Volume VolumeID;
    SGM::Face FaceID;
    SGM::Edge EdgeID;
    SGM::Vertex VertexID;
    SGM::Curve CurveID;
    SGM::Surface SurfaceID;
    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,SGM::Thing(),sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,SGM::Thing(),sComplexes);
        EXPECT_EQ(sComplexes.size(),1);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
        EXPECT_EQ(sVolumes.size(),1);
        VolumeID=*(sVolumes.begin());

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,SGM::Thing(),sFaces);
        EXPECT_EQ(sFaces.size(),6);
        FaceID=*(sFaces.begin());

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,SGM::Thing(),sEdges);
        EXPECT_EQ(sEdges.size(),12);
        EdgeID=*(sEdges.begin());

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,SGM::Thing(),sVertex);
        EXPECT_EQ(sVertex.size(),8);
        VertexID=*(sVertex.begin());

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,SGM::Thing(),sCurves);
        EXPECT_EQ(sCurves.size(),12);
        CurveID=*(sCurves.begin());

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,SGM::Thing(),sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6);
        SurfaceID=*(sSurfaces.begin());
    }

    // Body level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,BlockID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,BlockID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,BlockID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,BlockID,sFaces);
        EXPECT_EQ(sFaces.size(),6);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,BlockID,sEdges);
        EXPECT_EQ(sEdges.size(),12);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,BlockID,sVertex);
        EXPECT_EQ(sVertex.size(),8);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,BlockID,sCurves);
        EXPECT_EQ(sCurves.size(),12);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,BlockID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6);
    }

    // Volume level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,VolumeID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,VolumeID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,VolumeID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,VolumeID,sFaces);
        EXPECT_EQ(sFaces.size(),6);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,VolumeID,sEdges);
        EXPECT_EQ(sEdges.size(),12);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,VolumeID,sVertex);
        EXPECT_EQ(sVertex.size(),8);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,VolumeID,sCurves);
        EXPECT_EQ(sCurves.size(),12);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,VolumeID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6);
    }

    // Face level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,FaceID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,FaceID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,FaceID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,FaceID,sFaces);
        EXPECT_EQ(sFaces.size(),1);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,FaceID,sEdges);
        EXPECT_EQ(sEdges.size(),4);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,FaceID,sVertex);
        EXPECT_EQ(sVertex.size(),4);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,FaceID,sCurves);
        EXPECT_EQ(sCurves.size(),4);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,FaceID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),1);
    }

    // Surface level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,SurfaceID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,SurfaceID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,SurfaceID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,SurfaceID,sFaces);
        EXPECT_EQ(sFaces.size(),1);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,SurfaceID,sEdges);
        EXPECT_EQ(sEdges.size(),4);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,SurfaceID,sVertex);
        EXPECT_EQ(sVertex.size(),4);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,SurfaceID,sCurves);
        EXPECT_EQ(sCurves.size(),4);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,SurfaceID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),1);
    }

    // Edge level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,EdgeID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,EdgeID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,EdgeID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,EdgeID,sFaces);
        EXPECT_EQ(sFaces.size(),2);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,EdgeID,sEdges);
        EXPECT_EQ(sEdges.size(),1);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,EdgeID,sVertex);
        EXPECT_EQ(sVertex.size(),2);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,EdgeID,sCurves);
        EXPECT_EQ(sCurves.size(),1);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,EdgeID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),2);
    }

    // Curve level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,CurveID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,CurveID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,CurveID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,CurveID,sFaces);
        EXPECT_EQ(sFaces.size(),2);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,CurveID,sEdges);
        EXPECT_EQ(sEdges.size(),1);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,CurveID,sVertex);
        EXPECT_EQ(sVertex.size(),2);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,CurveID,sCurves);
        EXPECT_EQ(sCurves.size(),1);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,CurveID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),2);
    }

    // Vertex level checkes.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,VertexID,sBodies);
        EXPECT_EQ(sBodies.size(),1);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,VertexID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,VertexID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,VertexID,sFaces);
        EXPECT_EQ(sFaces.size(),3);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,VertexID,sEdges);
        EXPECT_EQ(sEdges.size(),3);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,VertexID,sVertex);
        EXPECT_EQ(sVertex.size(),1);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,VertexID,sCurves);
        EXPECT_EQ(sCurves.size(),3);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,VertexID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),3);
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sgm_save_and_read_cylinder) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body CylinderID=SGM::CreateCylinder(rResult,Pos1,Pos2,1);
    std::string file_path=get_models_file_path("cylinder.sgm");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,CylinderID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sgm_save_and_read_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0),Pos2(10,10,10);
    SGM::Body ConeID=SGM::CreateCone(rResult,Pos1,Pos2,1,2);
    std::string file_path=get_models_file_path("cone.sgm");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,ConeID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sgm_save_and_read_sphere) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0);
    SGM::Body ConeID=SGM::CreateSphere(rResult,Pos1,2);
    std::string file_path=get_models_file_path("sphere.sgm");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,ConeID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sgm_save_and_read_torus) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    SGM::Point3D Pos1(0,0,0);
    SGM::UnitVector3D Axis(0,0,1);
    SGM::Body TorusID=SGM::CreateTorus(rResult,Pos1,Axis,1,3);
    std::string file_path=get_models_file_path("torus.sgm");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, file_path, SGM::Thing() , Options); 
    SGM::DeleteEntity(rResult,TorusID);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    std::vector<std::string> aLog;
    SGM::CheckOptions CheckOptions;
    EXPECT_TRUE(SGM::CheckEntity(rResult,SGM::Thing(),CheckOptions,aLog));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, lemon_and_apple_tori) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
 
    // Test inverse on lemon and apple tori.

    SGM::Point3D Center;
    SGM::UnitVector3D Axis(0.0,0.0,1.0),XAxis(1.0,0.0,0.0);
    SGM::Surface AppleID=SGM::CreateTorusSurface(rResult,Center,Axis,2.0,0.5,true,&XAxis);
    SGM::Surface LemonID=SGM::CreateTorusSurface(rResult,Center,Axis,2.0,0.5,false,&XAxis);

    SGM::Point3D Pos0(0.5,0.0,0.0);
    SGM::Point3D Pos1(1.5,0.0,2.0);
    SGM::Point3D CPos0,CPos1;
    SGM::SurfaceInverse(rResult,AppleID,Pos0,&CPos0);
    SGM::SurfaceInverse(rResult,LemonID,Pos0,&CPos1);
    
    SGM::DeleteEntity(rResult,AppleID);
    SGM::DeleteEntity(rResult,LemonID);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(math_check, create_revolve) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
 
    std::vector<SGM::Point3D> aPoints;
    aPoints.emplace_back(-2,.5,0);
    aPoints.emplace_back(-1,1.5,0);
    aPoints.emplace_back(0,1,0);
    aPoints.emplace_back(1,1.5,0);
    aPoints.emplace_back(2,2,0);
    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints);
    CreateEdge(rResult,CurveID);
    SGM::Point3D Origin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Body BodyID = SGM::CreateRevolve(rResult, Origin, Axis, CurveID);

    std::vector<std::string> aCheckStrings;
    SGM::CheckOptions Options;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, integrate) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    EXPECT_TRUE(SGM::RunInternalTest(rResult,1));

    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, body_copy_with_attribute) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::ChangeColor(rResult,BodyID,255,0,0);
    SGM::CopyEntity(rResult,BodyID);
    int nRed=0,nGreen=0,nBlue=0;
    SGM::GetColor(rResult,BodyID,nRed,nGreen,nBlue);
    EXPECT_EQ(nRed,255);
    EXPECT_FALSE(SGM::IsSheetBody(rResult,BodyID));

    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, wire_body) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Edge> sEdges;
    sEdges.insert(EdgeID);
    SGM::Body WireID=SGM::CreateWireBody(rResult,sEdges);
    EXPECT_TRUE(SGM::IsWireBody(rResult,WireID));
    SGM::ChangeColor(rResult,WireID,255,0,0);
    SGM::CopyEntity(rResult,WireID);
    int nRed=0,nGreen=0,nBlue=0;
    SGM::GetColor(rResult,WireID,nRed,nGreen,nBlue);
    EXPECT_EQ(nRed,255);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, point_body) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::set<SGM::Point3D> sPoints;
    sPoints.insert(SGM::Point3D(0,0,0));
    sPoints.insert(SGM::Point3D(10,10,10));
    SGM::Body BodyID=SGM::CreatePointBody(rResult,sPoints);
    EXPECT_FALSE(SGM::IsSheetBody(rResult,BodyID));
    EXPECT_FALSE(SGM::IsWireBody(rResult,BodyID));
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, circle_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Normal(0,0,1);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Normal,1);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Point3D Pos0(1,0,0),Pos1(-1,0,0);
    SGM::ImprintPoint(rResult,Pos0,EdgeID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos1,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, ellipse_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0),YAxis(0,1,0);
    double dXRadius=1,dYRadius=2;
    SGM::Curve CurveID=SGM::CreateEllipse(rResult,Center,XAxis,YAxis,dXRadius,dYRadius);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Point3D Pos0(1,0,0),Pos1(-1,0,0);
    SGM::ImprintPoint(rResult,Pos0,EdgeID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos1,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, line_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(0,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Curve CurveID=SGM::CreateLine(rResult,Origin,Axis);
    SGM::Interval1D Domain(-10,10);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID,&Domain);
    SGM::Point3D Pos0(0,0,0);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, parabola_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0),YAxis(0,1,0);
    double dA=1.0;
    SGM::Curve CurveID=SGM::CreateParabola(rResult,Center,XAxis,YAxis,dA);
    SGM::Interval1D Domain(-10,10);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID,&Domain);
    SGM::Point3D Pos0(0,0,0);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,1,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, hyperbola_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(0,1,0),YAxis(1,0,0);
    SGM::Curve CurveID=SGM::CreateHyperbola(rResult,Center,XAxis,YAxis,1,1);
    SGM::Interval1D Domain(-10,10);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID,&Domain);
    SGM::Point3D Pos0(1,0,0);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check,hermite_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::Vector3D> aVectors;
    std::vector<double> aParams;
    aPoints.push_back(SGM::Point3D(0,0,0));
    aPoints.push_back(SGM::Point3D(10,0,0));
    aVectors.push_back(SGM::Vector3D(1,1,0));
    aVectors.push_back(SGM::Vector3D(1,1,0));
    aParams.push_back(0);
    aParams.push_back(12);
    SGM::Curve CurveID=SGM::CreateHermitCurve(rResult,aPoints,aVectors,aParams);

    SGM::Point3D Pos0;
    SGM::EvaluateCurve(rResult,CurveID,6,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, NUB_curve_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1;
    aPoints1.emplace_back(-2,.5,0);
    aPoints1.emplace_back(-1,1.5,0);
    aPoints1.emplace_back(0,1,0);
    aPoints1.emplace_back(1,1.5,0);
    aPoints1.emplace_back(2,2,0);

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Pos0;
    double dt=SGM::GetDomainOfCurve(rResult,CurveID).MidPoint();
    SGM::EvaluateCurve(rResult,CurveID,dt,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, DISABLED_NURB_curve_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,1,0,1);
    aControlPoints.emplace_back(-1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(-1,0,0,1);
    aControlPoints.emplace_back(-1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(0,-1,0,1);
    aControlPoints.emplace_back(1,-1,0,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,0,1);
    
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

    SGM::Curve CurveID=SGM::CreateNURBCurve(rResult,aControlPoints,aKnots);

    SGM::Point3D Pos0;
    double dt=SGM::GetDomainOfCurve(rResult,CurveID).MidPoint();
    SGM::EvaluateCurve(rResult,CurveID,dt,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, ray_fire_thing) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BodyID);
    SGM::TransformEntity(rResult,SGM::Transform3D(SGM::Vector3D(1,1,1)),ComplexID);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::RayFire(rResult,SGM::Point3D(-1,-1,-1),SGM::UnitVector3D(1,1,1),SGM::Thing(),aPoints,aTypes);

    EXPECT_EQ(aPoints.size(),4);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_coincident_lines) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,1,1));
    SGM::Curve LineID2=SGM::CreateLine(rResult,SGM::Point3D(1,1,1),SGM::UnitVector3D(-1,-1,-1));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID1,LineID2,aPoints,aTypes);

    EXPECT_EQ(aPoints.size(),2);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, DISABLED_intersect_line_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface SurfID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,0.5);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndSurface(rResult,LineID,SurfID,aPoints,aTypes);

    EXPECT_EQ(aPoints.size(),2);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_circle_circle) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve CircleID1=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Curve CircleID2=SGM::CreateCircle(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,0,1),1);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,CircleID1,CircleID2,aPoints,aTypes);

    EXPECT_EQ(aPoints.size(),2);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_sphere_sphere) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SphereID1=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface SphereID2=SGM::CreateSphereSurface(rResult,SGM::Point3D(1,0,0),1);
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SphereID1,SphereID2,aCurves);

    EXPECT_EQ(aCurves.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_sphere_plane) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface PlaneID=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0),SGM::Point3D(0,1,0));
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SphereID,PlaneID,aCurves);

    EXPECT_EQ(aCurves.size(),1);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_sphere_cylinder) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Surface PlaneID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0),0.5);
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SphereID,PlaneID,aCurves);

    EXPECT_EQ(aCurves.size(),2);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, complex_tests) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints;
    std::vector<unsigned int> aTriangles;
    aPoints.push_back(SGM::Point3D(0,0,0));
    aPoints.push_back(SGM::Point3D(1,0,0));
    aPoints.push_back(SGM::Point3D(0,1,0));
    aPoints.push_back(SGM::Point3D(1,1,0));
    aPoints.push_back(SGM::Point3D(0,1,0));
    aPoints.push_back(SGM::Point3D(1,0,0));
    aTriangles.push_back(0);
    aTriangles.push_back(1);
    aTriangles.push_back(2);
    aTriangles.push_back(3);
    aTriangles.push_back(4);
    aTriangles.push_back(5);
    SGM::Complex ComplexID=SGM::CreateTriangles(rResult,aPoints,aTriangles);

    std::vector<unsigned int> aSegments;
    aSegments.push_back(0);
    aSegments.push_back(1);
    aSegments.push_back(1);
    aSegments.push_back(2);
    aSegments.push_back(2);
    aSegments.push_back(0);
    aSegments.push_back(3);
    aSegments.push_back(4);
    SGM::Complex SegmentsID=SGM::CreateSegments(rResult,aPoints,aSegments);

    std::vector<SGM::Complex> aComponents;
    EXPECT_EQ(SGM::FindComponents(rResult,SegmentsID,aComponents),3);
    SGM::ReduceToUsedPoints(rResult,SegmentsID);
    aComponents.clear();
    EXPECT_EQ(SGM::FindComponents(rResult,SegmentsID,aComponents),2);
    aComponents.clear();
    EXPECT_EQ(SGM::FindPlanarParts(rResult,SegmentsID,aComponents,SGM_MIN_TOL),1);
    SGM::FindBoundary(rResult,ComplexID);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, point_in_tests) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Volume> sVolume;
    SGM::FindVolumes(rResult,BodyID,sVolume);
    SGM::Volume VolumeID=*(sVolume.begin());
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    EXPECT_FALSE(SGM::PointInEntity(rResult,SGM::Point3D(20,0,0),VolumeID));
    EXPECT_FALSE(SGM::PointInEntity(rResult,SGM::Point3D(20,0,0),FaceID));

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, checking_the_checker) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Check for empty volumes.

    SGM::Body BodyID=SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Volume VolumeID=SGM::FindVolume(rResult,FaceID);
    SGM::DeleteEntity(rResult,FaceID);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_FALSE(SGM::CheckEntity(rResult,VolumeID,Options,aCheckStrings));

    // Check for empty bodies.

    SGM::DeleteEntity(rResult,VolumeID);
    SGM::CheckOptions Options2;
    std::vector<std::string> aCheckStrings2;
    EXPECT_FALSE(SGM::CheckEntity(rResult,BodyID,Options2,aCheckStrings2));

    // Bad segments in a complex.

    std::vector<SGM::Point3D> aPoints;
    std::vector<unsigned int> aSegments;
    aPoints.push_back(SGM::Point3D(0,0,0));
    aSegments.push_back(0);
    aSegments.push_back(0);
    aSegments.push_back(1);
    aSegments.push_back(2);
    SGM::Complex ComplexID=SGM::CreateSegments(rResult,aPoints,aSegments);
    SGM::CheckOptions Options3;
    std::vector<std::string> aCheckStrings3;
    EXPECT_FALSE(SGM::CheckEntity(rResult,ComplexID,Options3,aCheckStrings3));
    SGM::DeleteEntity(rResult,ComplexID);

    // Bad triangles in a complex.

    std::vector<SGM::Point3D> aPoints2;
    std::vector<unsigned int> aTriangles2;
    aPoints2.push_back(SGM::Point3D(0,0,0));
    aTriangles2.push_back(0);
    aTriangles2.push_back(0);
    aTriangles2.push_back(2);
    SGM::Complex ComplexID2=SGM::CreateTriangles(rResult,aPoints2,aTriangles2);
    SGM::CheckOptions Options4;
    std::vector<std::string> aCheckStrings4;
    EXPECT_FALSE(SGM::CheckEntity(rResult,ComplexID2,Options4,aCheckStrings4));
    SGM::DeleteEntity(rResult,ComplexID2);

    SGMTesting::ReleaseTestThing(pThing);
} 