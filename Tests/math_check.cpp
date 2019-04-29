#include <gtest/gtest.h>

#include "SGMVector.h"
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
#include "SGMAttribute.h"
#include "SGMTranslators.h"
#include "SGMChecker.h"
#include "SGMTriangle.h"
#include "SGMPolygon.h"

#define SGM_TIMER 
#include "Util/timer.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(math_check, least_sqaures_circle)
{
    std::vector<SGM::Point3D> aPoints={{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
    SGM::Point3D Center;
    SGM::UnitVector3D Normal;
    double dRadius;
    EXPECT_TRUE(SGM::FindLeastSqaureCircle3D(aPoints,Center,Normal,dRadius));
    EXPECT_TRUE(SGM::NearEqual(Normal,SGM::Vector3D(0,0,1),SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(Center,SGM::Point3D(0.5,0.5,0),SGM_MIN_TOL));
    EXPECT_NEAR(dRadius,0.70710678118654752440084436210485,SGM_ZERO);
}

TEST(math_check, point_in_tetrahedron)
{
    SGM::Point3D A(0,0,0),B(1,0,0),C(0,1,0),D(0,0,1);
    SGM::Point3D Pos1(0.1,0.1,0.1),Pos2(1,1,1);
    bool bIn1=SGM::PointInTetrahedron(A,B,C,D,Pos1);
    bool bIn2=SGM::PointInTetrahedron(A,B,C,D,A);
    bool bIn3=SGM::PointInTetrahedron(A,B,C,D,Pos2);
    EXPECT_TRUE(bIn1);
    EXPECT_TRUE(bIn2);
    EXPECT_FALSE(bIn3);
}

TEST(math_check, dihedral_angle)
{
    SGM::Point3D A(1,0,0),B(1,1,0),C(0,0,0),D1(1,0,1),D2(1,0,-1);
    double dAngle1=SGM::DihedralAngle(A,B,C,D1);
    double dAngle2=SGM::DihedralAngle(A,B,C,D2);
    EXPECT_NEAR(dAngle1,-SGM_HALF_PI,SGM_ZERO);
    EXPECT_NEAR(dAngle2,SGM_HALF_PI,SGM_ZERO);
}

TEST(math_check, distance_squared_to_triangle)
{
    {
    SGM::Point3D A(0,0,0),B(10,0,0),C(0,10,0);
    SGM::Point3D P1(10,10,10);
    double dDistanceSquared1=DistanceSquaredTriangle3D(A,B,C,P1);

    EXPECT_NEAR(dDistanceSquared1,150,SGM_ZERO);

    SGM::Point3D P2(2,2,2);
    double dDistanceSquared2=DistanceSquaredTriangle3D(A,B,C,P2);

    EXPECT_NEAR(dDistanceSquared2,4,SGM_ZERO);

    SGM::Point3D P3(5,-5,0);
    double dDistanceSquared3=DistanceSquaredTriangle3D(A,B,C,P3);

    EXPECT_NEAR(dDistanceSquared3,25,SGM_ZERO);

    SGM::Point3D P4(-6,4,0);
    double dDistanceSquared4=DistanceSquaredTriangle3D(A,B,C,P4);

    EXPECT_NEAR(dDistanceSquared4,36,SGM_ZERO);

    SGM::Point3D P5(-1,-1,0);
    double dDistanceSquared5=DistanceSquaredTriangle3D(A,B,C,P5);

    EXPECT_NEAR(dDistanceSquared5,2,SGM_ZERO);

    SGM::Point3D P6(11,-1,-1);
    double dDistanceSquared6=DistanceSquaredTriangle3D(A,B,C,P6);

    EXPECT_NEAR(dDistanceSquared6,3,SGM_ZERO);

    SGM::Point3D P7(11,-1,1);
    double dDistanceSquared7=DistanceSquaredTriangle3D(A,B,C,P7);

    EXPECT_NEAR(dDistanceSquared7,3,SGM_ZERO);

    SGM::Point3D P8(2,2,-2);
    double dDistanceSquared8=DistanceSquaredTriangle3D(A,B,C,P8);

    EXPECT_NEAR(dDistanceSquared8,4,SGM_ZERO);
    }
    
    {
    SGM::Point3D A(0,0,0),B(10,10,0),C(5,0,0);
    SGM::Point3D P1(2,1,1);
    double dDistanceSquared1=DistanceSquaredTriangle3D(A,B,C,P1);

    EXPECT_NEAR(dDistanceSquared1,1,SGM_ZERO);

    double dDistanceSquared2=DistanceSquaredTriangle3D(A,C,B,P1);

    EXPECT_NEAR(dDistanceSquared2,1,SGM_ZERO);

    double dDistanceSquared3=DistanceSquaredTriangle3D(C,A,B,P1);

    EXPECT_NEAR(dDistanceSquared3,1,SGM_ZERO);
    }

    {
    SGM::Point3D A(0,0,0),B(10,10.00000000001,0),C(5,5,0);
    SGM::Point3D P1(6,4,0);
    double dDistanceSquared1=DistanceSquaredTriangle3D(A,B,C,P1);

    EXPECT_NEAR(dDistanceSquared1,2,SGM_ZERO);
    }

    {
    SGM::Point3D A(0,0,0),B(10,10.0000000000001,0),C(5,5,0);
    SGM::Point3D P1(6,4,0);
    double dDistanceSquared1=DistanceSquaredTriangle3D(A,B,C,P1);

    EXPECT_NEAR(dDistanceSquared1,2,SGM_ZERO);
    }
}

TEST(math_check, bulge_area)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;
    std::string file_path = get_models_file_path("bulge.stp");
    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, aEntities, log, options);
    ASSERT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,aEntities[0],sFaces);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);
    auto iter=sFaces.begin();
    ++iter;
    SGM::Face FaceID=*(iter);
    double dArea=SGM::FindArea(rResult,FaceID);
   
    EXPECT_NEAR(dArea,40.676307120618583750332669282937,SGM_MIN_TOL);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, ruled_surface)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 = {{0,0,0}, {1,0,0}};
    std::vector<SGM::Point3D> aPoints2 = {{0,1,0}, {1,1,.7}};
    std::vector<std::vector<SGM::Point3D>> aaPoints;
    aaPoints.push_back(aPoints1);
    aaPoints.push_back(aPoints2);
    SGM::Surface NUBID = SGM::CreateNUBSurface(rResult, aaPoints);

    std::vector<SGM::Edge> aEdges;
    std::vector<SGM::EdgeSideType> aTypes;
    /*SGM::Body BodyID = */SGM::CreateSheetBody(rResult, NUBID, aEdges, aTypes);

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(math_check, top_level_faces)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 =
        {
            {-2, 0.5, 0},
            {-1, 1.5, 0},
            { 0, 1,   0},
            { 1, 1.5, 0},
            { 2, 2,   0}
        };

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Origin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Surface RevolveID=SGM::CreateRevolveSurface(rResult,Origin,Axis,CurveID);

    std::vector<SGM::Edge> aEdges;
    std::vector<SGM::EdgeSideType> aTypes;
    SGM::CreateFaceFromSurface(rResult,RevolveID,aEdges,aTypes);

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aTypes2;
    std::vector<SGM::Entity> aEntities2;
    SGM::RayFire(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),SGM::Thing(),aHits,aTypes2,aEntities2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, disk_area)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    double dArea=SGM::FindArea(rResult,FaceID);
    
    EXPECT_NEAR(dArea,SGM_PI,SGM_MIN_TOL);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, edge_params)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,1,1));
    SGM::GetEdgeParams(rResult,EdgeID);
    SGM::GetStartPointOfEdge(rResult,EdgeID);
    SGM::GetEndPointOfEdge(rResult,EdgeID);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, plane_param_line)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Interval2D Domain(0,1,0,1);
    SGM::SetDomainOfSurface(rResult,SurfID,Domain);
    size_t Index1;
    for(Index1=0;Index1<5;++Index1)
        {
        double dU=Domain.m_UDomain.MidPoint(Index1/4.0);
        SGM::Curve CurveIDU=SGM::FindUParamCurve(rResult,SurfID,dU);
        SGM::CreateEdge(rResult,CurveIDU);
        
        double dV=Domain.m_VDomain.MidPoint(Index1/4.0);
        SGM::Curve CurveIDV=SGM::FindVParamCurve(rResult,SurfID,dV);
        SGM::CreateEdge(rResult,CurveIDV);
        }
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sphere_param_line)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreateSphereSurface(rResult,SGM::Point3D(0,0,0),1);
    SGM::Interval2D Domain=SGM::GetDomainOfSurface(rResult,SurfID);
    size_t Index1;
    for(Index1=0;Index1<10;++Index1)
        {
        double dU=Domain.m_UDomain.MidPoint(Index1/10.0);
        SGM::Curve CurveIDU=SGM::FindUParamCurve(rResult,SurfID,dU);
        SGM::CreateEdge(rResult,CurveIDU);
        
        double dV=Domain.m_VDomain.MidPoint(Index1/10.0);
        SGM::Curve CurveIDV=SGM::FindVParamCurve(rResult,SurfID,dV);
        SGM::CreateEdge(rResult,CurveIDV);
        }
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, create_sheet_body_with_edges)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreatePlaneFromOriginAndNormal(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1));
    SGM::Curve CurveID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(EdgeID);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::FaceOnLeftType);
    /* SGM::Body BodyID= */ SGM::CreateSheetBody(rResult,SurfID,aEdges,aTypes);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, singular_in_V_NURBSurface)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point4D>> aaPoints = 
        {{{0,0,0,0}, {1,0,0,0}, {2,0,0,0}},
         {{0,0,0,0}, {1,1,0,0}, {2,0,0,0}},
         {{0,0,0,0}, {1,2,0,0}, {2,0,0,0}}};

    SGM::Surface NUBSurfaceID=SGM::CreateNURBSurface(rResult,aaPoints,aUKnots,aVKnots);

    EXPECT_TRUE(SGM::IsSurfaceSingularHighV(rResult,NUBSurfaceID));
    EXPECT_TRUE(SGM::IsSurfaceSingularLowV(rResult,NUBSurfaceID));

    SGM::FindVParamCurve(rResult,NUBSurfaceID,0);
    SGM::FindVParamCurve(rResult,NUBSurfaceID,1);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, singular_in_U_NURBSurface)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point4D>> aaPoints = 
        {{{0,0,0,0}, {0,0,0,0}, {0,0,0,0}},
         {{1,0,0,0}, {1,1,0,0}, {1,2,0,0}},
         {{2,0,0,0}, {2,0,0,0}, {2,0,0,0}}};
    
    SGM::Surface NUBSurfaceID=SGM::CreateNURBSurface(rResult,aaPoints,aUKnots,aVKnots);

    EXPECT_TRUE(SGM::IsSurfaceSingularHighU(rResult,NUBSurfaceID));
    EXPECT_TRUE(SGM::IsSurfaceSingularLowU(rResult,NUBSurfaceID));
    EXPECT_TRUE(SGM::IsSingularity(rResult,NUBSurfaceID,SGM::Point2D(0,0),SGM_MIN_TOL));

    SGM::FindUParamCurve(rResult,NUBSurfaceID,0);
    SGM::FindUParamCurve(rResult,NUBSurfaceID,1);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, singular_in_V_NUBSurface)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point3D>> aaPoints = 
        {{{0,0,0},{1,0,0},{2,0,0}},
         {{0,0,0},{1,1,0},{2,0,0}},
         {{0,0,0},{1,2,0},{2,0,0}}};
    
    SGM::Surface NUBSurfaceID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    SGM::FindVParamCurve(rResult,NUBSurfaceID,0);
    SGM::FindVParamCurve(rResult,NUBSurfaceID,1);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, singular_in_U_NUBSurface)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);

    std::vector<std::vector<SGM::Point3D>> aaPoints = 
        {{{0,0,0},{0,0,0},{0,0,0}},
         {{1,0,0},{1,1,0},{1,2,0}},
         {{2,0,0},{2,0,0},{2,0,0}}};
    
    SGM::Surface NUBSurfaceID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    SGM::FindUParamCurve(rResult,NUBSurfaceID,0);
    SGM::FindUParamCurve(rResult,NUBSurfaceID,1);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, imprint_face_on_face_through_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateDisk(rResult,SGM::Point3D(10,10,0),SGM::UnitVector3D(1,-1,0),1);
    SGM::UniteBodies(rResult,BodyID2,BodyID1);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, imprint_edge_on_face_vertex_hit)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    SGM::Edge EdgeID1=SGM::CreateLinearEdge(rResult,SGM::Point3D(6,6,0),SGM::Point3D(10,10,0));
    SGM::ImprintEdgeOnFace(rResult,EdgeID1,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, DISABLED_imprint_edge_on_face_atoll_flipped)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body DiskID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,DiskID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,-1),0.5);
    SGM::Edge EdgeID1=SGM::CreateEdge(rResult,CircleID);
    SGM::ImprintEdgeOnFace(rResult,EdgeID1,FaceID);

    SGM::Edge EdgeID2=SGM::CreateLinearEdge(rResult,SGM::Point3D(0.5,0,0),SGM::Point3D(1,0,0));
    SGM::ImprintEdgeOnFace(rResult,EdgeID2,FaceID);

    SGMTesting::CheckEntityAndPrintLog(rResult,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, color_inheritance)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,BodyID,sVolumes);
    SGM::Volume VolumeID=*(sVolumes.begin());
    SGM::ChangeColor(rResult,BodyID,255,0,0);
    int nRed,nBlue,nGreen;
    SGM::GetColor(rResult,FaceID,nRed,nGreen,nBlue);
    EXPECT_EQ(nRed,255);
    SGM::ChangeColor(rResult,VolumeID,254,0,0);
    SGM::GetColor(rResult,FaceID,nRed,nGreen,nBlue);
    EXPECT_EQ(nRed,254);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, public_math_functions)
{
    SGM::SAFEasin(-1.1);
    SGM::SAFEasin(1.1);
    SGM::SAFEasin(0);
    SGM::SignedArea(SGM::Point2D(0,0),SGM::Point2D(1,0),SGM::Point2D(0,1));
    SGM::CenterOfMass(SGM::Point3D(0,0,0),SGM::Point3D(1,0,0),SGM::Point3D(0,1,0));
    std::vector<SGM::Point3D> aPoints = {{0,0,0},{0.5,0.1,0.1},{1,0,0}};
    SGM::Point3D Origin;
    SGM::UnitVector3D Axis;
    SGM::FindLeastSquareLine3D(aPoints,Origin,Axis);

    std::vector<double> aRoots1,aRoots2,aRoots3;
    SGM::Quartic(1.0/4,1.0/3,-1.0/2,-1,0,aRoots1,SGM_MIN_TOL);
    SGM::Quartic(-1,1,1,1,1,aRoots2,SGM_MIN_TOL);
    SGM::Quartic(0,1,1,1,1,aRoots3,SGM_MIN_TOL);
}

TEST(math_check, public_functions_tests )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Vertex> sVertex;
    SGM::FindVertices(rResult,BodyID,sVertex);
    SGM::Vertex VertexID=*(sVertex.begin());
    SGM::GetPointOfVertex(rResult,VertexID);
    SGM::FindTopLevelEntities(rResult);
    std::vector<SGM::Face> aFaces;
    SGM::FindCloseFaces(rResult,SGM::Point3D(0,0,0),BodyID,10,aFaces);
    SGM::Attribute AttributeID=SGM::CreateAttribute(rResult,"Test");
    SGM::GetAttributeName(rResult,AttributeID);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, point_in_face_vertex_one_edge_case )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,BodyID,sEdges);
    SGM::Edge EdgeID=*(sEdges.begin());
    SGM::ImprintPoint(rResult,SGM::Point3D(1,0,0),EdgeID);
    SGM::PointInEntity(rResult,SGM::Point3D(0.5,0,0),FaceID);

    SGM::ImprintPoint(rResult,SGM::Point3D(-1,0,0),EdgeID);
    SGM::PointInEntity(rResult,SGM::Point3D(0.5,0,0),FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, step_save_wire_body )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,0,0));
    std::set<SGM::Edge> sEdges;
    sEdges.insert(EdgeID);
    SGM::Body WireID=SGM::CreateWireBody(rResult,sEdges);
    SGM::SaveSTEP(rResult,"Gtest_wire_save.stp",SGM::Thing(),SGM::TranslatorOptions());

    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,WireID,sVertices);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, remove_duplicates_2d )
{
    std::vector<SGM::Point2D> aPoints = {{0,0},{0,0}};
    SGM::RemoveDuplicates2D(aPoints,SGM_ZERO);
    EXPECT_EQ(aPoints.size(),1U);
}

TEST(math_check, lemon_torus )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,0.25,false);

    SGM::Point3D Pos(0.25,0,0.1),ClosePos;
    SGM::Point2D GuessUV(SGM_TWO_PI,0);
    SGM::Point2D uv=SGM::SurfaceInverse(rResult,SurfID,Pos,&ClosePos,&GuessUV);
    EXPECT_TRUE(SGM::NearEqual(uv.m_u,SGM_TWO_PI,SGM_ZERO,false));

    SGM::Point2D GuessUV2(0,0);
    SGM::Point2D uv2=SGM::SurfaceInverse(rResult,SurfID,Pos,&ClosePos,&GuessUV2);
    EXPECT_TRUE(SGM::NearEqual(uv2.m_u,0,SGM_ZERO,false));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, pinched_torus )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,1);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, face_color_test )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::ChangeColor(rResult,FaceID,255,0,0);
    int nRed,nGreen,nBlue;
    SGM::GetColor(rResult,FaceID,nRed,nGreen,nBlue);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, thing_tests)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Attribute AttributeID=SGM::CreateAttribute(rResult,"Test");
    SGM::Thing ThingID;
    SGM::AddAttribute(rResult,ThingID,AttributeID);
    std::set<SGM::Attribute> sAttributes;
    SGM::GetAttributes(rResult,ThingID,sAttributes,true);
    try { SGM::TransformEntity(rResult,SGM::Transform3D(SGM::Vector3D(1,1,1)),ThingID); } catch (const std::logic_error&) {}

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, graph_branch_tests)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Graph EdgeGraph(rResult,sFaces,true);
    SGM::Face FaceID=*(sFaces.begin());
    size_t nVert=FaceID.m_ID;
    EdgeGraph.GetStar(nVert);
    std::vector<SGM::Graph> aBranches;
    EdgeGraph.FindBranches(aBranches);

    std::set<size_t> sVertices = {0,1};
    std::set<SGM::GraphEdge> sEdges;
    sEdges.insert(SGM::GraphEdge(0,1,1,true));
    SGM::Graph DirectedGraph(sVertices,sEdges);
    std::vector<size_t> aSources;
    DirectedGraph.FindSources(aSources);

    sVertices.insert(2);
    sVertices.insert(3);
    sVertices.insert(4);
    sEdges.clear();
    sEdges.insert({0,1,1});
    sEdges.insert({2,1,1});
    sEdges.insert({2,3,1});
    sEdges.insert({2,4,1});
    SGM::Graph BranchedGraph(sVertices,sEdges);
    std::vector<SGM::Graph> aBranches2;
    BranchedGraph.FindBranches(aBranches2);
        
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sgm_file_complex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{0,1,0}};
    std::vector<unsigned> aTriangles = {0,1,2};
    std::vector<unsigned> aSegments = {0,1};
    SGM::CreateComplex(rResult,aPoints,aSegments,aTriangles);

    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult,"GTestFile.sgm",SGM::Thing(),Options);
    std::vector<std::string> aLog;
    std::vector<SGM::Entity> aEntities;
    SGM::ReadFile(rResult,"GTestFile.sgm",aEntities,aLog,Options);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, sortable_planes)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::RunInternalTest(rResult,4);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, check_bad_bodies)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    rResult.SetDebugFlag(5); // Make a bad body in CreateBlock.

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    SGM::CheckEntity(rResult,BlockID,Options,aCheckStrings);

    SGM::Assembly ID1;
    SGM::Reference ID2;
    SGM::Complex ID3;
    SGM::Body ID4;
    ID1.m_ID*=1;
    ID2.m_ID*=1;
    ID3.m_ID*=1;
    ID4.m_ID*=1;
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, closest_point_on_things)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Volume> sVolume;
    SGM::FindVolumes(rResult,BlockID,sVolume);
    SGM::Volume VolumeID=*(sVolume.begin());
    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,BlockID,sVertices);
    SGM::Vertex VertexID=*(sVertices.begin());

    SGM::Point3D ClosePos;
    SGM::Entity ClosestEntity;
    SGM::FindClosestPointOnEntity(rResult,SGM::Point3D(0,0,0),VolumeID,ClosePos,ClosestEntity);
    SGM::FindClosestPointOnEntity(rResult,SGM::Point3D(5,5,5),VolumeID,ClosePos,ClosestEntity,false);
    SGM::FindClosestPointOnEntity(rResult,SGM::Point3D(0,0,0),VertexID,ClosePos,ClosestEntity);
    SGM::FindClosestPointOnEntity(rResult,SGM::Point3D(0,0,0),BlockID,ClosePos,ClosestEntity);
    SGM::FindClosestPointOnEntity(rResult,SGM::Point3D(0,0,0),SGM::Thing(),ClosePos,ClosestEntity);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, points_in_volumes)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    /* SGM::Body SphereID = */ SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    std::vector<SGM::Point3D> aPoints = {{0,0,0}};
    std::vector<std::vector<SGM::Volume>> aaVolumeIDs;
    SGM::PointsInVolumes(rResult,aPoints,aaVolumeIDs);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, vector_angle_and_transforms)
{
    SGM::UnitVector3D UVec1(0,0,1),UVec2(1,0,0);
    EXPECT_TRUE(SGM::NearEqual(UVec1.Angle(UVec2),SGM_HALF_PI,SGM_MIN_TOL,false));

    SGM::Vector3D Vec(0,0,1);
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    Vec*=Trans;
    UVec1*=Trans;
}

TEST(math_check, find_comps_1d)
{
    std::vector<unsigned> aSegments = {0,1};
    EXPECT_EQ(SGM::FindComponents1D(aSegments),1U);
    }

TEST(math_check, point_curve)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos(0,0,0);
    SGM::Interval1D Domain(0,1);
    SGM::Curve CurveID=SGM::CreatePointCurve(rResult,Pos,&Domain);
    EXPECT_TRUE(SGM::GetPointCurveData(rResult,CurveID,Pos));
    
    SGM::Curve CurveID2=SGM::Curve(SGM::CopyEntity(rResult,CurveID).m_ID);
    EXPECT_TRUE(SGM::SameCurve(rResult,CurveID,CurveID2,SGM_MIN_TOL));
    SGM::TransformEntity(rResult,SGM::Transform3D(SGM::Vector3D(1,1,1)),CurveID);
    EXPECT_FALSE(SGM::SameCurve(rResult,CurveID,CurveID2,SGM_MIN_TOL));

    SGM::Point3D ClosePos;
    double dGuess1=-1,dGuess2=0.5,dGuess3=2;
    SGM::CurveInverse(rResult,CurveID,Pos,&ClosePos,&dGuess1);
    SGM::CurveInverse(rResult,CurveID,Pos,&ClosePos,&dGuess2);
    SGM::CurveInverse(rResult,CurveID,Pos,&ClosePos,&dGuess3);
    SGM::Point3D xyz;
    SGM::Vector3D Vec1;
    SGM::Vector3D Vec2;
    SGM::EvaluateCurve(rResult,CurveID,0,&xyz,&Vec1,&Vec2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, remove_duplicates)
{
    std::vector<SGM::Point3D> aPoints = {{0,0,0},{2,0,0},{0,0,0},{2,0,0}};
    SGM::Interval3D Box(0,1,0,1,0,1);
    SGM::RemoveDuplicates3D(aPoints,SGM_MIN_TOL,&Box);
    EXPECT_EQ(aPoints.size(),1U);
}

TEST(math_check, relatively_prime)
{
    EXPECT_TRUE(SGM::RelativelyPrime(6,25));
}

TEST(math_check, do_points_match)
{
    std::vector<SGM::Point3D> aPoints1 = {{0,0,0},{1,0,0}};
    std::vector<SGM::Point3D> aPoints2 = {{0,0,0},{1,0,0}};
    std::map<unsigned,unsigned> mMatchMap;
    EXPECT_TRUE(SGM::DoPointsMatch(aPoints1,aPoints2,mMatchMap,SGM_MIN_TOL));  

    size_t nWhere;
    double dDist=SGM::DistanceToPoints(aPoints1,SGM::Point3D(3,0,0),nWhere);
    EXPECT_TRUE(SGM::NearEqual(dDist,2,SGM_MIN_TOL,false));
}

TEST(math_check, find_max_edge_length_2D)
{
    std::vector<SGM::Point2D> aPoints = {{0,0},{1,0},{2,0}};
    std::vector<unsigned> aTriangles = {0,1,2};
    double dMin=SGM::FindMaxEdgeLength2D(aPoints,aTriangles);
    EXPECT_TRUE(SGM::NearEqual(dMin,2,SGM_ZERO,false));
}

TEST(math_check, find_min_edge_length_3D)
{
    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{2,0,0}};
    std::vector<unsigned> aTriangles = {0,1,2};
    double dMin=SGM::FindMinEdgeLength3D(aPoints,aTriangles);
    EXPECT_TRUE(SGM::NearEqual(dMin,1,SGM_ZERO,false));
}

TEST(math_check, matrix_multiply)
{
    double aMatrix1[2][2] = {1, 2,
                             3, 4};
    double aMatrix2[2][2] = {1, 2,
                             3, 4};
    double aAnswer[2][2];
    SGM::FindProduct2D(aMatrix1,aMatrix2,aAnswer);
    EXPECT_NEAR(aAnswer[0][0],7,SGM_ZERO);
    EXPECT_NEAR(aAnswer[0][1],10,SGM_ZERO);
    EXPECT_NEAR(aAnswer[1][0],15,SGM_ZERO);
    EXPECT_NEAR(aAnswer[1][1],22,SGM_ZERO);

    double aMatrix3[3][3] = {1,2,2,
                             3,4,2,
                             3,4,2};
    double aAnswer2[3][3];
    SGM::FindProduct3D(aMatrix3,aMatrix3,aAnswer2);
}

//TEST(math_check, polynomial_fit)
//{
//    std::vector<SGM::Point2D> aPoints = {{-1,1},{0,0},{1,1}};
//    SGM::PolynomialFit(aPoints,aCoefficients);
//    EXPECT_TRUE(SGM::NearEqual(aCoefficients[0],1,SGM_MIN_TOL,false));
//    EXPECT_TRUE(SGM::NearEqual(aCoefficients[1],0,SGM_MIN_TOL,false));
//    EXPECT_TRUE(SGM::NearEqual(aCoefficients[2],0,SGM_MIN_TOL,false));
//}

TEST(math_check, partial_order)
{
    std::set<std::pair<size_t,size_t>> sPartialOrder;
    sPartialOrder.insert({1,0});
    sPartialOrder.insert({2,0});
    sPartialOrder.insert({4,3});
    sPartialOrder.insert({5,0});
    sPartialOrder.insert({5,1});
    std::vector<size_t> aMaximalElements;
    size_t nMax=SGM::FindMaximalElements(sPartialOrder,aMaximalElements);
    EXPECT_EQ(nMax,2U);

    std::vector<size_t> aAllDecendents;
    size_t nAllDecendents=SGM::FindDecendentsOfGroup(sPartialOrder,aMaximalElements,aAllDecendents);
    EXPECT_EQ(nAllDecendents,4U);

    std::vector<std::vector<size_t>> aaGenerations;
    size_t nGenerations=SGM::FindGenerations(sPartialOrder,0,aaGenerations);
    EXPECT_EQ(nGenerations,2U);

    std::vector<size_t> aDecendents;
    size_t nDecendents= SGM::FindDescendants(sPartialOrder, 0, aDecendents);
    EXPECT_EQ(nDecendents,3U);

    std::vector<size_t> aChildren;
    size_t nChildern= SGM::FindChildren(sPartialOrder, 0, aChildren);
    EXPECT_EQ(nChildern,2U);

    std::vector<size_t> aKeep = {0,2};
    SGM::SubsetPartialOrder(aKeep, sPartialOrder);
    EXPECT_EQ(sPartialOrder.size(),1U);

    
}
    
TEST(math_check, save_step_primitives)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    SGM::CreateCylinder(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::CreateCone(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1,2);
    SGM::Body BodyID=SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,3);

    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,BodyID);

    SGM::TranslatorOptions TranslatorOpts;
    SGM::SaveSTEP(rResult,"primitives.stp",SGM::Thing(),TranslatorOpts);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(math_check, revolve_surface_save_step)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 = 
        {
            {-2,.5,0},
            {-1,1.5,0},
            {0,1,0},
            {1,1.5,0},
            {2,2,0}
        };

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Origin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Surface RevolveID=SGM::CreateRevolveSurface(rResult,Origin,Axis,CurveID);

    std::vector<std::string> aLog1;
    SGM::CheckOptions Options1;
    SGM::CheckEntity(rResult,RevolveID,Options1,aLog1);

    SGM::SaveSGM(rResult,"GTest_revolve_test.sgm",SGM::Thing(),SGM::TranslatorOptions());

    std::vector<SGM::Edge> aEdges;
    std::vector<SGM::EdgeSideType> aTypes;
    SGM::Body BodyID=SGM::CreateSheetBody(rResult,RevolveID,aEdges,aTypes);
    
    SGM::TranslatorOptions TranslatorOpts;
    SGM::SaveSTEP(rResult,"revolve_sheet.stp",SGM::Thing(),TranslatorOpts);
    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> aLog;
    SGM::ReadFile(rResult,"revolve_sheet.stp",aEntities,aLog,TranslatorOpts);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID,Options,aCheckStrings));

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,RevolveID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,RevolveID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,BodyID);
    EXPECT_FALSE(SGM::SameSurface(rResult,RevolveID,SurfID2,SGM_MIN_TOL));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, revolve_surface_test)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Revolve Surface

    // NUB curve is a straight line parallel to the axis
    // both have a slope of 2:1, and distance between is sqrt(5.0)
    std::vector<double> aKnots1={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints1 = 
        {
            {3.5,1,0},
            {3.75,1.5,0},
            {4,2,0},
            {4.25,2.5,0},
            {4.5,3,0}
        };

    SGM::Curve NUBID1=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints1,aKnots1);

    SGM::Point3D Origin1(1.0,1.0,0.0);
    SGM::UnitVector3D Axis1(1.0,2.0,0.0);
    SGM::Surface RevolveID1=SGM::CreateRevolveSurface(rResult,Origin1,Axis1,NUBID1);

    SGM::Point3D Pos;
    SGM::Vector3D Du, Dv;
    SGM::EvaluateSurface(rResult,RevolveID1,SGM::Point2D(SGM_PI/2.0, 0.5), &Pos, &Du, &Dv);

    double dDistance = sqrt(5.0);
    EXPECT_TRUE(SGM::NearEqual(Pos, SGM::Point3D(2.0, 3.0, -dDistance), SGM_ZERO));
    SGM::UnitVector3D uDuDirection(-2,1,0);
    SGM::UnitVector3D UnitDu = Du;
    EXPECT_TRUE(SGM::NearEqual(uDuDirection,UnitDu,SGM_ZERO));
    SGM::UnitVector3D uDvDirection(1,2,0);
    SGM::UnitVector3D UnitDv = Dv;
    EXPECT_TRUE(SGM::NearEqual(uDvDirection,UnitDv,SGM_ZERO));

    // create a more general NUB to revolve
    std::vector<double> aKnots2 = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
    std::vector<SGM::Point3D> aControlPoints2 = {
            {1,                  1,                  0},
            {1.1666666666666666, 1.1666666666666666, 0},
            {2,                  2.8333333333333333, 0},
            {2.8333333333333333, 2.8333333333333333, 0},
            {3,                  3,                  0}
    };
    SGM::Curve NUBID2=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints2,aKnots2);

    SGM::Point3D Origin2 = {1.0,3.0,0.0};
    SGM::UnitVector3D Axis2 = {1.0,2.0,0.0};
    SGM::Surface RevolveID2=SGM::CreateRevolveSurface(rResult,Origin2,Axis2,NUBID2);

    EXPECT_TRUE(SGM::TestSurface(rResult,RevolveID2, SGM::Point2D(0.5,0.2)));

    SGM::DeleteEntity(rResult,RevolveID1);
    SGM::DeleteEntity(rResult,RevolveID2);
    SGM::DeleteEntity(rResult,NUBID1);
    SGM::DeleteEntity(rResult,NUBID2);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, extrude_hermite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{10,0,0}};
    std::vector<SGM::Vector3D> aVectors = {{1,1,0},{1,1,0}};
    std::vector<double> aParams = {0,12};
    SGM::Curve CurveID=SGM::CreateHermiteCurve(rResult,aPoints,aVectors,aParams);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,6));

    double dGuess=0;
    SGM::CurveInverse(rResult,CurveID,SGM::Point3D(0,0,0),nullptr,&dGuess);

    SGM::SaveSGM(rResult,"GTest_hermite_test.sgm",CurveID,SGM::TranslatorOptions());

    SGM::GetHermiteCurveData(rResult,CurveID,aPoints,aVectors,aParams);

    SGM::UnitVector3D Axis(0,0,1);
    SGM::Surface SurfID=SGM::CreateExtrudeSurface(rResult,Axis,CurveID);
    /* SGM::Surface SurfIDCopy = */ SGM::CreateExtrudeSurface(rResult,Axis,CurveID);
    EXPECT_TRUE(SGM::TestSurface(rResult,SurfID,SGM::Point2D(6,1)));
    SGM::Point2D uv(0,0);
    SGM::SurfaceInverse(rResult,SurfID,SGM::Point3D(0,0,0),nullptr,&uv);
    
    SGM::SaveSGM(rResult,"GTest_extrude_test.sgm",SGM::Thing(),SGM::TranslatorOptions());

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,SurfID).m_ID);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    EXPECT_TRUE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,SurfID);
    EXPECT_FALSE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGM::CopyEntity(rResult,CurveID);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGM::DeleteEntity(rResult,SurfID);
    SGM::DeleteEntity(rResult,CurveID);

    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, revolve_hermite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{10,0,0}};
    std::vector<SGM::Vector3D> aVectors = {{1,1,0},{1,1,0}};
    std::vector<double> aParams = {0,12};
    SGM::Curve CurveID=SGM::CreateHermiteCurve(rResult,aPoints,aVectors,aParams);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,6));

    double dGuess=0;
    SGM::CurveInverse(rResult,CurveID,SGM::Point3D(0,0,0),nullptr,&dGuess);

    SGM::SaveSGM(rResult,"GTest_hermite_test.sgm",CurveID,SGM::TranslatorOptions());

    SGM::GetHermiteCurveData(rResult,CurveID,aPoints,aVectors,aParams);

    SGM::UnitVector3D Axis(0,0,1);
    /*SGM::Surface SurfID=*/SGM::CreateExtrudeSurface(rResult,Axis,CurveID);
    /*SGM::Surface SurfIDCopy=*/SGM::CreateExtrudeSurface(rResult,Axis,CurveID);
}

TEST(math_check, DISABLED_closest_point)
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

    // Public interface tests

    SGM::GetFaceTriangles(rResult,FaceID1);
    SGM::GetFacePoints3D(rResult,FaceID1);
    SGM::GetFaceNormals(rResult,FaceID1);
    SGM::FindBody(rResult,FaceID1);
    SGM::GetType(rResult,FaceID1);
    std::set<SGM::Entity> sOwners;
    SGM::GetOwners(rResult,FaceID1,sOwners);
    SGM::GetFileType("test.stp");
    SGM::GetSidesOfFace(rResult,FaceID1);
    SGM::IsFaceFlipped(rResult,FaceID1);

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

    SGM::GetComplexPoints(rResult,RectangleID);
    SGM::GetComplexSegments(rResult,RectangleID);

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

    SGM::GetComplexTriangles(rResult,RectangleID);
    SGM::MergePoints(rResult,RectangleID,SGM_MIN_TOL);
    SGM::FindTriangleAreas(rResult,RectangleID);

    SGM::FindDegenerateTriangles(rResult,FilledID);
    SGM::GetBoundingBox(rResult,FilledID);
    SGM::GetBoundingBox(rResult,RectangleID);

    SGM::Body IDBox=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Complex IDComplexBox=SGM::CreateComplex(rResult,IDBox);
    SGM::FindSharpEdges(rResult,SGM::MergePoints(rResult,IDComplexBox,SGM_MIN_TOL),0.1);

    std::vector<SGM::Point3D> aPoints1,aPoints2,aPoints3;
    std::vector<SGM::IntersectionType> aTypes1,aTypes2;
    std::vector<SGM::Entity> aEntities1,aEntities2;
    SGM::Point3D Root(5,5,5);
    SGM::UnitVector3D Axis(0,0,1);
    SGM::RayFire(rResult,Root,Axis,IDComplexBox,aPoints1,aTypes1,aEntities1);
    SGM::RayFire(rResult,Root,Axis,IDComplexBox,aPoints2,aTypes2,aEntities2,SGM_MIN_TOL,true);
    EXPECT_EQ(aPoints1.size(),1U);
    EXPECT_EQ(aPoints2.size(),2U);

    SGM::Segment3D Seg(Root,SGM::Point3D(5,5,15));
    SGM::IntersectSegment(rResult,Seg,IDBox,aPoints3);
    EXPECT_EQ(aPoints3.size(),1U);
        
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

    SGM::Surface SurfID1=SGM::CreateCylinderSurface(rResult,Pos0,Pos1,1.0);
    SGM::Surface SurfID2=SGM::CreateCylinderSurface(rResult,Pos0,Pos1,1.0);
    bool bTest=SGM::SameSurface(rResult,SurfID1,SurfID2,SGM_MIN_TOL);
    EXPECT_TRUE(bTest);
    SGM::Point2D uvGuess1(0,0);
    SGM::Point2D uv1=SGM::SurfaceInverse(rResult,SurfID1,SGM::Point3D(1,0,0),nullptr,&uvGuess1);
    bool bTest1=SGM::NearEqual(uv1.m_u,0,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest1);
    SGM::Point2D uvGuess2(SGM_TWO_PI,0);
    SGM::Point2D uv2=SGM::SurfaceInverse(rResult,SurfID1,SGM::Point3D(1,0,0),nullptr,&uvGuess2);
    bool bTest2=SGM::NearEqual(uv2.m_u,SGM_TWO_PI,SGM_MIN_TOL,false);
    EXPECT_TRUE(bTest2);

    double dU=SGM::GetDomainOfSurface(rResult,SurfID1).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,SurfID1,dU);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, cone_copy_transform)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos0(0,0,0),Pos1(0,0,1);
    SGM::Body ConeID=SGM::CreateCone(rResult,Pos0,Pos1,1.0,2.0,true);
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

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,SurfID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));
    SGM::TransformEntity(rResult,Trans,SurfID);
    EXPECT_FALSE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));
    
    double dU=SGM::GetDomainOfSurface(rResult,SurfID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,SurfID,dU);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, find_least_square_plane)
{
    // points all in a x=constant plane
    std::vector<SGM::Point3D> aPointsX =
        {
            {0.0, -2.0, 0.0},
            {0.0, -1.0, 0.5},
            {0.0,  0.0, 0.25},
            {0.0,  1.0, 0.1},
            {0.0,  2.0, 0.5}
        };
    SGM::Point3D Origin;
    SGM::UnitVector3D XVec;
    SGM::UnitVector3D YVec;
    SGM::UnitVector3D ZVec;
    bool bFound = FindLeastSquarePlane(aPointsX, Origin, XVec, YVec, ZVec);

    EXPECT_TRUE(bFound);
    EXPECT_TRUE(SGM::NearEqual(fabs(ZVec%SGM::UnitVector3D(1,0,0)), 1.0, SGM_ZERO, false));

    // points all in a y=constant plane
    std::vector<SGM::Point3D> aPointsY =
        {
            {-2.0, 0.0, 0.0},
            {-1.0, 0.0, 0.5},
            { 0.0, 0.0, 0.25},
            { 1.0, 0.0, 0.1},
            { 2.0, 0.0, 0.5}
        };
    bFound = FindLeastSquarePlane(aPointsY, Origin, XVec, YVec, ZVec);

    EXPECT_TRUE(bFound);
    EXPECT_TRUE(SGM::NearEqual(fabs(ZVec%SGM::UnitVector3D(0,1,0)), 1.0, SGM_ZERO, false));

    // points all in a z=constant plane
    std::vector<SGM::Point3D> aPointsZ =
        {
            {-2.0, 0.0,  0.0},
            {-1.0, 0.5,  0.0},
            { 0.0, 0.25, 0.0},
            { 1.0, 0.1,  0.0},
            { 2.0, 0.5,  0.0}
        };
    bFound = FindLeastSquarePlane(aPointsZ, Origin, XVec, YVec, ZVec);

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

    // 2*(x-1)(x-2)(x-3)(x-4) -> 2*x^4-20*x^3+70*x^2-100*x+48 Four roots

    std::vector<double> aRoots;
    size_t nRoots=SGM::Quartic(2,-20,70,-100,48,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,4U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[2]-3),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[3]-4),SGM_ZERO);

    // (x-1)(x-2)(x-3)(x-3) -> x^4-9*x^3+29*x^2-39*x+18 Three roots, one double

    aRoots.clear();
    nRoots=SGM::Quartic(1,-9,29,-39,18,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,3U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[2]-3),SGM_ZERO);

    // (x-1)(x-2)(x-2)(x-2) -> x^4-7*x^3+18*x^2-20*x+8 Two roots, one triple

    aRoots.clear();
    nRoots=SGM::Quartic(1,-7,18,-20,8,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,2U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);

    // (x-1)(x-1)(x-2)(x-2) -> x^4-6*x^3+13*x^2-12*x+4 Two double roots

    aRoots.clear();
    nRoots=SGM::Quartic(1,-6,13,-12,4,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,2U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);

    // (x-1)(x-2)(x^2+1) -> x^4-3*x^3+3*x^2-3*x+2 Two roots

    aRoots.clear();
    nRoots=SGM::Quartic(1,-3,3,-3,2,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,2U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);

    // (x-1)(x-1)(x^2+1) -> x^4-2*x^3+2*x^2-2*x+1 One double root.

    aRoots.clear();
    nRoots=SGM::Quartic(1,-2,2,-2,1,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,1U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);

    // (x-1)(x-1)(x-1)(x-1) -> x^4-4*x^3+6*x^2-4*x+1 One quadruple root.

    aRoots.clear();
    nRoots=SGM::Quartic(1,-4,6,-4,1,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,1U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);

    // (x^2+1)(x^2+1) -> x^4+2*x^2+1 No roots.

    aRoots.clear();
    nRoots=SGM::Quartic(1,0,2,0,1,aRoots,SGM_MIN_TOL);
    EXPECT_EQ(nRoots,0U);
    }

TEST(math_check, cubic_equation)
    {
    // Test the cubic equation.

    // 2*(x-1)*(x-2)*(x-3)=0 -> 2*x^3-12*x^2+22*x-12=0 Three roots

    std::vector<double> aRoots;
    size_t nRoots=SGM::Cubic(2,-12,22,-12,aRoots);
    EXPECT_EQ(nRoots,3U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[2]-3),SGM_ZERO);

    // (x-1)*(x-2)*(x-2)=0 -> x^3-5*x^2+8*x-4=0 Two roots, one double

    aRoots.clear();
    nRoots=SGM::Cubic(1,-5,8,-4,aRoots);
    EXPECT_EQ(nRoots,2U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);

    // (x-1)*(x^2+1)=0 -> x^3-x^2+x-1=0 One root

    aRoots.clear();
    nRoots=SGM::Cubic(1,-1,1,-1,aRoots);
    EXPECT_EQ(nRoots,1U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);

    // (x-1)*(x-1)*(x-1)=0 -> x^3-x^2+x-1=0 One triple root

    aRoots.clear();
    nRoots=SGM::Cubic(1,-3,3,-1,aRoots);
    EXPECT_EQ(nRoots,1U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);

    // (x-1)*(x-2)=0 -> x^2-3*x+2=0 Two roots and degenerate

    aRoots.clear();
    nRoots=SGM::Cubic(0,1,-3,2,aRoots);
    EXPECT_EQ(nRoots,2U);
    EXPECT_LT(fabs(aRoots[0]-1),SGM_ZERO);
    EXPECT_LT(fabs(aRoots[1]-2),SGM_ZERO);
    }

TEST(surface_check, plane)
    {
    // Test plane inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D XAxis(1,2,3),ZAxis;
    SGM::UnitVector3D YAxis=XAxis.Orthogonal();
    SGM::Surface PlaneID=SGM::CreatePlane(rResult,Origin,Origin+XAxis,Origin+YAxis);

    SGM::GetPlaneData(rResult,PlaneID,Origin,XAxis,YAxis,ZAxis);  
    EXPECT_TRUE(SGM::TestSurface(rResult,PlaneID,SGM::Point2D(0.5,0.2)));
    SGM::DeleteEntity(rResult,PlaneID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(surface_check, sphere)
    {
    // Test sphere inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D XAxis(1,2,3),ZAxis;
    SGM::UnitVector3D YAxis=XAxis.Orthogonal();
    double dRadius=2.5;
    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,Origin,dRadius,&XAxis,&YAxis);

    SGM::Point2D GuessUV(1,2);
    SGM::SurfaceInverse(rResult,SphereID,Origin,nullptr,&GuessUV);

    SGM::GetSphereData(rResult,SphereID,Origin,XAxis,YAxis,ZAxis,dRadius);

    double dU=SGM::GetDomainOfSurface(rResult,SphereID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,SphereID,dU);
    EXPECT_TRUE(SGM::TestSurface(rResult,SphereID,SGM::Point2D(0.5,0.2)));

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,SphereID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,SphereID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,SphereID);
    EXPECT_FALSE(SGM::SameSurface(rResult,SphereID,SurfID2,SGM_MIN_TOL));
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGM::DeleteEntity(rResult,SphereID);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(surface_check, cylinder)
    {
    // Test cylinder inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(10,11,12),Top(13,14,15);
    double dRadius=2.5;
    SGM::Surface SphereID=SGM::CreateCylinderSurface(rResult,Bottom,Top,dRadius);

    SGM::Point3D Origin;
    SGM::UnitVector3D XAxis,YAxis,ZAxis;
    SGM::GetCylinderData(rResult,SphereID,Origin,XAxis,YAxis,ZAxis,dRadius);

    EXPECT_TRUE(SGM::TestSurface(rResult,SphereID,SGM::Point2D(0.5,0.2)));
    SGM::DeleteEntity(rResult,SphereID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(surface_check, torus)
    {
    // Test torus inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(0,0,0);
    SGM::UnitVector3D ZAxis(0,0,1),XAxis,YAxis;
    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,Origin,ZAxis,2,5,true);

    SGM::TorusKindType nKind;
    double dMinorRadius,dMajorRadius;
    SGM::GetTorusData(rResult,TorusID,Origin,XAxis,YAxis,ZAxis,dMinorRadius,dMajorRadius,nKind);

    EXPECT_TRUE(SGM::TestSurface(rResult,TorusID,SGM::Point2D(0.5,0.2)));

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,TorusID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,TorusID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,TorusID);
    EXPECT_FALSE(SGM::SameSurface(rResult,TorusID,SurfID2,SGM_MIN_TOL));
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));
    
    double dU=SGM::GetDomainOfSurface(rResult,TorusID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,TorusID,dU);

    double dV=SGM::GetDomainOfSurface(rResult,TorusID).m_VDomain.MidPoint();
    SGM::FindVParamCurve(rResult,TorusID,dV);

    SGM::DeleteEntity(rResult,TorusID);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(surface_check, cone)
    {
    // Test cone inverse.
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(10,11,12);
    SGM::UnitVector3D ZAxis(1,2,3);
    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,Origin,ZAxis,2,0.4);

    SGM::UnitVector3D XAxis,YAxis;
    double dHalfAngle,dRadius;
    SGM::Point3D Apex;
    SGM::GetConeData(rResult,ConeID,Origin,XAxis,YAxis,ZAxis,dHalfAngle,dRadius,Apex);

    SGM::UnitVector3D Vec1,Vec2;
    double dk1,dk2;
    SGM::Point2D uv=SGM::SurfaceInverse(rResult,ConeID,Apex);
    SGM::PrincipleCurvature(rResult,ConeID,uv,Vec1,Vec2,dk1,dk2);

    EXPECT_TRUE(SGM::TestSurface(rResult,ConeID,SGM::Point2D(0.5,0.2)));
    SGM::DeleteEntity(rResult,ConeID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(curve_check, NUB)
    {
    // Test NUB Curve inverse.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    std::vector<double> aKnots={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints = 
        {
            {1,1,0},
            {1.166666666666666,1.166666666666666,0},
            {2,2.8333333333333333,0},
            {2.8333333333333333,1.166666666666666,0},
            {3,1,0}
        };

    SGM::Curve NUBID=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints,aKnots);

    EXPECT_TRUE(SGM::TestCurve(rResult,NUBID,0.45));
    SGM::DeleteEntity(rResult,NUBID);
    SGMTesting::ReleaseTestThing(pThing);
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

    EXPECT_TRUE(SGM::TestCurve(rResult,LineID,0.5));
    SGM::DeleteEntity(rResult,LineID);
    SGM::DeleteEntity(rResult,CopyID);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));
    SGMTesting::ReleaseTestThing(pThing);
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

    EXPECT_TRUE(SGM::TestCurve(rResult,CircleID,0.45));
    SGM::DeleteEntity(rResult,CircleID);
    SGM::DeleteEntity(rResult,CopyID);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(surface_check, principle_curvature)
    {
    // Test Principle Curvature

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0.0,0.0,0.0);
    SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,Center,ZAxis,2,5,true);
    SGM::Point2D uv = {0.0,0.0};
    SGM::UnitVector3D Vec1,Vec2;
    double k1,k2;
    SGM::PrincipleCurvature(rResult,TorusID,uv,Vec1,Vec2,k1,k2);

    EXPECT_TRUE(SGM::NearEqual(Vec1,SGM::UnitVector3D(0.0,1.0,0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(Vec2,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO));

    EXPECT_TRUE(SGM::NearEqual(k1,-7.0,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(k2,-2.0,SGM_ZERO,false));

    SGM::DeleteEntity(rResult,TorusID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, triangulate_overlapping_polygon)
    {
    // Tests to see if an overlapping triangle can be triangulated.

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point2D> aPoints = {{0,0},{1,1},{0,1},{1,0}};
    std::vector<unsigned> aPolygon = {0,1,2,3};
    std::vector<unsigned> aTriangles;
    EXPECT_TRUE(SGM::TriangulatePolygon(rResult,aPoints,aPolygon,aTriangles));
    EXPECT_EQ(aTriangles.size(),6U);
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, min_cycles_odd)
    {
    // Tests to find min cycles with an odd number of edges.

    std::set<size_t> sVertices = {0,1,2,3,4,5,6,7,8};
    std::set<SGM::GraphEdge> sEdges = 
        {
            {0,1,0},
            {0,2,1},
            {3,1,2},
            {2,4,3},
            {3,5,4},
            {4,6,5},
            {5,6,6},
            {5,7,7},
            {6,8,8},
            {7,8,9}
        };

    SGM::Graph graph(sVertices,sEdges);

    SGM::GraphEdge GE(0,1,0);
    SGM::Graph* pGLoop= graph.CreateMinCycle(GE);
    EXPECT_EQ(pGLoop->GetVertices().size(),7U);
    EXPECT_EQ(pGLoop->GetEdges().size(),7U);
    delete pGLoop;
    }

TEST(math_check, min_cycles_even)
    {
    // Tests to find min cycles with an even number of edges.

    std::set<size_t> sVertices {0,1,2,3,4,5,6};
    std::set<SGM::GraphEdge> sEdges = 
        {
            {0,1,0},
            {0,2,1},
            {3,1,2},
            {2,4,3},
            {3,5,4},
            {4,5,5},
            {4,6,6},
            {6,6,7}
        };
    SGM::Graph graph(sVertices,sEdges);

    SGM::GraphEdge GE(0,1,0);
    SGM::Graph *pGLoop=graph.CreateMinCycle(GE);
    EXPECT_EQ(pGLoop->GetVertices().size(),6U);
    EXPECT_EQ(pGLoop->GetEdges().size(),6U);

    delete pGLoop;
    }

TEST(math_check, triangulate_polygon_with_holes)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point2D> aPoints2D = {{0,0},{1,0},{0,1},{0.25,0.25}};
    std::vector<std::vector<unsigned>> aaPolygons = {{0,1,2}, {3}};
    std::vector<unsigned> aTriangles,aAdjacencies;
    SGM::TriangulatePolygonWithHoles(rResult,aPoints2D,aaPolygons,aTriangles,aAdjacencies);
    EXPECT_EQ(aTriangles.size(),9U);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, triangulate_polygon)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Testing polygon triangulation.

    std::vector<SGM::Point2D> aPoints2D = {
        {2 , 0},
        {1 , 0},
        {0 ,-1},
        {-1, 0},
        {0 , 1},
        {1 , 0},
        {2 , 0},
        {0 , 2},
        {-2, 0},
        {0 ,-2}
    };
    std::vector<SGM::Point3D> aPoints3D = {
        {2 , 0, 0},
        {1 , 0, 0},
        {0 ,-1, 0},
        {-1, 0, 0},
        {0 , 1, 0},
        {1 , 0, 0},
        {2 , 0, 0},
        {0 , 2, 0},
        {-2, 0, 0},
        {0 ,-2, 0}
    };
    std::vector<unsigned> aPolygon={0,1,2,3,4,5,6,7,8,9};
    std::vector<unsigned> aTriangles,aSegments;
    SGM::TriangulatePolygon(rResult,aPoints2D,aPolygon,aTriangles);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);
 
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, octahedron)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned> aSegments,aTriangles;
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
    std::vector<unsigned> aSegments,aTriangles;
    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D ZAxis(0,0,1),XAxis(1,0,0);
    SGM::CreateIcosahedron(1.0,Center,ZAxis,XAxis,aPoints3D,aTriangles,3);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_creation_and_point_removal)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test grid creation and point imprinting.

    std::vector<double> aValues = {0,1,2,3};
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<unsigned> aTriangles,aSegments;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point3D> aGridPoints3D;
    aGridPoints3D.reserve(aPoints2D.size());
    for(auto Pos : aPoints2D)
        {
        aGridPoints3D.emplace_back(Pos.m_u,Pos.m_v,0.0);
        }
    
    // Point (1,1,0) is at index 5.
    
    std::vector<unsigned> aRemovedOrChanged,aReplacedTriangles;
    SGM::RemovePointFromTriangles(rResult,5,aPoints2D,aTriangles,aRemovedOrChanged,aReplacedTriangles);
    SGM::CreateComplex(rResult,aGridPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_creation_and_imprinting_polygon)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aValues = {0,1,2,3};
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<unsigned> aTriangles;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point2D> aPolyPoints2D = {{0,0},{3,0},{3,3}};
    std::vector<SGM::Point3D> aPolyPoints3D = {{0,0,0},{3,0,0},{3,3,0}};

    std::vector<unsigned> aSegments = {0,1,1,2,2,0};

    std::vector<unsigned> aPolygonIndices;
    SGM::InsertPolygon(rResult,aPolyPoints2D,aPoints2D,aTriangles,aPolygonIndices);
    std::vector<SGM::Point3D> aPoints3D;
    aPoints3D.reserve(aPoints2D.size());
    for(auto Pos : aPoints2D)
        {
        aPoints3D.emplace_back(Pos.m_u,Pos.m_v,0.0);
        }
    aSegments.clear();
    SGM::ReduceToUsedPoints(aPoints2D,aTriangles,&aPoints3D,nullptr);
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, grid_creation_and_imprinting_circle)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aValues = {0,1,2,3};
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<unsigned> aTriangles;

    SGM::CreateTrianglesFromGrid(aValues,aValues,aPoints2D,aTriangles);

    std::vector<SGM::Point3D> aGridPoints3D;
    aGridPoints3D.reserve(aPoints2D.size());
    for(auto Pos : aPoints2D)
        {
        aGridPoints3D.emplace_back(Pos.m_u,Pos.m_v,0.0);
        }

    std::vector<SGM::Point2D> aPolyPoints2D;
    std::vector<SGM::Point3D> aPolyPoints3D;
    std::vector<unsigned> aSegments;
    SGM::Interval1D Domain(0,SGM_TWO_PI);
    size_t Index1,nPoints=12;
    double dRadius=1.5;
    aPolyPoints2D.reserve(nPoints);
    aPolyPoints3D.reserve(nPoints);
    aSegments.reserve(2*nPoints);
    for(Index1=0;Index1<nPoints;++Index1)
        {
        double t=Domain.MidPoint(Index1/(double)nPoints);
        double x=cos(t)*dRadius+1.5;
        double y=sin(t)*dRadius+1.5;
        aPolyPoints2D.emplace_back(x,y);
        aPolyPoints3D.emplace_back(x,y,0);
        aSegments.push_back((unsigned)Index1);
        aSegments.push_back((unsigned)((Index1+1)%nPoints));
        }

    std::vector<unsigned> aPolygonIndices;
    SGM::InsertPolygon(rResult,aPolyPoints2D,aPoints2D,aTriangles,aPolygonIndices);
    std::vector<SGM::Point3D> aPoints3D;
    aPoints3D.reserve(aPoints2D.size());
    for(auto Pos : aPoints2D)
        {
        aPoints3D.emplace_back(Pos.m_u,Pos.m_v,0.0);
        }
    SGM::CreateComplex(rResult,aPoints3D,aSegments,aTriangles);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, splitting_complex_at_points)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{10,0,0},{10,10,0}};
    std::vector<unsigned> aSegments = {0,1,1,2};
    std::vector<unsigned> aTriangles;
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,aPoints,aSegments,aTriangles);
    
    std::vector<SGM::Point3D> aNewPoints = {{2,0,0},{7,0,0},{10,5,0}};
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

    std::vector<SGM::Point3D> aNUBPoints = {{-2,.5,0},{-1,1.5,0},{0,1,0},{1,1.5,0},{2,2,0}};
    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aNUBPoints);

    SGM::Point3D AxisOrigin(-1,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Surface RevolveID = SGM::CreateRevolveSurface(rResult, AxisOrigin, Axis, CurveID);

    SGM::Point2D GuessUV(0,0);
    SGM::SurfaceInverse(rResult,RevolveID,SGM::Point3D(-2,0.5,0),nullptr,&GuessUV);

    SGM::GetRevolveData(rResult,RevolveID,AxisOrigin,Axis,CurveID);

    double dU=SGM::GetDomainOfSurface(rResult,RevolveID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,RevolveID,dU);

    double dV=SGM::GetDomainOfSurface(rResult,RevolveID).m_VDomain.MidPoint();
    SGM::FindVParamCurve(rResult,RevolveID,dV);

    SGM::Point3D LineOrigin1(1,0,0);
    SGM::UnitVector3D uDirection1(0,0,1);
    SGM::Curve Line1ID = SGM::CreateLine(rResult, LineOrigin1, uDirection1);

    SGM::Point3D Origin;
    SGM::GetLineData(rResult,Line1ID,Origin,Axis);

    std::vector<SGM::Point3D> aPoints1;
    std::vector<SGM::IntersectionType> aTypes1;
    double dTolerance = SGM_MIN_TOL;
    SGM::IntersectCurveAndSurface(rResult, Line1ID, RevolveID, aPoints1, aTypes1, dTolerance);

    EXPECT_EQ(aPoints1.size(),2U);

    for (SGM::IntersectionType IType : aTypes1 )
        EXPECT_EQ(IType,SGM::PointType);

    std::vector<SGM::Point3D> aExpected1 = {{1,0,1.5},{1,0,-1.5}};
    int found1=0;
    for (SGM::Point3D PosExpected : aExpected1)
        {
        for (SGM::Point3D PosComputed : aPoints1)
            if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                found1++;
        }
    EXPECT_EQ(found1,2);
    
    SGM::Point3D LineOrigin2(1,1,-1.2);
    SGM::UnitVector3D uDirection2(0,-1,0);
    SGM::Curve Line2ID = SGM::CreateLine(rResult, LineOrigin2, uDirection2);

    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    SGM::IntersectCurveAndSurface(rResult, Line2ID, RevolveID, aPoints2, aTypes2, dTolerance);

    EXPECT_EQ(aPoints2.size(),2U);

    for (SGM::IntersectionType IType : aTypes2)
        EXPECT_EQ(IType,SGM::PointType);

    std::vector<SGM::Point3D> aExpected2 = {{1,0.9,-1.2},{1,-0.9,-1.2}};

    int found2=0;
    for (SGM::Point3D PosExpected : aExpected2)
        {
        for (SGM::Point3D PosComputed : aPoints2)
            if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                found2++;
        }
    EXPECT_EQ(found2,2);
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, intersect_nubcurve_and_plane)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 =
        {
            {-2,.5,0},
            {-1,1.5,0},
            {0,1,0},
            {1,1.5,0},
            {2,2,0}
        };

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    SGM::Point3D Origin(1,0,0);
    SGM::Point3D XPos(1,0,-1);
    SGM::Point3D YPos(1,1,0);

    SGM::Surface PlaneID = SGM::CreatePlane(rResult, Origin, XPos, YPos);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    double dTolerance = SGM_MIN_TOL;
    SGM::IntersectCurveAndSurface(rResult, CurveID, PlaneID, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(),1U);
    SGM::Point3D Expected(1, 1.5, 0);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0], Expected, dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, point_in_body)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Point in Body

    SGM::Point3D Bottom(0,0,0),Top(0,0,2);
    double dRadius=1.0;
    SGM::Body BodyID=SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    EXPECT_FALSE(SGM::PointInEntity(rResult,{-2,0,1},BodyID));
    EXPECT_TRUE(SGM::PointInEntity(rResult,{0,0,1},BodyID));

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, ray_fire)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(0,0,0),Top(0,0,2);
    double dRadius=1.0;
    SGM::Body BodyID=SGM::CreateCylinder(rResult,Bottom,Top,dRadius);
    SGM::Point3D Origin(-2,0,1);
    SGM::UnitVector3D Axis(1,0,0);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<SGM::Entity> aEntites;
    SGM::RayFire(rResult,Origin,Axis,BodyID,aPoints,aTypes,aEntites,SGM_MIN_TOL);

    EXPECT_EQ(aPoints.size(),2U);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0],SGM::Point3D(-1,0,1),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[1],SGM::Point3D(1,0,1),SGM_ZERO));
    EXPECT_EQ(aTypes[0],SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1],SGM::IntersectionType::PointType);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NURB_surface)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test of NURB Surface.

    std::vector<std::vector<SGM::Point4D>> aaControlPoints =
        {{{ 1,  0, 0, 1},
          { 1,  1, 0, SGM_SQRT_2 / 2},
          { 0,  1, 0, 1},
          {-1,  1, 0, SGM_SQRT_2 / 2},
          {-1,  0, 0, 1},
          {-1, -1, 0, SGM_SQRT_2 / 2},
          { 0, -1, 0, 1},
          { 1, -1, 0, SGM_SQRT_2 / 2},
          { 1,  0, 0, 1}
         },
         {{ 1,  0, 1, 1},
          { 1,  1, 1, SGM_SQRT_2 / 2},
          { 0,  1, 1, 1},
          {-1,  1, 1, SGM_SQRT_2 / 2},
          {-1,  0, 1, 1},
          {-1, -1, 1, SGM_SQRT_2 / 2},
          { 0, -1, 1, 1},
          { 1, -1, 1, SGM_SQRT_2 / 2},
          { 1,  0, 1, 1}
        }};
    
    std::vector<double> aUKnots = {0,0,1,1};

    std::vector<double> aVKnots =
        {0,0,0,SGM_HALF_PI,SGM_HALF_PI,SGM_PI,SGM_PI,SGM_PI*1.5,SGM_PI*1.5,SGM_TWO_PI,SGM_TWO_PI,SGM_TWO_PI};

    SGM::Surface SurfID=SGM::CreateNURBSurface(rResult,aaControlPoints,aUKnots,aVKnots);
    EXPECT_TRUE(SGM::TestSurface(rResult,SurfID,SGM::Point2D(0.245,0.678)));

    // Code to test saving a NURB surface
    // turn on FindUMultiplicity and FindVMultiplicity
    //
    // std::vector<SGM::Edge> aEdges;
    // std::vector<SGM::EdgeSideType> aTypes;
    // SGM::CreateFaceFromSurface(rResult,SurfID,aEdges,aTypes);
    // SGM::SaveSTEP(rResult,"Gtest_NURB_Surface_Test.stp",SGM::Thing(),SGM::TranslatorOptions());

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

    EXPECT_NEAR(dDist0,1.0,SGM_ZERO);
    EXPECT_NEAR(dDist1,1.0,SGM_ZERO);
    EXPECT_NEAR(dDist2,1.0,SGM_ZERO);

    SGM::SaveSGM(rResult,"CoverageTest.sgm",SGM::Thing(),SGM::TranslatorOptions());

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,SurfID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,SurfID);
    EXPECT_FALSE(SGM::SameSurface(rResult,SurfID,SurfID2,SGM_MIN_TOL));
    
    double dU=SGM::GetDomainOfSurface(rResult,SurfID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,SurfID,dU);

    double dV=SGM::GetDomainOfSurface(rResult,SurfID).m_VDomain.MidPoint();
    SGM::FindVParamCurve(rResult,SurfID,dV);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NURB_curve_tangent)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints =
        {{1,0,0,1},
         {1,1,0,SGM_SQRT_2/2},
         {1,1,-1,1},
         {1,1,-2,SGM_SQRT_2/2},
         {1,0,-2,1},
         {1,-1,-2,SGM_SQRT_2/2},
         {1,-1,-3,1}
        };
    std::vector<double> aKnots = 
        {0,0,0,SGM_HALF_PI,SGM_HALF_PI,SGM_PI,SGM_PI,SGM_PI*1.5,SGM_PI*1.5,SGM_PI*1.5};

    SGM::Curve NURBcurve = SGM::CreateNURBCurve(rResult, aControlPoints, aKnots);

    SGM::Point3D Pos;
    SGM::Vector3D Vec;
    SGM::EvaluateCurve(rResult,NURBcurve,SGM_PI*0.25,&Pos,&Vec);

    SGM::Point3D ExpectedPos(1,SGM_SQRT_2/2,-1+SGM_SQRT_2/2);
    EXPECT_NEAR((ExpectedPos-Pos).Magnitude(),0,SGM_MIN_TOL);

    SGM::Vector3D ExpectedVec(0,1/SGM_SQRT_2,-1/SGM_SQRT_2);
    EXPECT_NEAR((SGM::UnitVector3D(Vec)-ExpectedVec).Magnitude(),0,SGM_MIN_TOL);
}

TEST(math_check, NURB_curve)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test of NURB Curves.
    
    std::vector<SGM::Point4D> aControlPoints = 
        {{ 1, 0, 0, 1},
         { 1, 1, 0, SGM_SQRT_2/2},
         { 0, 1, 0, 1},
         {-1, 1, 0, SGM_SQRT_2/2},
         {-1, 0, 0, 1},
         {-1,-1, 0, SGM_SQRT_2/2},
         { 0,-1, 0, 1},
         { 1,-1, 0, SGM_SQRT_2/2},
         { 1, 0, 0, 1}
        };

    std::vector<double> aKnots =
        {0, 0, 0, SGM_HALF_PI, SGM_HALF_PI, SGM_PI, SGM_PI, SGM_PI*1.5, SGM_PI*1.5, SGM_TWO_PI, SGM_TWO_PI, SGM_TWO_PI};


    SGM::Curve CurveID=SGM::CreateNURBCurve(rResult,aControlPoints,aKnots);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,1.234));

    SGM::GetNURBCurveData(rResult,CurveID,aControlPoints,aKnots);

    SGM::Point3D Pos0,Pos1,Pos2;
    SGM::EvaluateCurve(rResult,CurveID,1,&Pos0);
    SGM::EvaluateCurve(rResult,CurveID,2,&Pos1);
    SGM::EvaluateCurve(rResult,CurveID,3,&Pos2);
    double dDist0=(Pos0-SGM::Point3D(0,0,0)).Magnitude();
    double dDist1=(Pos1-SGM::Point3D(0,0,0)).Magnitude();
    double dDist2=(Pos2-SGM::Point3D(0,0,0)).Magnitude();

    EXPECT_NEAR(dDist0,1.0,SGM_ZERO);
    EXPECT_NEAR(dDist1,1.0,SGM_ZERO);
    EXPECT_NEAR(dDist2,1.0,SGM_ZERO);

    SGM::SaveSGM(rResult,"CoverageTest.sgm",SGM::Thing(),SGM::TranslatorOptions());

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, torus_area)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(0,0,1);
    SGM::Body BodyID=SGM::CreateTorus(rResult,Center,Norm,1,3);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    double dArea=SGM::FindArea(rResult,FaceID);
    EXPECT_TRUE(SGM::NearEqual(dArea,118.43525281307230342601389199851,SGM_MIN_TOL,false));

    SGM::DeleteEntity(rResult,BodyID);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, DISABLED_sphere_area)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Norm(1,1,1),XAxis,YAxis;
    SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Norm,1.0);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Surface SurfaceID=SGM::CreateSphereSurface(rResult,Center,1.0);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(EdgeID);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
    SGM::Body BodyID=SGM::CreateSheetBody(rResult,SurfaceID,aEdges,aTypes);

    // Public interface tests.

    SGM::GetEdgePoints(rResult,EdgeID);
    SGM::GetToleranceOfEdge(rResult,EdgeID);
    SGM::GetDomainOfEdge(rResult,EdgeID);

    double dRadius;
    SGM::GetCircleData(rResult,CurveID,Center,Norm,XAxis,YAxis,dRadius);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    //std::cout << "sphere_area" << std::endl;
    double dArea=SGM::FindArea(rResult,FaceID);
    EXPECT_TRUE(SGM::NearEqual(dArea,6.283185307179586476925286766559,SGM_MIN_TOL,true));
    SGM::DeleteEntity(rResult,BodyID);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, line_nub_curve_intersect)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,1,0},{2,0,0}};
    size_t Index1;
    SGM::Curve NUBCurveID=SGM::CreateNUBCurve(rResult,aPoints);

    // Test with two hits.

    SGM::Point3D Pos0(0,0.5,0),Pos1(2,0.5,0);
    SGM::Curve LineID1=SGM::CreateLine(rResult,Pos0,Pos1-Pos0);
    std::vector<SGM::Point3D> aHits1;
    std::vector<SGM::IntersectionType> aTypes1;
    SGM::IntersectCurves(rResult,LineID1,NUBCurveID,aHits1,aTypes1);

    size_t nHits1=aHits1.size();
    EXPECT_EQ(nHits1,2U);
    for(Index1=0;Index1<nHits1;++Index1)
        {
        SGM::Point3D const &Pos=aHits1[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID1,Pos,&CPos1);
        SGM::CurveInverse(rResult,NUBCurveID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        EXPECT_LT(dDist,SGM_ZERO);
        }
    SGM::DeleteEntity(rResult,LineID1);

    // Test with one tangent hit.

    SGM::Point3D Pos2(0,1,0),Pos3(2,1,0);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);
    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    SGM::IntersectCurves(rResult,LineID2,NUBCurveID,aHits2,aTypes2);

    size_t nHits2=aHits2.size();
    EXPECT_EQ(nHits2,1U);
    EXPECT_EQ(aTypes2[0],SGM::IntersectionType::TangentType);
    for(Index1=0;Index1<nHits2;++Index1)
        {
        SGM::Point3D const &Pos=aHits2[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::CurveInverse(rResult,NUBCurveID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        EXPECT_LT(dDist,SGM_ZERO);
        }
    SGM::DeleteEntity(rResult,LineID2);
    SGM::DeleteEntity(rResult,NUBCurveID);

    SGMTesting::ReleaseTestThing(pThing);
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

    auto aComplexes = (std::vector<SGM::Complex> *) &entities;
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

    auto aComplexes = (std::vector<SGM::Complex> *) &entities;
    SGM::Complex ComplexID = aComplexes->front();
    if (aComplexes->size() > 1)
        {
        ComplexID = SGM::MergeComplexes(rResult, *aComplexes);
        }
    std::vector<SGM::Complex> aHoles;
    SGM::FindHoles(rResult,ComplexID,aHoles);
    SGM::DeleteEntity(rResult,ComplexID); 

    std::string OutputSGMLFile("STL Files/SNL-2024-T3-IMP1_Output.stl");
    SGM::TranslatorOptions Options;
    SGM::SaveSGM(rResult, OutputSGMLFile, SGM::Thing() , Options);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, DISABLED_unite_spheres) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center1(0,0,0),Center2(1.5,0,0);
    SGM::Body SphereID1=SGM::CreateSphere(rResult,Center1,1.0);
    SGM::Body SphereID2=SGM::CreateSphere(rResult,Center2,1.0);
    SGM::UniteBodies(rResult,SphereID1,SphereID2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, NUB_curve_inverse) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {
            {-2,  0,   0},
            {-1,  0.1, 0},
            {0.0, 0.0, 0},
            {1,   0.1, 0},
            {2,   0,   0}
    };

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    size_t Index1;
    for(Index1=0;Index1<5;++Index1)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,aPoints[Index1],&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(aPoints[Index1],ClosePos,SGM_MIN_TOL));
        }
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, banded_matrix_solve) 
    {
    // if x=1,y=2,z=3,w=4,v=5,r=6 then
    //
    // 1x+2y+3z+0w+0v+0r=14
    // 2x+1y+1z+1w+0v+0r=11
    // 1x+2y+1z+1w+0v+0r=12
    // 0x+1y+2z-1w+1v+0r=9
    // 0x+0y-1z+1w+2v+1r=17
    // 0x+0y+0z-1w-1v+2r=3

    std::vector<std::vector<double>> aaMatrix = 
        {{ 0,  0,  1,  2,  3, 14},
         { 0,  2,  1,  1,  1, 11},
         { 1,  2,  1,  1,  0, 12},
         { 1,  2, -1,  1,  0,  9},
         {-1,  1,  2,  1,  0, 17},
         {-1, -1,  2,  0,  0,  3}};

    EXPECT_TRUE(SGM::BandedSolve(aaMatrix));
    EXPECT_NEAR(aaMatrix[0].back(),1,SGM_ZERO);
    EXPECT_NEAR(aaMatrix[1].back(),2,SGM_ZERO);
    EXPECT_NEAR(aaMatrix[2].back(),3,SGM_ZERO);
    EXPECT_NEAR(aaMatrix[3].back(),4,SGM_ZERO);
    EXPECT_NEAR(aaMatrix[4].back(),5,SGM_ZERO);
    }

TEST(math_check, matrix_solve) 
    {
    // if x=1,y=2,z=3,w=4, then
    //
    // 1x+2y+0z+1w= 9
    // 2x+2y+2z+0w=12
    // 0x+2y-1z+3w=13
    // 1x+1y+2z-1w= 5

    std::vector<std::vector<double>> aaMatrix =
        {{1, 2, 0,  1,  9},
         {2, 2, 2,  0,  12},
         {0, 2, -1, 3,  13},
         {1, 1, 2,  -1, 5}};

    EXPECT_TRUE(SGM::LinearSolve(aaMatrix));
    EXPECT_TRUE(SGM::NearEqual(aaMatrix[0].back(),1,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(aaMatrix[1].back(),2,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(aaMatrix[2].back(),3,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(aaMatrix[3].back(),4,SGM_ZERO,false));
    }

TEST(math_check, NUB_curve_through_three_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to three points with end vectors.

    std::vector<SGM::Point3D> aPoints = 
        {{1,   1,   0},
         {2,   2,   0},
         {2.1, 1.9, 0},
         {3,   1,   0}};

    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    for(auto const & Point : aPoints)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,Point,&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(Point,ClosePos,SGM_MIN_TOL));
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    EXPECT_TRUE(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL));

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_points_and_end_vecs)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to four points with end vectors.

    std::vector<SGM::Point3D> aPoints = 
        {{1,1,0},
         {2,2,0},
         {3,1,0},
         {5,0,0}};
    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    for(auto const &Point : aPoints)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,Point,&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(Point,ClosePos,SGM_MIN_TOL));
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    EXPECT_TRUE(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL));
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_two_points_and_end_vecs) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    // Fitting a NUB curve to two points with end vectors.

    std::vector<SGM::Point3D> aPoints = {{1,1,0}, {3,1,0}};
    SGM::Vector3D StartVec(1,1,0),EndVec(1,-1,0);

    SGM::Curve NUBID=SGM::CreateNUBCurveWithEndVectors(rResult,aPoints,StartVec,EndVec);
    for(auto const & Point : aPoints)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,Point,&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(Point,ClosePos,SGM_MIN_TOL));
        }
    SGM::Vector3D Vec0,Vec1;
    SGM::EvaluateCurve(rResult,NUBID,0.0,nullptr,&Vec0);
    SGM::EvaluateCurve(rResult,NUBID,1.0,nullptr,&Vec1);
    EXPECT_TRUE(SGM::NearEqual(Vec0,StartVec,SGM_MIN_TOL));
    EXPECT_TRUE(SGM::NearEqual(Vec1,EndVec,SGM_MIN_TOL));
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_to_three_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints =
        {
            {1,1,0},
            {2,2,0},
            {3,1,0}
        };

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    for(auto const & Point : aPoints)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,Point,&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(Point,ClosePos,SGM_MIN_TOL));
        }
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, NUB_curve_to_two_points) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Fitting a NUB curve to two points.
    // Which requires it be degree one.

    std::vector<SGM::Point3D> aPoints = {{1,1,0},{3,1,0}};

    SGM::Curve NUBID=SGM::CreateNUBCurve(rResult,aPoints);
    for(auto const & Point : aPoints)
        {
        SGM::Point3D ClosePos;
        SGM::CurveInverse(rResult,NUBID,Point,&ClosePos);
        EXPECT_TRUE(SGM::NearEqual(Point,ClosePos,SGM_MIN_TOL));
        }
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, create_nub_surface) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test NUB surface.

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point3D>> aaPoints =
        {{{0.0,  0.0,  1.0}, {0.0,  1.0,  0.0}, {0.0,  2.0, -1.0}},
         {{1.0,  0.0,  0.0}, {1.0,  1.0,  0.0}, {1.0,  2.0,  0.0}},
         {{2.0,  0.0, -1.0}, {2.0,  1.0,  0.0}, {2.0,  2.0,  1.0}}};
    
    SGM::Surface NUBSurfID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    SGM::GetNUBSurfaceData(rResult,NUBSurfID,aaPoints,aUKnots,aVKnots);
    EXPECT_TRUE(SGM::TestSurface(rResult,NUBSurfID,SGM::Point2D(0.3,0.2)));
    
    SGM::UnitVector3D Vec1,Vec2;
    double k1,k2;
    SGM::Point2D uv(0.5,0.5);
    SGM::PrincipleCurvature(rResult,NUBSurfID,uv,Vec1,Vec2,k1,k2);

    EXPECT_TRUE(SGM::NearEqual(Vec1,SGM::UnitVector3D(1.0,-1.0,0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(Vec2,SGM::UnitVector3D(1.0,1.0,0),SGM_ZERO));
    EXPECT_NEAR(k1,-4.0,SGM_ZERO);
    EXPECT_NEAR(k2, 4.0,SGM_ZERO);

    SGM::Surface SurfID2=SGM::Surface(SGM::CopyEntity(rResult,NUBSurfID).m_ID);
    EXPECT_TRUE(SGM::SameSurface(rResult,NUBSurfID,SurfID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,NUBSurfID);
    EXPECT_FALSE(SGM::SameSurface(rResult,NUBSurfID,SurfID2,SGM_MIN_TOL));

    double dU=SGM::GetDomainOfSurface(rResult,NUBSurfID).m_UDomain.MidPoint();
    SGM::FindUParamCurve(rResult,NUBSurfID,dU);

    double dV=SGM::GetDomainOfSurface(rResult,NUBSurfID).m_VDomain.MidPoint();
    SGM::FindVParamCurve(rResult,NUBSurfID,dV);

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(math_check, line_torus_intersection) 
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // Test Line Torus Intersections.

    SGM::Point3D Origin(-20,0,0);
    SGM::UnitVector3D Axis(1.0,0.0,0.0);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;

    SGM::Point3D Center(0.0,0.0,0.0);
    SGM::UnitVector3D ZAxis(0.0,0.0,1.0);
    SGM::Surface TorusID=SGM::CreateTorusSurface(rResult,Center,ZAxis,1.0,3.0,false);
    
    SGM::Curve Line1=SGM::CreateLine(rResult,Origin,Axis);
    size_t nHits=SGM::IntersectCurveAndSurface(rResult,Line1,TorusID,aPoints,aTypes,SGM_MIN_TOL);
    EXPECT_EQ(nHits,4U);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0],SGM::Point3D(-4.0,0.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[1],SGM::Point3D(-2.0,0.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[2],SGM::Point3D( 2.0,0.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[3],SGM::Point3D( 4.0,0.0,0.0),SGM_ZERO));

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=2.0;
    SGM::Curve Line2=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line2,TorusID,aPoints,aTypes,SGM_MIN_TOL);
    EXPECT_EQ(nHits,3U);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0],SGM::Point3D(-3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[1],SGM::Point3D(0.0,2.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[2],SGM::Point3D(3.4641016151377545870548926830117,2.0,0.0),SGM_ZERO));

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=3.0;
    SGM::Curve Line3=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line3,TorusID,aPoints,aTypes,SGM_MIN_TOL);
    EXPECT_EQ(nHits,2U);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0],SGM::Point3D(-2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(aPoints[1],SGM::Point3D(2.6457513110645905905016157536393,3.0,0.0),SGM_ZERO));

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=4.0;
    SGM::Curve Line4=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line4,TorusID,aPoints,aTypes,SGM_MIN_TOL);
    EXPECT_EQ(nHits,1U);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0],SGM::Point3D(0.0,4.0,0.0),SGM_ZERO));

    aPoints.clear();
    aTypes.clear();
    Origin.m_y=5.0;
    SGM::Curve Line5=SGM::CreateLine(rResult,Origin,Axis);
    nHits=SGM::IntersectCurveAndSurface(rResult,Line5,TorusID,aPoints,aTypes,SGM_MIN_TOL);
    EXPECT_EQ(nHits,0U);

    SGM::DeleteEntity(rResult,TorusID);
    SGM::DeleteEntity(rResult,Line1);
    SGM::DeleteEntity(rResult,Line2);
    SGM::DeleteEntity(rResult,Line3);
    SGM::DeleteEntity(rResult,Line4);
    SGM::DeleteEntity(rResult,Line5);

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(math_check, test_transforms) 
    {
    // Test Transforms

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

    EXPECT_TRUE(SGM::NearEqual(Pos,Pos2,SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(Vec,Vec2,SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(UVec,UVec2,SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(Pos,Pos3,SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(Vec,Vec3,SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(UVec,UVec3,SGM_ZERO));
    }


TEST(math_check, DISABLED_least_squares_plane) 
    {
    // Test Least Squared Plane Fitting.

    std::vector<SGM::Point3D> aPoints = 
        {
            {0.0,0.0,0.0},
            {2.0,0.0,0.0},
            {2.0,1.0,0.0},
            {0.0,1.0,0.0},
            {1.0,0.5,0.4}
        };
    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);

    EXPECT_TRUE(SGM::NearEqual(Origin,SGM::Point3D(1.0,0.5,0.08),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO));

    std::vector<SGM::Point3D> aPoints2 =
        {
            {1.0,0.0,0.0},
            {0.0,1.0,0.0},
            {3.0,2.0,0.0},
            {2.0,3.0,0.0}
        };
    SGM::FindLeastSquarePlane(aPoints2,Origin,XVec,YVec,ZVec);

    EXPECT_TRUE(SGM::NearEqual(Origin,SGM::Point3D(1.5,1.5,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(YVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,-1.0),SGM_ZERO));

    std::vector<SGM::Point3D> aPoints3 = 
        {
            {1.0,0.0,0.0},
            {0.0,1.0,0.0},
            {1.0,1.0,0.0},
            {0.0,0.0,0.0},
            {1.0,0.0,1.0},
            {0.0,1.0,1.0},
            {1.0,1.0,1.0},
            {0.0,0.0,1.0}
        };

    SGM::FindLeastSquarePlane(aPoints3,Origin,XVec,YVec,ZVec);

    EXPECT_TRUE(SGM::NearEqual(Origin,SGM::Point3D(0.5,0.5,0.5),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,0.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO));

    std::vector<SGM::Point3D> aPoints4 =
        {
            {0.0,0.0,0.0},
            {8.0,8.0,0.0},
            {4.0,4.0,0.0}
        };

    SGM::FindLeastSquarePlane(aPoints4,Origin,XVec,YVec,ZVec);

    EXPECT_TRUE(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(YVec,SGM::UnitVector3D(-1.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(ZVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO));

    std::vector<SGM::Point3D> aPoints5 =
        {
            {0.0,0.0,0.0},
            {8.0,8.0,0.0},
            {4.0,4.0,0.1}
        };

    SGM::FindLeastSquarePlane(aPoints5,Origin,XVec,YVec,ZVec);

    EXPECT_TRUE(SGM::NearEqual(Origin,SGM::Point3D(4.0,4.0,0.0333333333333333),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(XVec,SGM::UnitVector3D(1.0,1.0,0.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(YVec,SGM::UnitVector3D(0.0,0.0,1.0),SGM_ZERO));
    EXPECT_TRUE(SGM::NearEqual(ZVec,SGM::UnitVector3D(1.0,-1.0,0.0),SGM_ZERO));
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
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);

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

    EXPECT_EQ(nGenus,0U);
    
    bool bLinear=SGM::IsLinear(rResult,ComplexID);

    EXPECT_FALSE(bLinear);

    bool bCycle=SGM::IsCycle(rResult,ComplexID);

    EXPECT_FALSE(bCycle);

    EXPECT_TRUE(SGM::IsOriented(rResult,ComplexID));

    SGM::Complex SmallComplexID=SGM::MergePoints(rResult,ComplexID,SGM_MIN_TOL);
    bool bManifold=SGM::IsManifold(rResult,SmallComplexID);

    EXPECT_TRUE(bManifold);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{2,0,0}};
    std::vector<unsigned> aSegments1 = {0,1,1,2};
    std::vector<unsigned> aSegments2 = {0,1,2,1};
    SGM::Complex ComplexID1=SGM::CreateSegments(rResult,aPoints,aSegments1);
    SGM::Complex ComplexID2=SGM::CreateSegments(rResult,aPoints,aSegments2);
    EXPECT_TRUE(SGM::IsOriented(rResult,ComplexID1));
    EXPECT_FALSE(SGM::IsOriented(rResult,ComplexID2));

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
    /*SGM::Complex ComplexID=*/SGM::CreateComplex(rResult,BlockID);
    
    // Top level checks.

    SGM::Volume VolumeID;
    SGM::Face FaceID;
    SGM::Edge EdgeID;
    SGM::Vertex VertexID;
    SGM::Curve CurveID;
    SGM::Surface SurfaceID;
    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,SGM::Thing(),sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,SGM::Thing(),sComplexes);
        EXPECT_EQ(sComplexes.size(),1U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);
        VolumeID=*(sVolumes.begin());

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,SGM::Thing(),sFaces);
        EXPECT_EQ(sFaces.size(),6U);
        FaceID=*(sFaces.begin());

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,SGM::Thing(),sEdges);
        EXPECT_EQ(sEdges.size(),12U);
        EdgeID=*(sEdges.begin());

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,SGM::Thing(),sVertex);
        EXPECT_EQ(sVertex.size(),8U);
        VertexID=*(sVertex.begin());

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,SGM::Thing(),sCurves);
        EXPECT_EQ(sCurves.size(),12U);
        CurveID=*(sCurves.begin());

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,SGM::Thing(),sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6U);
        SurfaceID=*(sSurfaces.begin());
    }

    // Body level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,BlockID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,BlockID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,BlockID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,BlockID,sFaces);
        EXPECT_EQ(sFaces.size(),6U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,BlockID,sEdges);
        EXPECT_EQ(sEdges.size(),12U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,BlockID,sVertex);
        EXPECT_EQ(sVertex.size(),8U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,BlockID,sCurves);
        EXPECT_EQ(sCurves.size(),12U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,BlockID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6U);
    }

    // Volume level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,VolumeID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,VolumeID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,VolumeID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,VolumeID,sFaces);
        EXPECT_EQ(sFaces.size(),6U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,VolumeID,sEdges);
        EXPECT_EQ(sEdges.size(),12U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,VolumeID,sVertex);
        EXPECT_EQ(sVertex.size(),8U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,VolumeID,sCurves);
        EXPECT_EQ(sCurves.size(),12U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,VolumeID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),6U);
    }

    // Face level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,FaceID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,FaceID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,FaceID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,FaceID,sFaces);
        EXPECT_EQ(sFaces.size(),1U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,FaceID,sEdges);
        EXPECT_EQ(sEdges.size(),4U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,FaceID,sVertex);
        EXPECT_EQ(sVertex.size(),4U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,FaceID,sCurves);
        EXPECT_EQ(sCurves.size(),4U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,FaceID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),1U);
    }

    // Surface level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,SurfaceID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,SurfaceID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,SurfaceID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,SurfaceID,sFaces);
        EXPECT_EQ(sFaces.size(),1U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,SurfaceID,sEdges);
        EXPECT_EQ(sEdges.size(),4U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,SurfaceID,sVertex);
        EXPECT_EQ(sVertex.size(),4U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,SurfaceID,sCurves);
        EXPECT_EQ(sCurves.size(),4U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,SurfaceID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),1U);
    }

    // Edge level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,EdgeID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,EdgeID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,EdgeID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,EdgeID,sFaces);
        EXPECT_EQ(sFaces.size(),2U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,EdgeID,sEdges);
        EXPECT_EQ(sEdges.size(),1U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,EdgeID,sVertex);
        EXPECT_EQ(sVertex.size(),2U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,EdgeID,sCurves);
        EXPECT_EQ(sCurves.size(),1U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,EdgeID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),2U);
    }

    // Curve level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,CurveID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,CurveID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,CurveID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,CurveID,sFaces);
        EXPECT_EQ(sFaces.size(),2U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,CurveID,sEdges);
        EXPECT_EQ(sEdges.size(),1U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,CurveID,sVertex);
        EXPECT_EQ(sVertex.size(),2U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,CurveID,sCurves);
        EXPECT_EQ(sCurves.size(),1U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,CurveID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),2U);
    }

    // Vertex level checks.

    {
        std::set<SGM::Body> sBodies;
        SGM::FindBodies(rResult,VertexID,sBodies);
        EXPECT_EQ(sBodies.size(),1U);

        std::set<SGM::Complex> sComplexes;
        SGM::FindComplexes(rResult,VertexID,sComplexes);
        EXPECT_EQ(sComplexes.size(),0U);

        std::set<SGM::Volume> sVolumes;
        SGM::FindVolumes(rResult,VertexID,sVolumes);
        EXPECT_EQ(sVolumes.size(),1U);

        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,VertexID,sFaces);
        EXPECT_EQ(sFaces.size(),3U);

        std::set<SGM::Edge> sEdges;
        SGM::FindEdges(rResult,VertexID,sEdges);
        EXPECT_EQ(sEdges.size(),3U);

        std::set<SGM::Vertex> sVertex;
        SGM::FindVertices(rResult,VertexID,sVertex);
        EXPECT_EQ(sVertex.size(),1U);

        std::set<SGM::Curve> sCurves;
        SGM::FindCurves(rResult,VertexID,sCurves);
        EXPECT_EQ(sCurves.size(),3U);

        std::set<SGM::Surface> sSurfaces;
        SGM::FindSurfaces(rResult,VertexID,sSurfaces);
        EXPECT_EQ(sSurfaces.size(),3U);
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
    SGM::SurfaceInverse(rResult,LemonID,Pos1,&CPos1);
    
    SGM::DeleteEntity(rResult,AppleID);
    SGM::DeleteEntity(rResult,LemonID);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(math_check, create_revolve) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
 
    std::vector<SGM::Point3D> aPoints =
        {
            {-2.0, 0.5, 0.0},
            {-1.0, 1.5, 0.0},
            { 0.0, 1.0, 0.0},
            { 1.0, 1.5, 0.0},
            { 2.0, 2.0, 0.0}
        };
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

TEST(math_check, create_disk) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::GetPointsOfBody(rResult,BodyID);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, point_body) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::set<SGM::Point3D> sPoints = {{0,0,0},{10,10,10}};
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

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aHitTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(0,0,-1),EdgeID,aHits,aHitTypes,aEntities);
    EXPECT_EQ(aHits.size(),1U);

    SGM::Point3D Pos0(1,0,0),Pos1(-1,0,0);
    SGM::ImprintPoint(rResult,Pos0,EdgeID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos1,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2U);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, ellipse_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0),YAxis(0,1,0),Normal;
    double dXRadius=1,dYRadius=2;
    SGM::Curve CurveID=SGM::CreateEllipse(rResult,Center,XAxis,YAxis,dXRadius,dYRadius);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);

    double dGuess1=0;
    double dGuess2=SGM_TWO_PI;
    SGM::Point3D TestPos(1,0,0);
    double t1=SGM::CurveInverse(rResult,CurveID,TestPos,nullptr,&dGuess1);
    double t2=SGM::CurveInverse(rResult,CurveID,TestPos,nullptr,&dGuess2);
    EXPECT_TRUE(SGM::NearEqual(t1,0,SGM_ZERO,false));
    EXPECT_TRUE(SGM::NearEqual(t2,SGM_TWO_PI,SGM_ZERO,false));

    double dA,dB;
    SGM::GetEllipseData(rResult,CurveID,Center,XAxis,YAxis,Normal,dA,dB);

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aHitTypes;
    std::vector<SGM::Entity> aEntites;
    SGM::RayFire(rResult,SGM::Point3D(0,-2,1),SGM::UnitVector3D(0,0,-1),EdgeID,aHits,aHitTypes,aEntites);
    EXPECT_EQ(aHits.size(),1U);

    SGM::Point3D Pos0(1,0,0),Pos1(-1,0,0);
    SGM::ImprintPoint(rResult,Pos0,EdgeID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos1,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2U);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1U);
    
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
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, parabola_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0),YAxis(0,1,0),Normal;
    double dA=1.0;
    SGM::Curve CurveID=SGM::CreateParabola(rResult,Center,XAxis,YAxis,dA);

    SGM::CopyEntity(rResult,CurveID);

    SGM::Interval1D Domain(-10,10);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID,&Domain);

    SGM::GetParabolaData(rResult,CurveID,Center,XAxis,YAxis,Normal,dA);

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aHitTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(0,0,-1),EdgeID,aHits,aHitTypes,aEntities);
    EXPECT_EQ(aHits.size(),1U);

    SGM::Point3D Pos0(0,0,0);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,1,0),SGM::UnitVector3D(1,0,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2U);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, hyperbola_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(0,1,0),YAxis(1,0,0),Normal;
    SGM::Curve CurveID=SGM::CreateHyperbola(rResult,Center,XAxis,YAxis,1,1);
    SGM::Interval1D Domain(-10,10);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID,&Domain);

    double dA,dB;
    SGM::GetHyperbolaData(rResult,CurveID,Center,XAxis,YAxis,Normal,dA,dB); 

    std::vector<SGM::Point3D> aHits;
    std::vector<SGM::IntersectionType> aHitTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(1,0,1),SGM::UnitVector3D(0,0,-1),EdgeID,aHits,aHitTypes,aEntities);
    EXPECT_EQ(aHits.size(),1U);

    SGM::Point3D Pos0(1,0,0);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurves(rResult,LineID,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),2U);

    SGM::Curve LineID1=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID1,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check,hermite_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{10,0,0}};
    std::vector<SGM::Vector3D> aVectors = {{1,1,0},{1,1,0}};
    std::vector<double> aParams = {0.,12.};
    SGM::Curve CurveID=SGM::CreateHermiteCurve(rResult,aPoints,aVectors,aParams);

    SGM::Point3D Pos0;
    SGM::EvaluateCurve(rResult,CurveID,6,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, NUB_curve_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 = 
        {
            {-2.0, 0.5, 0.0},
            {-1.0, 1.5, 0.0},
            { 0.0, 1.0, 0.0},
            { 1.0, 1.5, 0.0},
            { 2.0, 2.0, 0.0}
        };

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

    std::vector<SGM::Point3D> aControlPoints;
    std::vector<double> aKnots;
    SGM::GetNUBCurveData(rResult,CurveID,aControlPoints,aKnots);

    SGM::Point3D Pos0;
    double dt=SGM::GetDomainOfCurve(rResult,CurveID).MidPoint();
    SGM::EvaluateCurve(rResult,CurveID,dt,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGM::SaveSGM(rResult,"CoverageTest.sgm",SGM::Thing(),SGM::TranslatorOptions());

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, DISABLED_NURB_curve_merge) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints =
        {
            {1,0,0,1},
            {1,1,0,SGM_SQRT_2/2},
            {0,1,0,1},
            {-1,1,0,SGM_SQRT_2/2},
            {-1,0,0,1},
            {-1,-1,0,SGM_SQRT_2/2},
            {0,-1,0,1},
            {1,-1,0,SGM_SQRT_2/2},
            {1,0,0,1}
        };
    std::vector<double> aKnots = 
        {
            0,
            0,
            0,
            SGM_HALF_PI,
            SGM_HALF_PI,
            SGM_PI,
            SGM_PI,
            SGM_PI*1.5,
            SGM_PI*1.5,
            SGM_TWO_PI,
            SGM_TWO_PI,
            SGM_TWO_PI
        };

    SGM::Curve CurveID=SGM::CreateNURBCurve(rResult,aControlPoints,aKnots);

    SGM::Point3D Pos0;
    double dt=SGM::GetDomainOfCurve(rResult,CurveID).MidPoint();
    SGM::EvaluateCurve(rResult,CurveID,dt,&Pos0);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    SGM::Vertex VertexID=SGM::ImprintPoint(rResult,Pos0,EdgeID);
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,VertexID,sEdges);
    EXPECT_EQ(sEdges.size(),2U);
    SGM::Body BodyID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Merge(rResult,BodyID);
    sEdges.clear();
    SGM::FindEdges(rResult,BodyID,sEdges);
    EXPECT_EQ(sEdges.size(),1U);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, ray_fire_thing) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,BodyID);
    SGM::TransformEntity(rResult,SGM::Transform3D(SGM::Vector3D(1,1,1)),ComplexID);

    SGM::DeleteEntity(rResult,BodyID);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(-1,-1,-1),SGM::UnitVector3D(1,1,1),SGM::Thing(),aPoints,aTypes,aEntities);

    EXPECT_EQ(aPoints.size(),2U);

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

    EXPECT_EQ(aPoints.size(),2U);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersect_line_cone) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0));
    SGM::Surface SurfID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,0.5);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndSurface(rResult,LineID,SurfID,aPoints,aTypes);

    EXPECT_EQ(aPoints.size(),2U);

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

    EXPECT_EQ(aPoints.size(),2U);

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

    EXPECT_EQ(aCurves.size(),2U);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, complex_tests) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints {
        {0,0,0},
        {1,0,0},
        {0,1,0},
        {1,1,0},
        {0,1,0},
        {1,0,0}        
    };
    std::vector<unsigned> aTriangles = {
            0,1,2,3,4,5
    };
    SGM::Complex ComplexID=SGM::CreateTriangles(rResult,aPoints,aTriangles);

    std::vector<unsigned> aSegments = 
            {0,1,1,2,2,0,3,4};
    SGM::Complex SegmentsID=SGM::CreateSegments(rResult,aPoints,aSegments);

    std::vector<SGM::Complex> aComponents;
    EXPECT_EQ(SGM::FindComponents(rResult,SegmentsID,aComponents),3U);
    SGM::ReduceToUsedPoints(rResult,SegmentsID);
    aComponents.clear();
    EXPECT_EQ(SGM::FindComponents(rResult,SegmentsID,aComponents),2U);
    aComponents.clear();
    EXPECT_EQ(SGM::FindPlanarParts(rResult,SegmentsID,aComponents,SGM_MIN_TOL),2U);
    SGM::FindBoundary(rResult,ComplexID);

    SGM::SaveSGM(rResult,"CoverageTest.sgm",SGM::Thing(),SGM::TranslatorOptions());

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

TEST(math_check, stl_face_save) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    SGM::TranslatorOptions TranslatorOpts;
    SGM::SaveSTL(rResult,"sphere_test1.stl",BodyID,TranslatorOpts);
    TranslatorOpts.m_b2D=true;
    SGM::SaveSTL(rResult,"sphere_test2.stl",BodyID,TranslatorOpts);

    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, imprint_point_on_block) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,BodyID,sEdges);
    SGM::Edge EdgeID=*(sEdges.begin());
    SGM::ImprintPoint(rResult,SGM::Point3D(0,0,0),EdgeID);

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
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,VolumeID));
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    // Check for empty bodies.

    SGM::DeleteEntity(rResult,VolumeID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    // Bad segments in a complex.

    std::vector<SGM::Point3D> aPoints = {{0,0,0}};
    std::vector<unsigned> aSegments = {0,0,1,2};
    SGM::Complex ComplexID=SGM::CreateSegments(rResult,aPoints,aSegments);
    SGM::CheckOptions Options3;
    std::vector<std::string> aCheckStrings3;
    EXPECT_FALSE(SGM::CheckEntity(rResult,ComplexID,Options3,aCheckStrings3));
    SGM::DeleteEntity(rResult,ComplexID);

    // Bad triangles in a complex.

    std::vector<SGM::Point3D> aPoints2 = {{0,0,0}};
    std::vector<unsigned> aTriangles2 = {0,0,2};
    SGM::Complex ComplexID2=SGM::CreateTriangles(rResult,aPoints2,aTriangles2);
    SGM::CheckOptions Options4;
    std::vector<std::string> aCheckStrings4;
    EXPECT_FALSE(SGM::CheckEntity(rResult,ComplexID2,Options4,aCheckStrings4));
    SGM::DeleteEntity(rResult,ComplexID2);

    // Bad facets normals and connectivity.

    rResult.SetDebugFlag(1);
    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::CheckOptions Options5;
    std::vector<std::string> aCheckStrings5;
    EXPECT_FALSE(SGM::CheckEntity(rResult,BlockID,Options5,aCheckStrings5));
    SGM::DeleteEntity(rResult,BlockID);
    rResult.SetDebugFlag(0);

    // Missing facets.

    rResult.SetDebugFlag(2);
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::CheckOptions Options6;
    std::vector<std::string> aCheckStrings6;
    EXPECT_FALSE(SGM::CheckEntity(rResult,BlockID2,Options6,aCheckStrings6));
    SGM::DeleteEntity(rResult,BlockID2);
    rResult.SetDebugFlag(0);

    // bad curves.

    rResult.SetDebugFlag(3);
    SGM::Edge LineID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::CheckOptions Options7;
    std::vector<std::string> aCheckStrings7;
    EXPECT_FALSE(SGM::CheckEntity(rResult,LineID,Options7,aCheckStrings7));
    SGM::DeleteEntity(rResult,LineID);
    rResult.SetDebugFlag(0);

    // bad surface.

    rResult.SetDebugFlag(4);
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    SGM::CheckOptions Options8;
    std::vector<std::string> aCheckStrings8;
    EXPECT_FALSE(SGM::CheckEntity(rResult,SphereID,Options8,aCheckStrings8));
    SGM::DeleteEntity(rResult,SphereID);
    rResult.SetDebugFlag(0);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, create_polygon) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{0,1,0}};
    SGM::CreatePolygon(rResult,aPoints,false);
    SGM::CreatePolygon(rResult,aPoints,true);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, intersection_tests) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body SphereID=SGM::CreateSphere(rResult,{0,0,0},1);
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(0.9,0.9,0),SGM::UnitVector3D(1,0,0),SphereID,aPoints,aTypes,aEntities);
    EXPECT_EQ(aPoints.size(),0U);
    SGM::RayFire(rResult,SGM::Point3D(1,-1,0),SGM::UnitVector3D(0,1,0),SphereID,aPoints,aTypes,aEntities);
    EXPECT_EQ(aPoints.size(),1U);

    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0));
    EXPECT_TRUE(SGM::NearEqual(SGM::FindEdgeLength(rResult,EdgeID),1,SGM_MIN_TOL,false));

    SGM::Surface CylinderID=SGM::CreateCylinderSurface(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,1),1);
    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,0,1));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndSurface(rResult,LineID,CylinderID,aPoints,aTypes);
    EXPECT_EQ(aTypes[0],SGM::IntersectionType::CoincidentType);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(0,1,0),YAxis(1,0,0);
    SGM::Curve CurveID=SGM::CreateHyperbola(rResult,Center,XAxis,YAxis,1,1);
    SGM::Curve LineID2=SGM::CreateLine(rResult,SGM::Point3D(0,1,0),SGM::UnitVector3D(1,-1,0));
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurves(rResult,LineID2,CurveID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),1U);

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, volume_ray_fire) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateSphere(rResult,SGM::Point3D(0,0,0),1);
    std::set<SGM::Volume> sVolumes;
    SGM::ReduceToVolumes(rResult,BodyID,sVolumes);
    SGM::Volume VolumeID=*(sVolumes.begin());
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<SGM::Entity> aEntities;
    SGM::RayFire(rResult,SGM::Point3D(-2,0,0),SGM::UnitVector3D(1,0,0),VolumeID,aPoints,aTypes,aEntities);
    EXPECT_EQ(aPoints.size(),2U);
    SGM::RayFire(rResult,SGM::Point3D(-2,0,0),SGM::UnitVector3D(1,0,0),SGM::Thing(),aPoints,aTypes,aEntities);
    EXPECT_EQ(aPoints.size(),2U);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, non_manifold_complex) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{0,1,0},{1,0,0},{0,0,1},{-1,0,0}};
    std::vector<unsigned> aTriangles = {0, 1, 2, 0, 1, 3, 0, 1, 4};
    SGM::Complex IDComplex=SGM::CreateTriangles(rResult,aPoints,aTriangles);
    EXPECT_FALSE(SGM::IsManifold(rResult,IDComplex));
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, DISABLED_non_manifold_complex_1d) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints {{0,0,0},{0,1,0},{1,0,0},{-1,0,0}};
    std::vector<unsigned> aSegments = {0,1,0,2,0,3};
    SGM::Complex IDComplex=SGM::CreateSegments(rResult,aPoints,aSegments);
    EXPECT_FALSE(SGM::IsManifold(rResult,IDComplex));
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, find_sharp_edges) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,{0,0,0},{10,10,10});
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    //SGM::DeleteEntity(rResult,FaceID);
    //SGM::Complex ComplexID=SGM::CreateComplex(rResult,BodyID);
    SGM::Complex ComplexID=SGM::CreateComplex(rResult,FaceID);
    SGM::FindSharpEdges(rResult,ComplexID,0.1,true);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, find_oriented_box) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints {{0,0,0},{0,1,0},{1,0,0},{0,0,1},{-1,0,0}};
    SGM::Point3D Origin;
    SGM::UnitVector3D XVec(1,0,0),YVec(0,1,0),ZVec(0,0,1);
    SGM::FindOrientedBox(aPoints,Origin,XVec,YVec,ZVec);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, cone_inverse_at_apex) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,1);
    SGM::Point3D Pos(1,0,10),ClosePos;
    SGM::Point2D Guess(1,0);
    SGM::SurfaceInverse(rResult,SurfID,Pos,&ClosePos,&Guess);
        
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, find_sad_parabola) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,-1,0},{2,-4,0},{-1,-1,0},{-2,-4,0}};
    SGM::FindConic(rResult,aPoints,SGM_MIN_TOL);
    
    SGMTesting::ReleaseTestThing(pThing);
} 


TEST(math_check, find_conic_line) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{2,0,0},{3,0,0},{4,0,0}};
    SGM::FindConic(rResult,aPoints,SGM_MIN_TOL);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, find_conic_too_few_points) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
    SGM::FindConic(rResult,aPoints,SGM_MIN_TOL);
    EXPECT_FALSE(rResult.GetResult()==SGM::ResultTypeOK);
    
    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, testing_the_thing) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID = SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Interval3D ThingBox1 = SGM::GetBoundingBox(rResult,SGM::Thing());
    SGM::Interval3D BlockBox1 = SGM::GetBoundingBox(rResult,BodyID);
    EXPECT_TRUE(SGM::NearEqual(BlockBox1.MidPoint(), ThingBox1.MidPoint(), SGM_MIN_TOL));

    SGM::Entity EntID=SGM::Thing();
    SGM::Vector3D MoveVector(1,1,1);
    SGM::Transform3D Translation(MoveVector);
    SGM::TransformEntity(rResult,Translation,EntID);
    SGM::Interval3D ThingBox2 = SGM::GetBoundingBox(rResult,SGM::Thing());
    EXPECT_TRUE(SGM::NearEqual(ThingBox2.MidPoint(), ThingBox1.MidPoint() + MoveVector, SGM_MIN_TOL));

    // make sure the Block's box also got updated
    SGM::Interval3D BlockBox2 = SGM::GetBoundingBox(rResult, BodyID);
    EXPECT_TRUE(SGM::NearEqual(ThingBox2.MidPoint(), BlockBox2.MidPoint(), SGM_MIN_TOL));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, principal_curvatures_on_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfID=SGM::CreatePlane(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0),SGM::Point3D(0,1,0));
    double dK1,dK2;
    SGM::UnitVector3D Vec1,Vec2;
    SGM::Point2D uv(0,0);
    SGM::PrincipleCurvature(rResult,SurfID,uv,Vec1,Vec2,dK1,dK2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, same_curve_tests)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints =
        {
            { 1,  0,  0, 1},
            { 1,  1,  0, SGM_SQRT_2 / 2},
            { 0,  1,  0, 1},
            {-1,  1,  0, SGM_SQRT_2 / 2},
            {-1,  0,  0, 1},
            {-1, -1,  0, SGM_SQRT_2 / 2},
            { 0, -1,  0, 1},
            { 1, -1,  0, SGM_SQRT_2 / 2},
            { 1,  0,  0, 1},
        };

    std::vector<double> aKnots =
        {0,0,0,SGM_HALF_PI,SGM_HALF_PI,SGM_PI,SGM_PI,SGM_PI*1.5,SGM_PI*1.5,SGM_TWO_PI,SGM_TWO_PI,SGM_TWO_PI};

    SGM::Curve CurveID1=SGM::CreateNURBCurve(rResult,aControlPoints,aKnots);

    SGM::Curve CurveID2=SGM::Curve(SGM::CopyEntity(rResult,CurveID1).m_ID);
    EXPECT_TRUE(SGM::SameCurve(rResult,CurveID1,CurveID2,SGM_MIN_TOL));
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,CurveID1);
    EXPECT_FALSE(SGM::SameCurve(rResult,CurveID1,CurveID2,SGM_MIN_TOL));
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, transform_nub_curve)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 = 
        {
            {-2,.5,0},
            {-1,1.5,0},
            {0,1,0},
            {1,1.5,0},
            {2,2,0}
        };

    SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);
    SGM::TransformEntity(rResult,SGM::Transform3D(SGM::Vector3D(1,2,3)),CurveID);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, entity_free_intersections) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    EXPECT_TRUE(SGM::RunInternalTest(rResult,2));

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, internal_surface_tests) 
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    EXPECT_TRUE(SGM::RunInternalTest(rResult,6));

    SGMTesting::ReleaseTestThing(pThing);
} 

TEST(math_check, line_cone_intersect_empty)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface ConeID=SGM::CreateConeSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,SGM_HALF_PI*0.5);
    SGM::Curve LineID=SGM::CreateLine(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0));
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndSurface(rResult,LineID,ConeID,aPoints,aTypes);
    EXPECT_EQ(aPoints.size(),0U);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(math_check, line_nub_surface_intersect)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots =
        {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point3D>> aaPoints =
        {{{ 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 2.0,-1.0}},
         {{ 1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 2.0, 0.0}},
         {{ 2.0, 0.0,-1.0}, {2.0, 1.0, 0.0}, {2.0, 2.0, 1.0}}};
    
    SGM::Surface NUBSurfaceID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    SGM::SaveSGM(rResult,"GTest_NUB_surface_test.sgm",NUBSurfaceID,SGM::TranslatorOptions());

    // Test with a line that hits two points.

    SGM::Point3D Pos2(0,0,0.5),Pos3(2,2,0.5);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);

    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    size_t nHits2=SGM::IntersectCurveAndSurface(rResult,LineID2,NUBSurfaceID,aHits2,aTypes2);

    EXPECT_EQ(nHits2,2U);
    for(auto const & Pos : aHits2)
        {
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        EXPECT_LT(dDist,SGM_MIN_TOL);
        }
    SGM::DeleteEntity(rResult,LineID2);;
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, line_nurb_surface_intersect)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots = {0,0,0,1,1,1};
    std::vector<double> aVKnots(aUKnots);
    std::vector<std::vector<SGM::Point4D>> aaPoints =
        {{{0.0, 0.0, 1.0, 1}, {0.0, 1.0, 0.0, 1}, {0.0, 2.0,-1.0, 1}},
         {{1.0, 0.0, 0.0, 1}, {1.0, 1.0, 0.0, 1}, {1.0, 2.0, 0.0, 1}},
         {{2.0, 0.0,-1.0, 1}, {2.0, 1.0, 0.0, 1}, {2.0, 2.0, 1.0, 1}}};
    
    SGM::Surface NUBSurfaceID=SGM::CreateNURBSurface(rResult,aaPoints,aUKnots,aVKnots);

    SGM::GetNURBSurfaceData(rResult,NUBSurfaceID,aaPoints,aUKnots,aVKnots);

    // Test with a line that hits two points.

    SGM::Point3D Pos2(0,0,0.5),Pos3(2,2,0.5);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);

    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    size_t nHits2=SGM::IntersectCurveAndSurface(rResult,LineID2,NUBSurfaceID,aHits2,aTypes2);

    EXPECT_EQ(nHits2,2U);
    for(SGM::Point3D const &Pos : aHits2)
        {
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        EXPECT_LT(dDist,SGM_MIN_TOL);
        }
    SGM::DeleteEntity(rResult,LineID2);
    
    SGMTesting::ReleaseTestThing(pThing);

    }

TEST(math_check, line_extrude_intersect_conicident)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints = {{0,0,0},{10,0,0}};
    std::vector<SGM::Vector3D> aVectors{{1,1,0},{1,1,0}};
    std::vector<double> aParams = {0,12};
    SGM::Curve CurveID=SGM::CreateHermiteCurve(rResult,aPoints,aVectors,aParams);
    SGM::UnitVector3D Axis(0,0,1);
    SGM::Surface SurfID=SGM::CreateExtrudeSurface(rResult,Axis,CurveID);

    SGM::Curve LineID=SGM::CreateLine(rResult,{0,0,0},{0,0,1});

    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    SGM::IntersectCurveAndSurface(rResult,LineID,SurfID,aHits2,aTypes2);

    EXPECT_EQ(aTypes2[0],SGM::IntersectionType::CoincidentType);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(math_check, local_coordinates)
{
    SGM::Point3D Origin(1, 1, 1);
    SGM::UnitVector3D Vec1(1, 0, 0);
    SGM::UnitVector3D Vec2(1, 1, 0);
    SGM::UnitVector3D Vec3(1, 0, 1);
    SGM::Point3D Pos(2, 2, 2);
    SGM::Point3D Local = SGM::FindLocalCoordinates(Origin, Vec1, Vec2, Vec3, Pos, false);

    SGM::Point3D TestPos = Origin + Local.m_x*Vec1 + Local.m_y*Vec2 + Local.m_z*Vec3;
    EXPECT_TRUE(SGM::NearEqual(TestPos, Pos, SGM_ZERO));

    Pos = {1, 1, 1};
    Local = SGM::FindLocalCoordinates(Origin, Vec1, Vec2, Vec3, Pos, false);

    EXPECT_TRUE(SGM::NearEqual(Local, SGM::Point3D(0, 0, 0), SGM_ZERO));
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif
