#include <limits>
#include <string>
#include <set>
#include <gtest/gtest.h>
#include <EntityClasses.h>

#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMMeasure.h"
#include "SGMModify.h" 
#include "SGMTopology.h"
#include "SGMGeometry.h"
#include "SGMInterrogate.h"
#include "SGMIntersector.h"
#include "SGMEntityFunctions.h"
#include "SGMTransform.h"
#include "SGMDisplay.h"

#include "test_utility.h"

TEST(all_cases, peninsula_peninsula)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,-5),SGM::Point3D(15,5,5));

    //SGM::UniteBodies(rResult,BodyID1,BodyID2);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID2,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(all_cases, peninsula_peninsula_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,-5),SGM::Point3D(15,5,5));

    SGM::Transform3D Trans(SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),SGM::UnitVector3D(0,0,1),SGM::Point3D(5,5,0),
                           SGM::UnitVector3D(1,1,0),SGM::UnitVector3D(-1,1,0),SGM::UnitVector3D(0,0,1),SGM::Point3D(5,5,0));

    SGM::TransformEntity(rResult,Trans,BodyID2);

    SGM::Transform3D Trans2(SGM::Vector3D(20,0,0));
    SGM::TransformEntity(rResult,Trans2,BodyID1);
    SGM::TransformEntity(rResult,Trans2,BodyID2);

    //SGM::UniteBodies(rResult,BodyID1,BodyID2);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID2,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(all_cases, peninsula_peninsula_vertex_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,-5),SGM::Point3D(15,5,5));

    SGM::Transform3D TransA(SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,0,1),SGM::UnitVector3D(0,-1,0),SGM::Point3D(10,5,0),
                            SGM::UnitVector3D(1,0,1),SGM::UnitVector3D(-1,0,1),SGM::UnitVector3D(0,-1,0),SGM::Point3D(10,5,0));

    SGM::Transform3D TransB(SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),SGM::UnitVector3D(0,0,1),SGM::Point3D(5,5,0),
                            SGM::UnitVector3D(1,1,0),SGM::UnitVector3D(-1,1,0),SGM::UnitVector3D(0,0,1),SGM::Point3D(5,5,0));

    SGM::TransformEntity(rResult,TransA,BodyID2);
    SGM::TransformEntity(rResult,TransB,BodyID2);

    SGM::Transform3D Trans2(SGM::Vector3D(40,0,0));
    SGM::TransformEntity(rResult,Trans2,BodyID1);
    SGM::TransformEntity(rResult,Trans2,BodyID2);

    //SGM::UniteBodies(rResult,BodyID1,BodyID2);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID2,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(all_cases, split_split)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(0,5,-5),SGM::Point3D(10,5,5));

    SGM::Transform3D Trans2(SGM::Vector3D(0,20,0));
    SGM::TransformEntity(rResult,Trans2,BodyID1);
    SGM::TransformEntity(rResult,Trans2,BodyID2);

    //SGM::UniteBodies(rResult,BodyID1,BodyID2);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID2,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

/*
TEST(all_cases, split_split_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    std::set<SGM::Edge> sEdges;
    sEdges.insert(SGM::CreateLinearEdge(rResult,SGM::Point3D(0,10,-5),SGM::Point3D(10,5,-5)));
    sEdges.insert(SGM::CreateLinearEdge(rResult,SGM::Point3D(10,5,-5),SGM::Point3D(10,5,5)));
    sEdges.insert(SGM::CreateLinearEdge(rResult,SGM::Point3D(10,5,5),SGM::Point3D(0,10,5)));
    sEdges.insert(SGM::CreateLinearEdge(rResult,SGM::Point3D(0,10,5),SGM::Point3D(0,10,-5)));
    SGM::Body WireID=SGM::CreateWireBody(rResult,sEdges);
    SGM::Body BodyID2=SGM::CoverPlanarWire(rResult,WireID);
    SGM::DeleteEntity(rResult,WireID);

    SGM::Transform3D Trans2(SGM::Vector3D(20,20,0));
    SGM::TransformEntity(rResult,Trans2,BodyID1);
    SGM::TransformEntity(rResult,Trans2,BodyID2);

    //SGM::UniteBodies(rResult,BodyID1,BodyID2);
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID2,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}
*/
///////////////////////////////////////////////////////////////////////////////

TEST(boolean_check, Peninsula_Peninsula_Disks)
    {
    // Boolean of two disks Peninsula
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center0(0,0,0),Center1(0,1.0,0);
    SGM::UnitVector3D Normal0(0,0,1),Normal1(1,0,0);
    SGM::Body KeepID=SGM::CreateDisk(rResult,Center0,Normal0,1.0);
    SGM::Body DeleteID=SGM::CreateDisk(rResult,Center1,Normal1,1.0);

    SGM::UniteBodies(rResult,KeepID,DeleteID);

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,KeepID,sFaces);
    EXPECT_EQ(sFaces.size(), 2);

    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,KeepID,sEdges);
    EXPECT_EQ(sEdges.size(), 3);

    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,KeepID,sVertices);
    EXPECT_EQ(sVertices.size(), 2);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,KeepID,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, Splitter_Island_Disks) 
    {
    // Boolean of two disks Splitter and Island
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center0(0,0,0),Center1(0,0.5,0);
    SGM::UnitVector3D Normal0(0,0,1),Normal1(0,1,0);
    SGM::Body KeepID=SGM::CreateDisk(rResult,Center0,Normal0,0.8);
    SGM::Body DeleteID=SGM::CreateDisk(rResult,Center1,Normal1,1.0);

    SGM::UniteBodies(rResult,KeepID,DeleteID);
    
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,KeepID,sFaces);
    EXPECT_EQ(sFaces.size(), 3);
    
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,KeepID,sEdges);
    EXPECT_EQ(sEdges.size(), 4);
    
    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,KeepID,sVertices);
    EXPECT_EQ(sVertices.size(), 2);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,KeepID,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, Imprinting_Atoll_Bridge_Edge)
    {
    // Imprinting an atoll edge on a face.
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D Normal(0,0,1);
    SGM::Body DiskID=SGM::CreateDisk(rResult,Center,Normal,1.0);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Normal,0.5);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,DiskID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    std::vector<SGM::Face> aFaces=SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    // Imprinting a bridge edge on a face.

    SGM::Point3D StartPos(0.5,0,0),EndPos(1.0,0,0);
    SGM::Edge EdgeID2=SGM::CreateLinearEdge(rResult,StartPos,EndPos);

    SGM::ImprintEdgeOnFace(rResult,EdgeID2,FaceID);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,DiskID,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, Sphere_Sphere_Imprint1)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center1(0,0,0),Center2(2,0,0);
    SGM::Body Sphere1=SGM::CreateSphere(rResult,Center1,1.0);
    SGM::Body Sphere2=SGM::CreateSphere(rResult,Center2,2.0);

    std::set<SGM::Surface> sSurfaces1,sSurfaces2;
    SGM::FindSurfaces(rResult,Sphere1,sSurfaces1);
    SGM::Surface SurfID1=*(sSurfaces1.begin());
    SGM::FindSurfaces(rResult,Sphere2,sSurfaces2);
    SGM::Surface SurfID2=*(sSurfaces2.begin());
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SurfID1,SurfID2,aCurves);

    SGM::DeleteEntity(rResult,Sphere2);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,aCurves[0]);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,Sphere1,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,Sphere1,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, Sphere_Sphere_Imprint2)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center1(0,0,0),Center2(2,0,0);
    SGM::Body Sphere1=SGM::CreateSphere(rResult,Center1,1.0);
    SGM::Body Sphere2=SGM::CreateSphere(rResult,Center2,2.0);

    std::set<SGM::Surface> sSurfaces1,sSurfaces2;
    SGM::FindSurfaces(rResult,Sphere1,sSurfaces1);
    SGM::Surface SurfID1=*(sSurfaces1.begin());
    SGM::FindSurfaces(rResult,Sphere2,sSurfaces2);
    SGM::Surface SurfID2=*(sSurfaces2.begin());
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SurfID1,SurfID2,aCurves);

    SGM::DeleteEntity(rResult,Sphere1);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    SGM::Edge EdgeID=SGM::CreateEdge(rResult,aCurves[0]);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,Sphere2,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,Sphere2,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, Sphere_Sphere_Unite)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center1(0,0,0),Center2(2,0,0);
    SGM::Body Sphere1=SGM::CreateSphere(rResult,Center1,1.0);
    SGM::Body Sphere2=SGM::CreateSphere(rResult,Center2,2.0);

    SGM::UniteBodies(rResult,Sphere1,Sphere2);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,Sphere1,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(boolean_check, imprint_face_on_face_through_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateDisk(rResult,SGM::Point3D(10,10,0),SGM::UnitVector3D(1,-1,0),1);
    SGM::UniteBodies(rResult,BodyID2,BodyID1);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, imprint_edge_on_face_vertex_hit)
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


TEST(boolean_check, tangent_imprint_case )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(-1,-1,0),SGM::Point3D(1,-1,0));
    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);
    
    SGMTesting::ReleaseTestThing(pThing);
}


TEST(boolean_check, unite_bodies_peninsula )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body DiskID1=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Body DiskID2=SGM::CreateDisk(rResult,SGM::Point3D(1,0,0),SGM::UnitVector3D(0,1,0),1);
    SGM::UniteBodies(rResult,DiskID1,DiskID2);

    // TODO part needs to check.  PRS

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, unite_bodies_island )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body DiskID1=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Body DiskID2=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),0.5);
    SGM::UniteBodies(rResult,DiskID1,DiskID2);

    // TODO part needs to check.  PRS

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, imprint_edge_on_face_atoll)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body DiskID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,DiskID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Curve CircleID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),0.5);
    SGM::Edge EdgeID1=SGM::CreateEdge(rResult,CircleID);
    SGM::ImprintEdgeOnFace(rResult,EdgeID1,FaceID);

    SGM::Edge EdgeID2=SGM::CreateLinearEdge(rResult,SGM::Point3D(0.5,0,0),SGM::Point3D(1,0,0));
    SGM::ImprintEdgeOnFace(rResult,EdgeID2,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, imprint_edge_on_face)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body DiskID=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,DiskID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,2,0),SGM::Point3D(0,-2,0));
    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);
    SGM::Merge(rResult,DiskID);
     
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, winding_numbers)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Surface SurfaceID=SGM::CreateTorusSurface(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    //SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Curve CurveID=SGM::CreateTorusKnot(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),SGM::UnitVector3D(0,1,0),1,2,3,4);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    std::vector<SGM::Point3D> const &aPoints=SGM::GetEdgePoints(rResult,EdgeID);
    int nUWinds,nVWinds;
    SGM::FindWindingNumbers(rResult,SurfaceID,aPoints,nUWinds,nVWinds);

    EXPECT_EQ(nUWinds,3);
    EXPECT_EQ(nVWinds,4);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, lower_genus)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateTorus(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1,2);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,SGM::Point3D(2,0,0),SGM::UnitVector3D(0,1,0),1);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    
    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, coincident)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(10,0,0),SGM::Point3D(10,10,10));
    SGM::UniteBodies(rResult,BodyID1,BodyID2);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(boolean_check, coincident_2)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(5,0,0),SGM::Point3D(5,10,5));
    SGM::UniteBodies(rResult,BodyID1,BodyID2);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(boolean_check, coincident_3)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
//    SGM::Body BodyID2=SGM::CreateBlock(rResult,SGM::Point3D(10,-2,0),SGM::Point3D(10,12,5));
//    SGM::UniteBodies(rResult,BodyID1,BodyID2);
//
//    SGM::CheckOptions Options;
//    std::vector<std::string> aCheckStrings;
//    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

//TEST(boolean_check, imprint_line_sqaure)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//
//    // TrimCurveWithFaces issue
//
//    SGM::Body BodyID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
//    std::set<SGM::Face> sFaces;
//    SGM::FindFaces(rResult,BodyID1,sFaces);
//    SGM::Face FaceID=*(sFaces.begin());
//    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(5,0,0),SGM::Point3D(5,5,0));
//    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);
//
//    SGM::CheckOptions Options;
//    std::vector<std::string> aCheckStrings;
//    EXPECT_TRUE(SGM::CheckEntity(rResult,BodyID1,Options,aCheckStrings));
//
//    SGMTesting::ReleaseTestThing(pThing);
//}
