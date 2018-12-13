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

#include "test_utility.h"

TEST(Boolean_check, Peninsula_Peninsula_Disks)
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

TEST(Boolean_check, Splitter_Island_Disks)
    {
    // Boolean of two disks Splitter and Island
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center0(0,0,0),Center1(0,0.5,0);
    SGM::UnitVector3D Normal0(0,0,1),Normal1(0,1,0);
    SGM::Body KeepID=SGM::CreateDisk(rResult,Center0,Normal0,1.0);
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

TEST(Boolean_check, Imprinting_Atoll_Bridge_Edge)
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

TEST(Boolean_check, Sphere_Sphere_Imprint1)
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

TEST(Boolean_check, Sphere_Sphere_Imprint2)
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

TEST(Boolean_check, Sphere_Sphere_Unite)
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

