#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMEntityFunctions.h"
#include "SGMEntityClasses.h"
#include "SGMTopology.h"
#include "SGMGeometry.h"
#include "SGMModify.h"

#include "test_utility.h"

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "cert-err58-cpp"

TEST(delete_check, delete_cannot_delete_entity)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body Block = SGM::CreateBlock(rResult, SGM::Point3D(0.,0.,0.), SGM::Point3D(.5,1.0, 1.5));

    std::set<SGM::Vertex> sVertices;
    std::set<SGM::Edge> sEdges;
    std::set<SGM::Face> sFaces;
    std::set<SGM::Volume> sVolumes;
    std::set<SGM::Curve> sCurves;
    std::set<SGM::Surface> sSurfaces;
    SGM::FindVertices(rResult, Block, sVertices);
    SGM::FindEdges(rResult, Block, sEdges);
    SGM::FindFaces(rResult, Block, sFaces);
    SGM::FindVolumes(rResult, Block, sVolumes);
    SGM::FindCurves(rResult, Block, sCurves);
    SGM::FindSurfaces(rResult, Block, sSurfaces);

    SGM::Vertex VertexID = *(sVertices.begin());
    SGM::Edge EdgeID = *(sEdges.begin());
    SGM::Face FaceID = *(sFaces.begin());
    SGM::Volume VolumeID = *(sVolumes.begin());
    SGM::Curve CurveID = *(sCurves.begin());
    SGM::Surface SurfaceID = *(sSurfaces.begin());

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    SGM::DeleteEntity(rResult, VertexID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    SGM::DeleteEntity(rResult, EdgeID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();

    SGM::DeleteEntity(rResult, FaceID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();

    SGM::DeleteEntity(rResult, VolumeID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();

    SGM::DeleteEntity(rResult, CurveID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();

    SGM::DeleteEntity(rResult, SurfaceID);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeCannotDelete);
    rResult.ClearMessage();

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    SGM::DeleteEntity(rResult, Block);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(delete_check, delete_edge_with_shared_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Origin(0,0,0);
    SGM::UnitVector3D Axis(1,0,0);
    SGM::Curve CurveID=SGM::CreateLine(rResult,Origin,Axis);

    SGM::Vertex Vertex1 = SGM::CreateVertex(rResult, SGM::Point3D(1,0,0));
    SGM::Vertex Vertex2 = SGM::CreateVertex(rResult, SGM::Point3D(10,0,0));
    SGM::Vertex Vertex3 = SGM::CreateVertex(rResult, SGM::Point3D(30,0,0));

    SGM::Edge Edge1=SGM::CreateEdge(rResult, CurveID, Vertex1, Vertex2);
    SGM::Edge Edge2=SGM::CreateEdge(rResult, CurveID, Vertex2, Vertex3);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    SGM::DeleteEntity(rResult, Edge1);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult,SGM::Thing()));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(delete_check, delete_surface_with_shared_curve)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aKnots={0,0,0,0,0.5,1,1,1,1};
    std::vector<SGM::Point3D> aControlPoints = {{1  ,1  ,0}, {1.1,1.1,0}, {2  ,2.8,0}, {2.8,1.1,0}, {3  ,1  ,0}};
    SGM::Curve NUBID=SGM::CreateNUBCurveWithControlPointsAndKnots(rResult,aControlPoints,aKnots);

    SGM::Surface ExtrudeID1 = SGM::CreateExtrudeSurface(rResult, SGM::UnitVector3D(0,0,1), NUBID);
    SGM::Surface ExtrudeID2 = SGM::CreateExtrudeSurface(rResult, SGM::UnitVector3D(0,0,1), NUBID);

    SGM::DeleteEntity(rResult, ExtrudeID1);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGM::Curve NUBCopyID = SGM::Curve(SGM::CopyEntity(rResult, NUBID).m_ID);

    SGM::DeleteEntity(rResult, ExtrudeID1);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    SGM::Point3D Origin(0.1, 0, 0);
    SGM::UnitVector3D Axis(0,1,0);
    SGM::Surface RevolveID1 = SGM::CreateRevolveSurface(rResult, Origin, Axis, NUBCopyID);
    SGM::Surface RevolveID2 = SGM::CreateRevolveSurface(rResult, Origin, Axis, NUBCopyID);

    SGM::DeleteEntity(rResult, RevolveID2);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);
    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGM::DeleteEntity(rResult, RevolveID1);
    EXPECT_EQ(rResult.GetResult(), SGM::ResultTypeOK);

    EXPECT_TRUE(SGMTesting::CheckEntityAndPrintLog(rResult, SGM::Thing()));

    SGMTesting::ReleaseTestThing(pThing);
}

//#pragma clang diagnostic pop
