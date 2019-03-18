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

#define SGM_TIMER 
#include "Util/timer.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(topology_check, find_adjacent_faces )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    auto iter=sFaces.begin();
    SGM::Face FaceID1=*iter;
    std::vector<SGM::Face> aFaces;
    EXPECT_EQ(SGM::FindAdjacentFaces(rResult,FaceID1,aFaces),5U);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(topology_check, find_common_edges )
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    auto iter=sFaces.begin();
    SGM::Face FaceID1=*iter;
    ++iter;
    SGM::Face FaceID2=*iter;
    std::vector<SGM::Edge> aEdges;
    EXPECT_EQ(SGM::FindCommonEdgesFromFaces(rResult,FaceID1,FaceID2,aEdges),1U);

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(topology_check, edge_params)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,1,1));
    SGM::GetEdgeParams(rResult,EdgeID);
    SGM::GetStartPointOfEdge(rResult,EdgeID);
    SGM::GetEndPointOfEdge(rResult,EdgeID);
    
    SGMTesting::ReleaseTestThing(pThing);
}


TEST(topology_check, create_sheet_body_with_edges)
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

#ifdef __clang__
#pragma clang diagnostic pop
#endif
