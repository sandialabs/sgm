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
#include "SGMPolygon.h"
#include "SGMTriangle.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

bool check_entity_verbose(SGM::Result &rResult, const SGM::Entity &entity)
    {
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    bool bValue = SGM::CheckEntity(rResult,entity,Options,aCheckStrings);
    for (auto & String: aCheckStrings)
        std::cout << String << std::endl;
    return bValue;
    }

TEST(repair, block_block)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,5),SGM::Point3D(15,15,15));
    std::vector<SGM::Body> aBodies;
    aBodies.push_back(BlockID1);
    aBodies.push_back(BlockID2);
    std::vector<SGM::Edge> aEdges;
    SGM::FindOverLappingEdges(rResult,aBodies,aEdges);

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(repair, spring_bottom_cap)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing); 
//
//    std::vector<SGM::Entity> entities;
//    std::vector<std::string> log;
//    SGM::TranslatorOptions const options;
//    std::string file_path1 = get_models_file_path("RepairTest/Shock-BottomCap.stp");
//    SGM::ReadFile(rResult, file_path1, entities, log, options);
//
//    std::string file_path2 = get_models_file_path("RepairTest/Shock-Spring.stp");
//    SGM::ReadFile(rResult, file_path2, entities, log, options);
//
//    std::vector<SGM::Body> aBodies;
//    aBodies.push_back(SGM::Body(entities[0].m_ID));
//    aBodies.push_back(SGM::Body(entities[1].m_ID));
//
//    std::vector<SGM::Edge> aEdges;
//    SGM::FindOverLappingEdges(rResult,aBodies,aEdges);
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

//TEST(repair, spring_bottom_cap_faces)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing); 
//
//    std::vector<SGM::Entity> entities;
//    std::vector<std::string> log;
//    SGM::TranslatorOptions const options;
//    std::string file_path1 = get_models_file_path("RepairTest/Shock-Faces484-454.stp");
//    SGM::ReadFile(rResult, file_path1, entities, log, options);
//
//    std::vector<SGM::Body> aBodies;
//    aBodies.push_back(SGM::Body(entities[0].m_ID));
//    aBodies.push_back(SGM::Body(entities[1].m_ID));
//
//    std::vector<SGM::Edge> aEdges;
//    SGM::FindOverLappingEdges(rResult,aBodies,aEdges);
//
//    SGMTesting::ReleaseTestThing(pThing);
//}


//TEST(modify, block_block_slot)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing); 
//
//    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
//    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(4,0,8),SGM::Point3D(6,10,10));
//
//    SGM::SubtractBodies(rResult,BlockID1,BlockID2);
//
//    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(modify, block_block_slot_face2)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,10));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID1,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    //SGM::Edge EdgeID1=SGM::CreateLinearEdge(rResult,SGM::Point3D(4,0,10),SGM::Point3D(4,10,10));
    SGM::Edge EdgeID2=SGM::CreateLinearEdge(rResult,SGM::Point3D(4,0,10),SGM::Point3D(6,0,10));
    
    //SGM::ImprintEdgeOnFace(rResult,EdgeID1,FaceID);
    SGM::ImprintEdgeOnFace(rResult,EdgeID2,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

//TEST(modify, block_block_slot_face)
//{
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing); 
//
//    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,10));
//    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(4,0,10),SGM::Point3D(6,10,10));
//
//    SGM::SubtractBodies(rResult,BlockID1,BlockID2);
//
//    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));
//
//    SGMTesting::ReleaseTestThing(pThing);
//}

TEST(modify, block_block_one_eighth_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,5),SGM::Point3D(15,15,15));

    SGM::IntersectBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_block_one_eighth_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,5),SGM::Point3D(15,15,15));

    SGM::SubtractBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_block_one_eighth_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(5,5,5),SGM::Point3D(15,15,15));

    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_square_coincident_face_vertex_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Transform3D Trans1(SGM::Point3D(0,0,0),SGM::UnitVector3D(0,1,0),SGM_HALF_PI);
    SGM::Transform3D Trans2(SGM::Point3D(0,0,0),SGM::UnitVector3D(1,0,0),-SGM_PI*0.25);
    SGM::Transform3D Trans3(SGM::Vector3D(5,5,0));
    SGM::TransformEntity(rResult,Trans1,BlockID2);
    SGM::TransformEntity(rResult,Trans2,BlockID2);
    SGM::TransformEntity(rResult,Trans3,BlockID2);

    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_square_coincident_edge_vertex_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Transform3D Trans1(SGM::Point3D(5,5,0),SGM::UnitVector3D(0,0,1),SGM_PI*0.25);
    SGM::Transform3D Trans2(SGM::Vector3D(10+(sqrt(2)-1)*5,0,0));
    SGM::TransformEntity(rResult,Trans1,BlockID2);
    SGM::TransformEntity(rResult,Trans2,BlockID2);

    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_square_over_coincident_edge_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(10,-5,0),SGM::Point3D(20,15,0));
    
    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_square_half_coincident_edge_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(10,5,0),SGM::Point3D(20,15,0));
    
    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_block_coincident_vertex_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(10,10,10),SGM::Point3D(20,20,20));
    
    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_block_coincident_edge_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(10,0,10),SGM::Point3D(20,10,20));
    
    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_block_coincident_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,20));

    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_sqaure_full_coincident_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(10,10,0),SGM::Point3D(0,0,0));

    SGM::UniteBodies(rResult,BlockID1,BlockID2);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_sqaure_coincident_edge_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID1=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,0,10));
    SGM::Body BlockID2=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,0,20));
    SGM::Body BlockID3=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,10));

    SGM::UniteBodies(rResult,BlockID1,BlockID2);
    SGM::UniteBodies(rResult,BlockID1,BlockID3);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_sphere_unite)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    SGM::UniteBodies(rResult,BlockID,SphereID);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_sphere_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    SGM::SubtractBodies(rResult,BlockID,SphereID);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, sphere_block_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    SGM::SubtractBodies(rResult,SphereID,BlockID);

    EXPECT_TRUE(check_entity_verbose(rResult,SphereID));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_sphere_intersect)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    SGM::IntersectBodies(rResult,BlockID,SphereID);

    EXPECT_TRUE(check_entity_verbose(rResult,BlockID));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, imprint_block_on_sphere)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,5));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    std::set<SGM::Face> sSphereFaces;
    SGM::FindFaces(rResult,SphereID,sSphereFaces);
    SGM::Face SphereFaceID=*(sSphereFaces.begin());
    SGM::Surface SphereSurfaceID=SGM::GetSurfaceOfFace(rResult,SphereFaceID);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID,sFaces);
    std::vector<SGM::Edge> aEdges;
    for(SGM::Face FaceID : sFaces)
        {
        SGM::Surface PlaneID=SGM::GetSurfaceOfFace(rResult,FaceID);
        std::vector<SGM::Curve> aCurves;
        SGM::IntersectSurfaces(rResult,SphereSurfaceID,PlaneID,aCurves);
        SGM::TrimCurveWithFace(rResult,aCurves[0],FaceID,aEdges);
        }
    SGM::DeleteEntity(rResult,BlockID);

    size_t nEdges=aEdges.size();
    size_t Index1;
    for(Index1=0;Index1<nEdges;++Index1)
        {
        SGM::Edge EdgeID=aEdges[Index1];
        SGM::ImprintEdgeOnFace(rResult,EdgeID,SphereFaceID);
        }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, DISABLED_close_island_imprint)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),2);
    SGM::Edge Edge1=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,0,0),SGM::Point3D(1,0,0));
    SGM::Edge Edge2=SGM::CreateLinearEdge(rResult,SGM::Point3D(1,0,0),SGM::Point3D(0,1,0));
    SGM::Edge Edge3=SGM::CreateLinearEdge(rResult,SGM::Point3D(0,1,0),SGM::Point3D(0,0,0));

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID1,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    SGM::ImprintEdgeOnFace(rResult,Edge1,FaceID);
    SGM::ImprintEdgeOnFace(rResult,Edge2,FaceID);
    SGM::ImprintEdgeOnFace(rResult,Edge3,FaceID);

    EXPECT_TRUE(check_entity_verbose(rResult,BodyID1));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, point_intersection_body_and_vertex)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Body BodyID2=SGM::CreateDisk(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(1,0,0),1);

    SGM::ImprintPoint(rResult,SGM::Point3D(0,0,0),BodyID2);
    SGM::ImprintBodies(rResult,BodyID1,BodyID2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, point_intersection_body_and_edge)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BodyID1=SGM::CreateDisk(rResult,SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),1);
    SGM::Body BodyID2=SGM::CreateDisk(rResult,SGM::Point3D(0,0,1),SGM::UnitVector3D(1,0,0),1);

    SGM::ImprintBodies(rResult,BodyID1,BodyID2);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, imprint_point_on_body_face)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));

    SGM::ImprintPoint(rResult,SGM::Point3D(5,5,10),BlockID);
    SGM::ImprintPoint(rResult,SGM::Point3D(5,5,0),BlockID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, imprint_point_on_body_edge)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,0));

    SGM::ImprintPoint(rResult,SGM::Point3D(5,0,0),BlockID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_top_face_sphere_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,10));
    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(5,5,5),7.5);

    SGM::SubtractBodies(rResult,BlockID,SphereID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, square_circle_imprint)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,10),SGM::Point3D(10,10,10));
    SGM::Curve CurveID=SGM::CreateCircle(rResult,SGM::Point3D(5,5,10),SGM::UnitVector3D(0,0,-1),6);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);

    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, block_cylinder_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,5));
    SGM::Body CylinderID=SGM::CreateCylinder(rResult,SGM::Point3D(5,5,-1),SGM::Point3D(5,5,6),2);

    SGM::SubtractBodies(rResult,BlockID,CylinderID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, trim_curve_cylinder)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::Body CylinderID=SGM::CreateCylinder(rResult,SGM::Point3D(5,5,0),SGM::Point3D(5,5,5),2);
    SGM::Curve CurveID=SGM::CreateCircle(rResult,SGM::Point3D(5,5,0),SGM::UnitVector3D(0,0,1),2);
    std::vector<SGM::Edge> aEdges;
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,CylinderID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::TrimCurveWithFace(rResult,CurveID,FaceID,aEdges);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, sphere_NUB_subtract)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    std::vector<std::vector<SGM::Point3D> > aaPoints;
    size_t Index1,Index2;
    double dStep=SGM_PI/8.0;
    for(Index1=0;Index1<49;++Index1)
        {
        double x=Index1*dStep;
        std::vector<SGM::Point3D> aPoints;
        for(Index2=0;Index2<49;++Index2)
            {
            double y=Index2*dStep;
            double z=sin(x)*cos(y);
            SGM::Point3D Pos(x,y,z);
            aPoints.push_back(Pos);
            }
        aaPoints.push_back(aPoints);
        }
    SGM::Surface SurfaceID=SGM::CreateNUBSurface(rResult,aaPoints);
    SGM::Body SheetID=SGM::CreateSheetBody(rResult,SurfaceID,SGM::GetDomainOfSurface(rResult,SurfaceID));
    SGM::Body BodyID=SGM::CreateSphere(rResult,SGM::Point3D(10,10,0),3);

    SGM::SubtractBodies(rResult,SheetID,BodyID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, sphere_NUB_imprint_NUB)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    std::vector<std::vector<SGM::Point3D> > aaPoints;
    size_t Index1,Index2;
    double dStep=SGM_PI/8.0;
    for(Index1=0;Index1<49;++Index1)
        {
        double x=Index1*dStep;
        std::vector<SGM::Point3D> aPoints;
        for(Index2=0;Index2<49;++Index2)
            {
            double y=Index2*dStep;
            double z=sin(x)*cos(y);
            SGM::Point3D Pos(x,y,z);
            aPoints.push_back(Pos);
            }
        aaPoints.push_back(aPoints);
        }
    SGM::Surface SurfaceID=SGM::CreateNUBSurface(rResult,aaPoints);
    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(10,10,0),3);
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SurfaceID,SphereID,aCurves);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,aCurves[0]);
    
    SGM::Body SheetID=SGM::CreateSheetBody(rResult,SurfaceID,SGM::GetDomainOfSurface(rResult,SurfaceID));
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,SheetID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, sphere_NUB_imprint_sphere)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    std::vector<std::vector<SGM::Point3D> > aaPoints;
    size_t Index1,Index2;
    double dStep=SGM_PI/8.0;
    for(Index1=0;Index1<49;++Index1)
        {
        double x=Index1*dStep;
        std::vector<SGM::Point3D> aPoints;
        for(Index2=0;Index2<49;++Index2)
            {
            double y=Index2*dStep;
            double z=sin(x)*cos(y);
            SGM::Point3D Pos(x,y,z);
            aPoints.push_back(Pos);
            }
        aaPoints.push_back(aPoints);
        }
    SGM::Surface SurfaceID=SGM::CreateNUBSurface(rResult,aaPoints);
    SGM::Surface SphereID=SGM::CreateSphereSurface(rResult,SGM::Point3D(10,10,0),3);
    std::vector<SGM::Curve> aCurves;
    SGM::IntersectSurfaces(rResult,SurfaceID,SphereID,aCurves);
    SGM::DeleteEntity(rResult,SphereID);
    SGM::DeleteEntity(rResult,SurfaceID);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,aCurves[0]);
    
    SGM::Body BodyID=SGM::CreateSphere(rResult,SGM::Point3D(10,10,0),3);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, tweak_face_ambiguous)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    SGM::CreateBlock(rResult,SGM::Point3D(0,0,0),SGM::Point3D(10,10,10));
    SGM::CreateSphere(rResult,SGM::Point3D(5,5,10),sqrt(2)*5);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(modify, tweak_face)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    std::vector<std::vector<SGM::Point3D> > aaPoints;
    size_t Index1,Index2;
    double dStep=SGM_PI/8.0;
    for(Index1=0;Index1<49;++Index1)
        {
        double x=Index1*dStep;
        std::vector<SGM::Point3D> aPoints;
        for(Index2=0;Index2<49;++Index2)
            {
            double y=Index2*dStep;
            double z=sin(x)*cos(y);
            SGM::Point3D Pos(x,y,z);
            aPoints.push_back(Pos);
            }
        aaPoints.push_back(aPoints);
        }
    SGM::Surface SurfaceID=SGM::CreateNUBSurface(rResult,aaPoints);
    SGM::Body SheetID=SGM::CreateSheetBody(rResult,SurfaceID,SGM::GetDomainOfSurface(rResult,SurfaceID));

    SGM::Body BlockID=SGM::CreateBlock(rResult,SGM::Point3D(5,5,-10),SGM::Point3D(15,15,5));

    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BlockID,sFaces);
    SGM::Face FaceID=*(--sFaces.end());
    SGM::TweakFace(rResult,FaceID,SurfaceID);
    SGM::DeleteEntity(rResult,SheetID);

    SGM::Body SphereID=SGM::CreateSphere(rResult,SGM::Point3D(10,10,0),3);
    SGM::SubtractBodies(rResult,BlockID,SphereID);
    
    EXPECT_TRUE(check_entity_verbose(rResult,BlockID));

    SGMTesting::ReleaseTestThing(pThing);
}

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

TEST(boolean_check, imprint_circle_on_cylinder)
    {
    // Boolean of two disks Peninsula
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Body BodyID=SGM::CreateCylinder(rResult,SGM::Point3D(0,0,0),SGM::Point3D(0,0,8),2);
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,BodyID,sFaces);
    SGM::Face FaceID=*(sFaces.begin());

    SGM::Curve CurveID=SGM::CreateCircle(rResult,SGM::Point3D(0,0,4),SGM::UnitVector3D(0,0,1),2);
    SGM::Edge EdgeID=SGM::CreateEdge(rResult,CurveID);

    SGM::ImprintEdgeOnFace(rResult,EdgeID,FaceID);
    
    SGMTesting::ReleaseTestThing(pThing);
    }

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
    EXPECT_EQ(sFaces.size(), 2U);

    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,KeepID,sEdges);
    EXPECT_EQ(sEdges.size(), 3U);

    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,KeepID,sVertices);
    EXPECT_EQ(sVertices.size(), 2U);
    
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
    EXPECT_EQ(sFaces.size(), 3U);
    
    std::set<SGM::Edge> sEdges;
    SGM::FindEdges(rResult,KeepID,sEdges);
    EXPECT_EQ(sEdges.size(), 4U);
    
    std::set<SGM::Vertex> sVertices;
    SGM::FindVertices(rResult,KeepID,sVertices);
    EXPECT_EQ(sVertices.size(), 2U);
    
    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    EXPECT_TRUE(SGM::CheckEntity(rResult,KeepID,Options,aCheckStrings));
    
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, DISABLED_Imprinting_Atoll_Bridge_Edge)
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

    EXPECT_TRUE(check_entity_verbose(rResult,Sphere1));

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

    EXPECT_TRUE(check_entity_verbose(rResult,Sphere2));

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(boolean_check, DISABLED_sphere_sphere_unite)
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

#ifdef __clang__
#pragma clang diagnostic pop
#endif
