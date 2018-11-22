#include <gtest/gtest.h>
#include "test_utility.h"

#include "SGMInterrogate.h"
#include "SGMTopology.h"
#include "SGMComplex.h"
#include "SGMIntersector.h"

///////////////////////////////////////////////////////////////////////////////
//
// Tests that import and read selected files from our OUO data directory
//
///////////////////////////////////////////////////////////////////////////////

// Import and OUO model and EXPECT ResultTypeOK
void expect_import_ouo_success(std::string const &file_name, SGM::Result &rResult)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    std::string file_path = get_models_ouo_file_path(file_name);
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);
}


// Import and model and EXPECT ResultTypeOK
void expect_import_success(std::string const &file_name, SGM::Result &rResult)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;

    std::string file_path = get_models_file_path(file_name);
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    ASSERT_EQ(resultType, SGM::ResultTypeOK);
}


// CheckEntity on a model (result), that has already been imported (from file_path);
// write any non-empty log file messages to failure message
void expect_check_success(SGM::Result &rResult)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(rResult,SGM::Thing(),Options,aLog);
    if (!aLog.empty())
        {
        std::string message;
        for (std::string &log_item: aLog)
            message += log_item;
        FAIL() << message;
        }
}

// Import a file from modles directory and check all entities
void expect_import_check_success(std::string const &file_name)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_success(file_name, rResult);
    expect_check_success(rResult);
    SGMTesting::ReleaseTestThing(pThing);
    }

// Import a file from OUO directory and check all entities
void expect_import_ouo_check_success(std::string const &ouo_file_name)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_ouo_success(ouo_file_name, rResult);
    expect_check_success(rResult);
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, inport_txt_points)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::string sFile=get_models_path();
    sFile+="/Points.txt";
    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> aLog;
    SGM::ReadFile(rResult,sFile,aEntities,aLog,SGM::TranslatorOptions());

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, scan_directory)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    std::string sDirectory=get_models_path();
    sDirectory+="/ScanTest";
    std::string sOutputName=sDirectory;
    sOutputName+="Output.txt";
    SGM::ScanDirectory(rResult,sDirectory,sOutputName);
    SGMTesting::ReleaseTestThing(pThing);
}


TEST(models_single_check, import_torus_apple)
{
    const char* file_name = "Working On/Torus Apple.stp";
    SCOPED_TRACE(file_name);
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
     
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;

    std::string file_path = get_models_file_path(file_name);
    SGM::ReadFile(rResult, file_path, entities, log, options);

    // TODO this parts needs to check also.  PRS

    SGMTesting::ReleaseTestThing(pThing);
}


TEST(models_single_check, import_flexyhose)
{
    const char* file_name = "flexyhose.stp";
    SCOPED_TRACE(file_name);
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;

    std::string file_path = get_models_file_path(file_name);
    SGM::ReadFile(rResult, file_path, entities, log, options);

    // TODO this parts needs to check also.  PRS

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, import_Closed_Kelvin_BCC_4_4_4)
{
    const char* file_name = "Closed_Kelvin_BCC_4_4_4.sgm";
    SCOPED_TRACE(file_name);
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_success(file_name, rResult);
    expect_check_success(rResult);

    std::set<SGM::Complex> sComplexes;
    SGM::Thing ThingID;
    SGM::FindComplexes(rResult,ThingID,sComplexes);
    SGM::Complex ComplexID=*(sComplexes.begin());
    SGM::CoverComplex(rResult,ComplexID);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, import_check_OUO_Cone_definition)
    {
    const char* file_name = "OUO_Cone_definition.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
    }

TEST(models_single_check, DISABLED_import_check_OUO_TSLhousingGeom)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_TSLhousingGeom.stp");
}

TEST(models_single_check, import_check_OUO_full_model_volume1)
{
    const char* file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_single_check, import_check_OUO_grv_geom)
{
    const char* file_name = "OUO_grv_geom.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_OUO_ZGeom) // TODO: Lots of faces.
{
    const char* file_name = "OUO_ZGeom.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0001_Bbat)
{
    const char* file_name = "Glom4/0001-_Bbat.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_points_text_file)
{
    const char* file_name = "points.txt";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}


TEST(models_single_check, import_check_glom4_0002_Bknob) 
{
    const char* file_name = "Glom4/0002-_Bknob.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0003_Bbracket)
{
    const char* file_name = "Glom4/0003-_Bbracket.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0004_BSump1)
{
    const char* file_name = "Glom4/0004-_BSump1.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0005_BSump2)
{
    const char* file_name = "Glom4/0005-_BSump2.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0006_Bangle)
{
    const char* file_name = "Glom4/0006-_Bangle.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0007_Bflange)
{
    const char* file_name = "Glom4/0007-_Bflange.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0008_Bkey) 
{
    const char* file_name = "Glom4/0008-_Bkey.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0009_Bcam) 
{
    const char* file_name = "Glom4/0009-_Bcam.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0010_Bsteer) // TODO: Face 388 infinite call
{
    const char* file_name = "Glom4/0010-_Bsteer.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0011_Bdice) 
{
    const char* file_name = "Glom4/0011-_Bdice.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0012_Bgear)
{
    const char* file_name = "Glom4/0012-_Bgear.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0013_Bbellows)
{
    const char* file_name = "Glom4/0013-_Bbellows.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0014_Bplate) // TODO: Lots of faces.
{
    const char* file_name = "Glom4/0014-_Bplate.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0015_Bpipe)
{
    const char* file_name = "Glom4/0015-_Bpipe.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0016_Bspring) 
{
    const char* file_name = "Glom4/0016-_Bspring.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0017_Bcube) 
{
    const char* file_name = "Glom4/0017-_Bcube.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0018_Bhinkey) // TODO: Lots of faces off.
{
    const char* file_name = "Glom4/0018-_Bhinkey.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0019_Bhinkey_A)
{
    const char* file_name = "Glom4/0019-_Bhinkey_A.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, ACISSphereGeometry_arbitraryCenter)
{
    const char* file_name = "ACISSphereGeometry_arbitraryCenter.stp";
    SCOPED_TRACE(file_name);
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_success(file_name, rResult);
    expect_check_success(rResult);
    
    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
    SGM::Point3D Pos(2.1,2.9,-3.89);
    bool bAnswer=SGM::PointInEntity(rResult,Pos,*(sVolumes.begin()));
    EXPECT_TRUE(bAnswer);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, ACISBrickWithImprintedEllipses)
{
    const char* file_name = "ACISBrickWithImprintedEllipses.stp";
    SCOPED_TRACE(file_name);
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_success(file_name, rResult);
    expect_check_success(rResult);

    SGM::Edge EdgeID1(23),EdgeID2(24);
    size_t Index1,Index2;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(81);
    for(Index1=1;Index1<10;++Index1)
        {
        for(Index2=1;Index2<10;++Index2)
            {
            SGM::Point3D Pos(5.0,Index1-5.0,Index2-5.0);
            aPoints.push_back(Pos);
            SGM::Entity CloseEntity;
            SGM::Point3D ClosePos1,ClosePos2;
            SGM::FindClosestPointOnEntity(rResult,Pos,EdgeID1,ClosePos1,CloseEntity);
            double dDist1=Pos.DistanceSquared(ClosePos1);
            SGM::FindClosestPointOnEntity(rResult,Pos,EdgeID2,ClosePos2,CloseEntity);
            double dDist2=Pos.DistanceSquared(ClosePos2);
            SGM::Point3D CPos=dDist1<dDist2 ? ClosePos1 : ClosePos2;
            SGM::CreateLinearEdge(rResult,Pos,CPos);
            }
        }
    SGM::CreatePoints(rResult,aPoints);

    //SGM::Edge EdgeID1(23);
    //SGM::Point3D Pos(5.0,-5.0,5.0);
    //SGM::Entity CloseEntity;
    //SGM::Point3D ClosePos1;
    //SGM::FindClosestPointOnEntity(rResult,Pos,EdgeID1,ClosePos1,CloseEntity);
    //SGM::CreateLinearEdge(rResult,Pos,ClosePos1);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, ACISNotchedBrickGeometry)
{
    const char* file_name = "ACISNotchedBrickGeometry.stp";
    SCOPED_TRACE(file_name);
    
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_success(file_name, rResult);
    expect_check_success(rResult);

    std::set<SGM::Body> sBodies;
    SGM::FindBodies(rResult,SGM::Thing(),sBodies);
    SGM::Body BodyID=*(sBodies.begin());
    int Index1;
    std::vector<SGM::Point3D> aPoints,aPoints2;
    aPoints2.reserve(6);
    for(Index1=-5;Index1<1;++Index1)
        {
        SGM::Point3D Pos(Index1,0.0,-1.0);
        aPoints2.push_back(Pos);
        bool bInBody=SGM::PointInEntity(rResult,Pos,BodyID);
        EXPECT_FALSE(bInBody);

        SGM::Point3D Pos2(Index1,0.0,0.25);
        aPoints.push_back(Pos2);
        bool bInBody2=SGM::PointInEntity(rResult,Pos2,BodyID);
        EXPECT_TRUE(bInBody2);
        }
    SGM::CreatePoints(rResult,aPoints);
    SGM::CreatePoints(rResult,aPoints2);
    
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, check_plane_circle_consistent_with_cylinder_line_intersections)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    const double tolerance = 1e-12;

    expect_import_success("half_annulus.stp", rResult);

    SGM::Point3D left_coords_top_edge  = {4.97552009369256, 0.4924115623322998, 0.5};
    SGM::Point3D left_coords_face_node = left_coords_top_edge; //{4.97552009369256, 0.4924115623322996, 5.677036079487535e-21};

    // top edge
    std::vector<SGM::Point3D> aPointsEdgePlane;
    std::vector<SGM::IntersectionType> aTypesEdgePlane;
    //Euclid::Point3D plane_normal = {-0.1032603999127515, 0.9946543569551478, -3.312864505906003e-16};
    SGM::UnitVector3D plane_normal = {-0.1032603999127515, 0.9946543569551478, 0}; //-3.312864505906003e-16};
    {
        size_t edge_id = 24; //40;

        SGM::IntersectEdgeAndPlane(rResult, edge_id, left_coords_top_edge, plane_normal, aPointsEdgePlane, aTypesEdgePlane);
    }

    // face node
    std::vector<SGM::Point3D> aPointsFaceLine;
    std::vector<SGM::IntersectionType> aTypesFaceLine;
    //Euclid::Point3D line_vect = {0.9946543569679107, 0.1032603997898127, -1.270825582042912e-14};
    SGM::UnitVector3D line_vect = {plane_normal[1],-plane_normal[0],plane_normal[2]}; // choose orthog to plane_normals
    {
        size_t surf_id = 7; //26;

        bool bUseWholeLine = true;
        SGM::RayFire(rResult, left_coords_face_node, line_vect, surf_id, aPointsFaceLine, aTypesFaceLine, tolerance, bUseWholeLine);
    }

    const double dot_prod = plane_normal[0]*line_vect[0]+plane_normal[1]*line_vect[1]+plane_normal[2]*line_vect[2];

    EXPECT_NEAR(dot_prod, 0.0, tolerance);
    EXPECT_EQ(1, aPointsEdgePlane.size());
    EXPECT_EQ(1, aPointsFaceLine.size());
    EXPECT_NEAR(aPointsEdgePlane[0].m_x, aPointsFaceLine[0].m_x, tolerance);
    EXPECT_NEAR(aPointsEdgePlane[0].m_x, aPointsFaceLine[0].m_x, tolerance);
}


TEST(intersection_check, intersect_line_and_extrude)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    expect_import_success("ACISSplineVolumeGeometry.stp", rResult);
    
    SGM::Point3D StartPos(-10,-10,-10);
    SGM::Point3D EndPos(10,10,10);
    SGM::Edge EdgeID=SGM::CreateLinearEdge(rResult,StartPos,EndPos);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::set<SGM::Face> sFaces;
    SGM::FindFaces(rResult,SGM::Thing(),sFaces);
    SGM::Face FaceID=*(sFaces.begin());
    SGM::Curve IDCurve=SGM::GetCurveOfEdge(rResult,EdgeID);
    SGM::Surface IDSurface=SGM::GetSurfaceOfFace(rResult,FaceID);
    SGM::IntersectCurveAndSurface(rResult,IDCurve,IDSurface,aPoints,aTypes);

    SGM::CreatePoints(rResult,aPoints);

    EXPECT_EQ(3, aPoints.size());

    SGMTesting::ReleaseTestThing(pThing); 
}

