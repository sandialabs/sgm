#include <gtest/gtest.h>
#include "test_utility.h"

#include "SGMInterrogate.h"
#include "SGMTopology.h"
#include "SGMComplex.h"
#include "SGMIntersector.h"
#include "SGMEntityFunctions.h"
#include "SGMChecker.h"
#include "SGMPrimitives.h"
#include "SGMTranslators.h"
#include "SGMModify.h"

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
            {
            message += log_item;
            message += "\n";
            }
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

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(repair, auto_match)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::string sFile=get_models_path();
    sFile+="/Plato/Panel1.STEP";
    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> aLog;
    SGM::ReadFile(rResult,sFile,aEntities,aLog,SGM::TranslatorOptions());

    SGM::Body BodyID0=SGM::Body(aEntities[0].m_ID);
    SGM::Body BodyID1=SGM::Body(aEntities[1].m_ID);
    std::vector<SGM::Body> aBodies;
    aBodies.push_back(BodyID0);
    aBodies.push_back(BodyID1);
    SGM::Repair(rResult,aBodies);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_single_check, import_txt_points)
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

TEST(models_single_check, import_vertex3)
{
    const char* file_name = "vertex3.stp";
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

TEST(models_single_check, DISABLED_import_check_OUO_Cone_definition)
    {
    const char* file_name = "OUO_Cone_definition.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
    }

TEST(models_single_check, DISABLED_import_check_OUO_TSLhousingGeom)
{
    //std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_TSLhousingGeom.stp");
}

TEST(models_single_check, import_check_OUO_full_model_volume1)
{
    const char* file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(speed_check, DISABLED_point)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    const char* ouo_file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(ouo_file_name);
    expect_import_ouo_success(ouo_file_name, rResult);

    SGM::Point3D TestPoint(-2.99338449336725,  0.366425467792747,  -1.64718061859023);

    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
    SGM::Volume VolumeID = *(sVolumes.begin());

    rResult.SetDebugFlag(6);
    std::vector<double> aData;
    aData.push_back(1);
    aData.push_back(0);
    aData.push_back(0);
    rResult.SetDebugData(aData);

    bool bValue = PointInEntity(rResult,TestPoint,VolumeID);
    
    std::vector<double> aData2=rResult.GetDebugData();
    SGM::CreateLinearEdge(rResult,TestPoint,TestPoint+6*SGM::Vector3D(aData2[0],aData2[1],aData2[2]));

    EXPECT_FALSE(bValue);

    SGMTesting::ReleaseTestThing(pThing);
    }

#if 0
TEST(speed_check, point_in_volume_OUO_full_model_volume1)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    enum {DIRECTION_X, DIRECTION_Y, DIRECTION_Z, DIRECTION_ALL};

    const int RAY_FIRE_DIRECTION = DIRECTION_X;// DIRECTION_ALL;

    const char* ouo_file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(ouo_file_name);
    expect_import_ouo_success(ouo_file_name, rResult);

    std::cout << "point_in_volume_OUO_full_model_volume1:" << std::endl;

    SGM::Interval3D Bounds = SGM::GetBoundingBox(rResult,SGM::Thing());

    std::cout << "  Bounds = ([" << Bounds.m_XDomain.m_dMin << "," << Bounds.m_XDomain.m_dMax << "]," <<
              "[" << Bounds.m_YDomain.m_dMin << "," << Bounds.m_YDomain.m_dMax << "]," <<
              "[" << Bounds.m_ZDomain.m_dMin << "," << Bounds.m_ZDomain.m_dMax << "])" << std::endl;

    // make a regular grid of equally spaced points that covers the bounding box

    // start and end slightly outside the bounding box
    double dExtraX = (Bounds.m_XDomain.m_dMax - Bounds.m_XDomain.m_dMin)/10.;
    double dExtraY = (Bounds.m_YDomain.m_dMax - Bounds.m_YDomain.m_dMin)/10.;
    double dExtraZ = (Bounds.m_ZDomain.m_dMax - Bounds.m_ZDomain.m_dMin)/10.;

    double dStartX = Bounds.m_XDomain.m_dMin - dExtraX;
    double dStartY = Bounds.m_YDomain.m_dMin - dExtraY;
    double dStartZ = Bounds.m_ZDomain.m_dMin - dExtraZ;

    double dEndX = Bounds.m_XDomain.m_dMax + dExtraX;
    double dEndY = Bounds.m_YDomain.m_dMax + dExtraY;
    double dEndZ = Bounds.m_ZDomain.m_dMax + dExtraZ;

    // equal spacing in all three directions
    double dIncrement = std::min({std::abs(dEndX - dStartX),
                                  std::abs(dEndY - dStartY),
                                  std::abs(dEndZ - dStartZ)});
    dIncrement /= 100.;

    std::vector<SGM::Point3D> aPoints;
    std::vector<unsigned> aEmpty;

    SGM::Vector3D FireDirection;

#if 0

    // a point that should be outside
    SGM::Point3D TestPoint(-2.9933844933672531,0.02004745364405721,-1.1948043844891936);
    aPoints.emplace_back(TestPoint);

    std::vector<SGM::Point3D> aRayIntersections =
    {
        {-2.6419999999999999,
        0.02004745364405721,
        -1.1948043844891936},

        {-2.1430009479311889,
        0.02004745364405721,
        -1.1948043844891936},

        {-1.7357589903911748,
        0.020047453644267937,
        -1.1948043844893057},

        {-0.51990540249910955,
        0.02004745364405721,
        -1.1948043844891936},

        {0.53116701498726382,
        0.02004745364405721,
        -1.1948043844891936},

        {0.59683298501273629,
        0.02004745364405721,
        -1.1948043844891936},
        
        {0.73720977951579103,
        0.02004745364405721,
        -1.1948043844891936}
    };


    // a point that should be outside
//    SGM::Point3D TestPoint(-1.7956696576099629, -0.28134119301999427, -0.29265438044251146);
//    aPoints.emplace_back(TestPoint);
//
//    std::vector<SGM::Point3D> aRayIntersections =
//    {
////        {-1.7956696576099629,
////         0.13,
////         -0.29265438044251146},
////
////        {-1.7956696576099629,
////        0.13,
////        -0.29265438044251146},
//
//        {-1.7956696576099629,
//        1.9515,
//        -0.29265438044251146}
//    };

    // a point that should be outside
//    SGM::Point3D TestPoint(0.68958862658642339,-0.28134119301999427,-1.0412261527908198);
//    aPoints.emplace_back(TestPoint);
//    SGM::CreateLinearEdge(rResult,TestPoint, TestPoint+SGM::Vector3D(0,9,0));
//
//    SGM::Curve CurveID = SGM::CreateLine(rResult,TestPoint, SGM::UnitVector3D(0,9,0));
//    SGM::Surface SurfaceA = SGM::GetSurfaceOfFace(rResult,SGM::Face(65));
//    SGM::Surface SurfaceB = SGM::GetSurfaceOfFace(rResult,SGM::Face(66));
//    std::vector<SGM::Point3D> aRayIntersections;
//    {
//        std::vector<SGM::Point3D> aSurfacePoints;
//        std::vector<SGM::IntersectionType> aSurfaceIntersectionTypes;
//        size_t nPoints = SGM::IntersectCurveAndSurface(rResult, CurveID, SurfaceA, aSurfacePoints, aSurfaceIntersectionTypes);
//        std::cout << "point_in_volume_OUO_full_model_volume1: SurfaceA has nPoints=" << nPoints << std::endl;
//        aRayIntersections.insert(aRayIntersections.end(),aSurfacePoints.begin(),aSurfacePoints.end());
//    }
//    {
//        std::vector<SGM::Point3D> aSurfacePoints;
//        std::vector<SGM::IntersectionType> aSurfaceIntersectionTypes;
//        size_t nPoints = SGM::IntersectCurveAndSurface(rResult, CurveID, SurfaceB, aSurfacePoints, aSurfaceIntersectionTypes);
//        std::cout << "point_in_volume_OUO_full_model_volume1: SurfaceB has nPoints=" << nPoints << std::endl;
//        aRayIntersections.insert(aRayIntersections.end(),aSurfacePoints.begin(),aSurfacePoints.end());
//    }

     SGM::CreateComplex(rResult,aRayIntersections,aEmpty,aEmpty);


#else

    switch(RAY_FIRE_DIRECTION)
        {
        case DIRECTION_X:
            {
            // points in the constant X plane
            FireDirection = {1.,0.,0.};
            double dX = dStartX;
            double dY = dStartY;
            while (dY < dEndY)
                {
                double dZ = dStartZ;
                while (dZ < dEndZ)
                    {
                    aPoints.emplace_back(dX,dY,dZ);
                    dZ += dIncrement;
                    }
                dY += dIncrement;
                }
            break;
            }
        case DIRECTION_Y:
            {
            // points in the constant Y plane
            FireDirection = {0.,1.,0.};
            double dX = dStartX;
            double dY = dStartY;
            while (dX < dEndX)
                {
                double dZ = dStartZ;
                while (dZ < dEndZ)
                    {
                    aPoints.emplace_back(dX,dY,dZ);
                    dZ += dIncrement;
                    }
                dX += dIncrement;
                }
            break;
            }
        case DIRECTION_Z:
            {
            // points in the constant Z plane
            FireDirection = {0.,0.,1.};
            double dX = dStartX;
            double dZ = dStartZ;
            while (dX < dEndX)
                {
                double dY = dStartY;
                while (dY < dEndY)
                    {
                    aPoints.emplace_back(dX,dY,dZ);
                    dY += dIncrement;
                    }
                dX += dIncrement;
                }
            break;
            }
        case DIRECTION_ALL:
            {
            // points in all planes
            FireDirection = {0.,0.,1.};
            double dX = dStartX;
            while (dX < dEndX)
                {
                double dY = dStartY;
                while (dY < dEndY)
                    {
                    double dZ = dStartZ;
                    while (dZ < dEndZ)
                        {
                        aPoints.emplace_back(dX, dY, dZ);
                        dZ += dIncrement;
                        }
                    dY += dIncrement;
                    }
                dX += dIncrement;
                }
            SGM::CreateComplex(rResult,aPoints,aEmpty,aEmpty);
            break;
            }
        }

    // set ray fire direction
    rResult.SetDebugFlag(6);
    rResult.SetDebugData({FireDirection.m_x, FireDirection.m_y, FireDirection.m_z});

    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
    SGM::Volume VolumeID = *(sVolumes.begin());

    std::vector<SGM::Point3D> aPointsInside;
    size_t Index1;
    size_t nPoints=aPoints.size();
    for(Index1=0;Index1<nPoints;++Index1)
        {
        if (SGM::PointInEntity(rResult,aPoints[Index1],VolumeID))
            {
            aPointsInside.push_back(aPoints[Index1]);
            }
        }

    SGM::CreateComplex(rResult,aPointsInside,aEmpty,aEmpty);
    if (RAY_FIRE_DIRECTION != DIRECTION_ALL)
        {
        std::cout << "Ray Direction = {" << FireDirection.m_x << ',' << FireDirection.m_y << ',' << FireDirection.m_z << '}' << std::endl;
        //std::cout.setf(std::ios::floatfield,std::ios::scientific);
        std::cout << std::setprecision(15);

        // make linear edges for the inside points
        for (SGM::Point3D const &Point : aPointsInside)
            {
            std::cout << '{' << std::setw(19) << Point[0] << ',' << std::setw(19) << Point[1] << ',' << std::setw(19) << Point[2] << '}' << std::endl;
            SGM::CreateLinearEdge(rResult,Point, Point+10*FireDirection);
            }
        }

    // print counts of face types that were hit
    rResult.PrintIntersectLineAndEntityCount();

    SGMTesting::ReleaseTestThing(pThing);
    }
#endif

//TEST(speed_check, dodecahedron_rays)
//    {
//    //  draw rays through vertices of dodecahedron
//    // where \phi is golden ratio
//    static const double A = 0.57735026918962576451; // 1 / \sqrt(3)
//    static const double B = 0.35682208977308993194; // 1 / (\phi * \sqrt(3)
//    static const double C = 0.93417235896271569645; // \phi / \sqrt(3)
//    EXPECT_NE(sqrt(3*(A*A)),1.0);
//    EXPECT_NE(sqrt(B*B+C*C),1.0);
//    static const double DodecahedronVertices[20][3] =
//        {
//            {  A,  A,  A}, { -A, -A, -A},
//            {  A,  A, -A}, { -A, -A,  A},
//            {  A, -A,  A}, { -A,  A, -A},
//            { -A,  A,  A}, {  A, -A, -A},
//            {  0,  B,  C}, {  0, -B, -C},
//            {  B,  C,  0}, { -B, -C,  0},
//            {  C,  0,  B}, { -C,  0, -B},
//            {  0,  B, -C}, {  0, -B,  C},
//            {  B, -C,  0}, { -B,  C,  0},
//            { -C,  0,  B}, {  C,  0, -B}
//        };
//    SGM::Point3D Origin(0,0,0);
//    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
//    SGM::Result rResult(pThing);
//    for (unsigned i = 0; i < 20; ++i)
//        {
//        const double *pData = DodecahedronVertices[i];
//        SGM::Point3D Vertex(pData[0], pData[1], pData[2]);
//        SGM::CreateLinearEdge(rResult, Origin, Vertex);
//        }
//    SGMTesting::ReleaseTestThing(pThing);
//    }
#endif

TEST(speed_check, DISABLED_points_in_volume_OUO_full_model_volume1)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    const char* ouo_file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(ouo_file_name);
    expect_import_ouo_success(ouo_file_name, rResult);

    SGM::Interval3D Bounds = SGM::GetBoundingBox(rResult,SGM::Thing());

    // make a regular grid of equally spaced points that covers the bounding box

    // start and end slightly outside the bounding box
    double dExtraX = (Bounds.m_XDomain.m_dMax - Bounds.m_XDomain.m_dMin)/10.;
    double dExtraY = (Bounds.m_YDomain.m_dMax - Bounds.m_YDomain.m_dMin)/10.;
    double dExtraZ = (Bounds.m_ZDomain.m_dMax - Bounds.m_ZDomain.m_dMin)/10.;

    double dStartX = Bounds.m_XDomain.m_dMin - dExtraX;
    double dStartY = Bounds.m_YDomain.m_dMin - dExtraY;
    double dStartZ = Bounds.m_ZDomain.m_dMin - dExtraZ;

    double dEndX = Bounds.m_XDomain.m_dMax + dExtraX;
    double dEndY = Bounds.m_YDomain.m_dMax + dExtraY;
    double dEndZ = Bounds.m_ZDomain.m_dMax + dExtraZ;

    // equal spacing in all three directions
    double dIncrement = std::min({std::abs(dEndX - dStartX),
                                  std::abs(dEndY - dStartY),
                                  std::abs(dEndZ - dStartZ)});
    dIncrement /= 100.;

    std::vector<SGM::Point3D> aPoints;
    std::vector<unsigned> aEmpty;

//    ALL DIRECTIONS
    double dX = dStartX;
    while (dX < dEndX)
        {
        double dY = dStartY;
        while (dY < dEndY)
            {
            double dZ = dStartZ;
            while (dZ < dEndZ)
                {
                aPoints.emplace_back(dX, dY, dZ);
                dZ += dIncrement;
                }
            dY += dIncrement;
            }
        dX += dIncrement;
        }

    //SGM::CreateComplex(rResult,aPoints,aEmpty,aEmpty);

    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
    SGM::Volume VolumeID = *(sVolumes.begin());

    std::vector<bool> aIsPointInside = SGM::PointsInVolume(rResult, aPoints, VolumeID);

    std::vector<SGM::Point3D> aPointsInside;
    for (size_t i = 0; i < aPoints.size(); ++i)
        {
        if (aIsPointInside[i])
            {
            aPointsInside.push_back(aPoints[i]);
            }
        }
    SGM::CreateComplex(rResult,aPointsInside,aEmpty,aEmpty);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(speed_check, DISABLED_single_points_in_volume_OUO_full_model_volume1)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    const char* ouo_file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(ouo_file_name);
    expect_import_ouo_success(ouo_file_name, rResult);

    SGM::Interval3D Bounds = SGM::GetBoundingBox(rResult,SGM::Thing());

    std::vector<SGM::Point3D> aPoints = {{0,0,0}};
    std::vector<unsigned> aEmpty;
    //SGM::CreateComplex(rResult,aPoints,aEmpty,aEmpty);

    std::set<SGM::Volume> sVolumes;
    SGM::FindVolumes(rResult,SGM::Thing(),sVolumes);
    SGM::Volume VolumeID = *(sVolumes.begin());

    std::vector<bool> aIsPointInside = SGM::PointsInVolume(rResult, aPoints, VolumeID);
    std::vector<SGM::Point3D> aPointsInside;
    for (size_t i = 0; i < aPoints.size(); ++i)
        {
        if (aIsPointInside[i])
            {
            aPointsInside.push_back(aPoints[i]);
            }
        }
    SGM::CreateComplex(rResult,aPointsInside,aEmpty,aEmpty);

    SGMTesting::ReleaseTestThing(pThing);
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

TEST(models_single_check, import_check_glom4_0009_Bcam) 
{
    const char* file_name = "Glom4/0009-_Bcam.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_curved_cube_cubit) 
{
    const char* file_name = "curved_cube_cubit.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, DISABLED_import_check_glom4_0010_Bsteer) 
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

TEST(models_single_check, DISABLED_import_check_glom4_0018_Bhinkey) 
{
    const char* file_name = "Glom4/0018-_Bhinkey.stp";
    SCOPED_TRACE(file_name);
    expect_import_check_success(file_name);
}

TEST(models_single_check, import_check_glom4_0019_Bhinkey_A)
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

TEST(models_single_check, import_check_brick10)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    const char* file_name = "brick10.stp";
    expect_import_success(file_name, rResult);
    SGMTesting::ReleaseTestThing(pThing);

}

TEST(intersection_check, DISABLED_check_plane_circle_consistent_with_cylinder_line_intersections)
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
    std::vector<SGM::Entity> aEntity;
    //Euclid::Point3D line_vect = {0.9946543569679107, 0.1032603997898127, -1.270825582042912e-14};
    SGM::UnitVector3D line_vect = {plane_normal[1],-plane_normal[0],plane_normal[2]}; // choose orthog to plane_normals
    {
        size_t surf_id = 7; //26;

        bool bUseWholeLine = true;
        SGM::RayFire(rResult, left_coords_face_node, line_vect, surf_id, aPointsFaceLine, aTypesFaceLine, aEntity, tolerance, bUseWholeLine);
    }

    const double dot_prod = plane_normal[0]*line_vect[0]+plane_normal[1]*line_vect[1]+plane_normal[2]*line_vect[2];

    EXPECT_NEAR(dot_prod, 0.0, tolerance);
    EXPECT_EQ(1U, aPointsEdgePlane.size());
    EXPECT_EQ(1U, aPointsFaceLine.size());
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

    EXPECT_EQ(3U, aPoints.size());

    SGMTesting::ReleaseTestThing(pThing); 
}

TEST(assembly_check, import_one_level_assembly)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    expect_import_success("2box.STEP", rResult);
    expect_check_success(rResult);

    std::set<SGM::Body> sBodies;
    bool bTopLevel = true;
    SGM::FindBodies(rResult, SGM::Thing(), sBodies, bTopLevel);
    EXPECT_EQ(sBodies.size(), 2U);

    SGM::Interval3D BodyBoxes[2];
    BodyBoxes[0] = SGM::Interval3D(0.0,2.5,0.0,1.7,0.0,0.5);
    BodyBoxes[1] = SGM::Interval3D(0.0,0.5,0.0,.25,.5,.9);
    
    auto iter = sBodies.begin();
    for (size_t iIndex=0; iIndex<sBodies.size(); ++iIndex)
    {
      SGM::Interval3D box = SGM::GetBoundingBox(rResult, *iter);
      EXPECT_TRUE(box == BodyBoxes[iIndex]);
      iter++;
    }

    SGMTesting::ReleaseTestThing(pThing); 
}

TEST(assembly_check, import_two_level_assembly)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing); 

    expect_import_success("box-smbox-2box.STEP", rResult);
    expect_check_success(rResult);

    std::set<SGM::Body> sBodies;
    bool bTopLevel = true;
    SGM::FindBodies(rResult, SGM::Thing(), sBodies, bTopLevel);
    EXPECT_EQ(sBodies.size(), 4U);

    SGM::Interval3D BodyBoxes[4];
    BodyBoxes[0] = SGM::Interval3D(0.0,0.5,0.0,0.25,1.5,1.9);
    BodyBoxes[1] = SGM::Interval3D(0.0,2.5,0.0,1.7,1.0,1.5);
    BodyBoxes[2] = SGM::Interval3D(1.0,1.4,1.45,1.7,0.5,1.0);
    BodyBoxes[3] = SGM::Interval3D(0.0,2.5,0.0,1.7,0.0,0.5);
    
    auto iter = sBodies.begin();
    for (size_t iIndex=0; iIndex<sBodies.size(); ++iIndex)
    {
      SGM::Interval3D box = SGM::GetBoundingBox(rResult, *iter);
      EXPECT_TRUE(box == BodyBoxes[iIndex]);
      iter++;
    }

    SGMTesting::ReleaseTestThing(pThing); 
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif
