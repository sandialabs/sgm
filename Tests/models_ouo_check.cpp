#include <gtest/gtest.h>
#include "test_utility.h"

#include "SGMInterrogate.h"

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
    SGM::TranslatorOptions const options;

    std::string file_path = get_models_ouo_file_path(file_name);
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);
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


// Import a file from OUO directory and check all entities
void expect_import_ouo_check_success(std::string const &ouo_file_name)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    expect_import_ouo_success(ouo_file_name, rResult);
    expect_check_success(rResult);
    SGMTesting::ReleaseTestThing(pThing);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_TSLhousingGeom) 
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_TSLhousingGeom.stp");
}

TEST(models_ouo_check, import_check_OUO_full_model_volume1)
{
    const char* file_name = "OUO_full_model_volume1.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_grv_geom)
{
    const char* file_name = "OUO_grv_geom.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
    std::vector<SGM::Face> faces;
    //SGM::Point3D sgm_point(0.00623037301312708,-1.04973316890717e-18,0.0166638951384168);
    SGM::Point3D sgm_point(0,0,0);
    SGM::FindCloseFaces(rResult, sgm_point, SGM::Thing(), 1e-5, faces);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_ZGeom) // TODO: Lots of faces.
{
    const char* file_name = "OUO_ZGeom.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0001_Bbat) 
{
    const char* file_name = "OUO_glom4/0001-_Bbat.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0002_Bknob) 
{
    const char* file_name = "OUO_glom4/0002-_Bknob.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0003_Bbracket)
{
    const char* file_name = "OUO_glom4/0003-_Bbracket.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0004_BSump1)
{
    const char* file_name = "OUO_glom4/0004-_BSump1.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0005_BSump2)
{
    const char* file_name = "OUO_glom4/0005-_BSump2.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0006_Bangle)
{
    const char* file_name = "OUO_glom4/0006-_Bangle.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_Cone_definition)
{
    const char* file_name = "OUO_glom4/OUO_Cone_definition.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0007_Bflange) // TODO: Faces 80 and 85.
{
    const char* file_name = "OUO_glom4/0007-_Bflange.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0008_Bkey) // TODO: Faces 5, 10, and 7.
{
    const char* file_name = "OUO_glom4/0008-_Bkey.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0009_Bcam) // TODO: Faces 78, 94, 100 surfaces 259, 265
{
    const char* file_name = "OUO_glom4/0009-_Bcam.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0010_Bsteer) // TODO: Face 388 infinite call
{
    const char* file_name = "OUO_glom4/0010-_Bsteer.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0011_Bdice) 
{
    const char* file_name = "OUO_glom4/0011-_Bdice.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0012_Bgear)
{
    const char* file_name = "OUO_glom4/0012-_Bgear.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0013_Bbellows)
{
    const char* file_name = "OUO_glom4/0013-_Bbellows.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0014_Bplate) // TODO: Lots of faces.
{
    const char* file_name = "OUO_glom4/0014-_Bplate.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0015_Bpipe)
{
    const char* file_name = "OUO_glom4/0015-_Bpipe.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0016_Bspring) // TODO: Faces 2 and 3 minor flipped facet.
{
    const char* file_name = "OUO_glom4/0016-_Bspring.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, import_check_OUO_glom4_0017_Bcube) 
{
    const char* file_name = "OUO_glom4/0017-_Bcube.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0018_Bhinkey) // TODO: Lots of faces off.
{
    const char* file_name = "OUO_glom4/0018-_Bhinkey.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}

TEST(models_ouo_check, DISABLED_import_check_OUO_glom4_0019_Bhinkey_A)
{
    const char* file_name = "OUO_glom4/0019-_Bhinkey_A.stp";
    SCOPED_TRACE(file_name);
    expect_import_ouo_check_success(file_name);
}
