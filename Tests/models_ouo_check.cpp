#include <gtest/gtest.h>
#include "test_utility.h"

///////////////////////////////////////////////////////////////////////////////
//
// Tests that import and read selected files from our OUO data directory
//
///////////////////////////////////////////////////////////////////////////////

// import and OUO model and EXPECT ResultTypeOK
void expect_import_ouo_success(std::string const &file_name, SGM::Result &result)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;

    std::string file_path = get_models_ouo_file_path(file_name);
    SGM::ReadFile(result, file_path, entities, log, options);
    auto resultType = result.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);
}


// CheckEntity on a model (result), that has already been imported (from file_path);
// write any non-empty log file messages to failure message
void expect_check_success(SGM::Result &result)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(result,SGM::Thing(),Options,aLog);
    if (!aLog.empty())
        {
        std::string message;
        for (std::string &log_item: aLog)
            message += log_item;
        FAIL() << message;
        }
}


// import a file from OUO directory and check all entities
void expect_import_ouo_check_success(std::string const &ouo_file_name)
{
    auto thing = SGM::CreateThing();
    SGM::Result result(thing);
    expect_import_ouo_success(ouo_file_name, result);
    expect_check_success(result);
    SGM::DeleteThing(thing);
}


//TEST(ModelsOUOCheck, import_check_OUO_TSLhousingGeom)
//{
//    std::cout << std::endl << std::flush;
//    expect_import_ouo_check_success("OUO_TSLhousingGeom.stp");
//}

//TEST(ModelsOUOCheck, import_check_OUO_full_model_volume1)
//{
//    std::cout << std::endl << std::flush;
//    expect_import_ouo_check_success("OUO_full_model_volume1.stp");
//}

//TEST(ModelsOUOCheck, import_check_OUO_ZGeom)
//{
//    std::cout << std::endl << std::flush;
//    expect_import_ouo_check_success("OUO_ZGeom.stp");
//}

/*
TEST(ModelsOUOCheck, import_check_OUO_glom4_0001_Bbat)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0001-_Bbat.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0002_Bknob)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0002-_Bknob.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0003_Bbracket)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0003-_Bbracket.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0004_BSump1)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0004-_BSump1.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0005_BSump2)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0005-_BSump2.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0006_Bangle)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0006-_Bangle.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0007_Bflange)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0007-_Bflange.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0008_Bkey)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0008-_Bkey.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0009_Bcam)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0009-_Bcam.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0010_Bsteer)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0010-_Bsteer.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0011_Bdice)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0011-_Bdice.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0012_Bgear)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0012-_Bgear.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0013_Bbellows)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0013-_Bbellows.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0014_Bplate)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0014-_Bplate.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0015_Bpipe)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0015-_Bpipe.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0016_Bspring)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0016-_Bspring.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0017_Bcube)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0017-_Bcube.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0018_Bhinkey)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0018-_Bhinkey.stp");
}

TEST(ModelsOUOCheck, import_check_OUO_glom4_0019_Bhinkey_A)
{
    std::cout << std::endl << std::flush;
    expect_import_ouo_check_success("OUO_glom4/0019-_Bhinkey_A.stp");
}
 */