#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <SGMTranslators.h>

#include "SGMComplex.h"
#include "SGMDisplay.h"

#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"
#include "SGMTransform.h"
#include "SGMAttribute.h"
#include "SGMInterval.h"
#include "SGMTopology.h"
#include "SGMChecker.h"

#include "test_utility.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(complex_check, stl_file_not_exist)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    std::string file_path = get_models_file_path("STL Files/fake_name_42.stl");
    options.m_bMerge=true;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeFileOpen);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(complex_check, complex_read_no_merge)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions options;

    std::string file_path = get_models_file_path("STL Files/Block_With_Face.stl");
    options.m_bMerge=false;
    SGM::ReadFile(rResult, file_path, entities, log, options);
    auto resultType = rResult.GetResult();
    EXPECT_EQ(resultType, SGM::ResultTypeOK);

    auto aComplexes = (std::vector<SGM::Complex> *) &entities;
    EXPECT_GE(aComplexes->size(), 1);

    // filenames are not allowed with '?' character on Windows
    // SGM::Complex ComplexID = aComplexes->front();
    // std::string OutputSTLFile = get_models_file_path("STL Files/Block?.stl");
    // SGM::SaveSTL(rResult, OutputSTLFile, ComplexID, options);
    // resultType = rResult.GetResult();
    // EXPECT_EQ(resultType, SGM::ResultTypeFileOpen);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(complex_check, order_points)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // run some tests for point sorting

    EXPECT_TRUE(SGM::RunInternalTest(rResult,7));

    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(complex_check, merge_complex_holes_data_other)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 =
        {
            {0.563698000000000, 0.789630000000000, 103.741945000000001},
            {0.563698000000000, 0.774716000000000, 103.781315000000006},
            {0.563698000000000, 0.760072000000000, 103.820684999999997},
            {0.603068000000000, 0.774628000000000, 103.781315000000006},
            {0.603068000000000, 0.789235000000000, 103.741945000000001},
            {0.603068000000000, 0.774628000000000, 103.781315000000006}
        };

    //    bool isLess34 = SGMInternal::LessZOrder(aPoints1[3], SGMInternal::Point3DSeparate(aPoints1[3]),
    //                                            aPoints1[4], SGMInternal::Point3DSeparate(aPoints1[4]));
    //    bool isLess43 = SGMInternal::LessZOrder(aPoints1[4], SGMInternal::Point3DSeparate(aPoints1[4]),
    //                                            aPoints1[3], SGMInternal::Point3DSeparate(aPoints1[3]));
    //    EXPECT_FALSE(isLess34);
    //    EXPECT_TRUE(isLess43);

    std::vector<unsigned> aTriangles1 = {0,1,2, 3,4,5};
    bool bMerge = true;

    auto ComplexID1 = SGM::CreateTriangles(rResult, aPoints1, aTriangles1, bMerge);

    auto aPointsComplex1 = SGM::GetComplexPoints(rResult, ComplexID1);
    EXPECT_EQ(aPointsComplex1.size(),5);
    //    std::cout << "aPointsComplex1" << std::endl;
    //    for (auto & Point : aPointsComplex1)
    //        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex1 = SGM::GetComplexTriangles(rResult, ComplexID1);
    EXPECT_EQ(aTrianglesComplex1.size(),6);
    //    std::cout << "aTrianglesComplex1" << std::endl;
    //    for (int i = 0; i < aTrianglesComplex1.size(); i += 3)
    //        std::cout << aTrianglesComplex1[i] << ' ' << aTrianglesComplex1[i+1] << ' ' << aTrianglesComplex1[i+2] << std::endl;

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(complex_check, DISABLED_merge_complex)
    {

// Two complexes with two triangles each, the complexes touch at one pair of edges.
//
// Unused point denoted by '+'
//
//                  Complex2                      Complex1
//
// (-1.5, 0.5, 1.0)             (-0.5, 0.5,-1.0)                (0.5, 0.5, 2.0)
//
//                  2 - - - - - 4              4 - - - - - 3
//                5   .         |              |         .   0
//      10        | .   .       |              |       .   . |
//     +          |   .   .     |              |     .   .   |
//      0   6     |     .   .   |              |   .   .     |
//     +   +      |  7    .   . |              | .   .       |
//      1         | +        .  3              5   .         |
//     +          8 - - - - - 9                  1 - - - - - 2
//
// (-1.5,-0.5, 1.0)            (-0.5,-0.5,-1.0)                 (0.5,-0.5, 2.0)
//
// First we merge triangles of each of the complexes, then we merge the two complexes.
//
// After merging each complex:
//
//               3 - - - - - 2                 1 - - - - - 3
//               | .         |                 |         . |
//               |   .       |                 |       .   |
//               |     .     |                 |     .     |
//               |       .   |                 |   .       |
//               |         . |                 | .         |
//               1 - - - - - 0                 0 - - - - - 2
//
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 =
        {
            { 0.5,              0.5,              2.0},
            {-0.5,             -0.5,             -1.0},
            { 0.5,             -0.5,              2.0},
            { 0.5000000000001,  0.5000000000001,  2.0000000000001},
            {-0.5000000000001,  0.5000000000001, -1.0000000000001},
            {-0.5000000000001, -0.5000000000001, -1.0000000000001}
        };

    std::vector<SGM::Point3D> aPoints2 =
        {
            { -2.5            ,  0.0            ,  1.0            }, // 0
            { -2.5000000000001, -0.0000000000001,  1.0000000000001},
            { -1.5            ,  0.5            ,  1.0            }, // 2
            { -0.5            , -0.5            , -1.0            },
            { -0.5            ,  0.5            , -1.0            }, // 4
            { -1.5            ,  0.5            ,  1.0            },
            { -1.51            , 0.0            ,  1.0            }, // 6
            { -1.5000000000001, -0.5000000000001,  1.0000000000001},
            { -1.5            , -0.5            ,  1.0            }, // 8
            { -0.5000000000001, -0.5000000000001, -1.0000000000001},
            { -2.e100         ,  5.e100         ,  5.e100         }  //10
        };

    std::vector<unsigned> aTriangles1 = {0,1,2, 3,4,5};
    std::vector<unsigned> aTriangles2 = {2,3,4, 5,8,9};

    // merge the points and triangles making the two complexes
    bool bMerge = true;
    auto ComplexID1 = SGM::CreateTriangles(rResult, aPoints1, aTriangles1, bMerge);

    auto aPointsComplex1 = SGM::GetComplexPoints(rResult, ComplexID1);
    EXPECT_EQ(aPointsComplex1.size(),4);
    std::cout << "aPointsComplex1" << std::endl;
    for (auto & Point : aPointsComplex1)
        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex1 = SGM::GetComplexTriangles(rResult, ComplexID1);
    EXPECT_EQ(aTrianglesComplex1.size(),6);
    std::cout << "aTrianglesComplex1" << std::endl;
    for (int i = 0; i < aTrianglesComplex1.size(); i += 3)
        std::cout << aTrianglesComplex1[i] << ' ' << aTrianglesComplex1[i+1] << ' ' << aTrianglesComplex1[i+2] << std::endl;

    EXPECT_EQ(aTrianglesComplex1, std::vector<unsigned>({3,0,2,3,1,0}));

    auto ComplexID2 = SGM::CreateTriangles(rResult, aPoints2, aTriangles2, bMerge);

    auto aPointsComplex2 = SGM::GetComplexPoints(rResult, ComplexID2);
    EXPECT_EQ(aPointsComplex2.size(),4);
    std::cout << "aPointsComplex2" << std::endl;
    for (auto & Point : aPointsComplex2)
        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex2 = SGM::GetComplexTriangles(rResult, ComplexID2);
    EXPECT_EQ(aTrianglesComplex2.size(),6);
    std::cout << "aTrianglesComplex2" << std::endl;
    for (int i = 0; i < aTrianglesComplex2.size(); i += 3)
        std::cout << aTrianglesComplex2[i] << ' ' << aTrianglesComplex2[i+1] << ' ' << aTrianglesComplex2[i+2] << std::endl;

    EXPECT_EQ(aTrianglesComplex2, std::vector<unsigned>({3,0,2,3,1,0}));

    // merge Complex1 and Complex2 into Complex3

    std::vector<SGM::Complex> aComplexIDs = {ComplexID1, ComplexID2};
    auto ComplexID3 = SGM::MergeComplexes(rResult, aComplexIDs);

    auto aPointsComplex3 = SGM::GetComplexPoints(rResult, ComplexID3);
    EXPECT_EQ(aPointsComplex3.size(),6);
    std::cout << "aPointsComplex3" << std::endl;
    for (auto & Point : aPointsComplex3)
        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex3 = SGM::GetComplexTriangles(rResult, ComplexID3);
    EXPECT_EQ(aTrianglesComplex2.size(),12);
    std::cout << "aTrianglesComplex3" << std::endl;
    for (int i = 0; i < aTrianglesComplex3.size(); i += 3)
        std::cout << aTrianglesComplex3[i] << ' ' << aTrianglesComplex3[i+1] << ' ' << aTrianglesComplex3[i+2] << std::endl;


    SGMTesting::ReleaseTestThing(pThing);
    }

//TEST(complex_check, remap)
//    {
//    // Test a triangles array where the points coincide and get merged.
//    //
//    //  0 - - - - - 2    6 - - - - - 8
//    //  |         .   3  |         .   11
//    //  |       .   . |  |       .   . |
//    //  |     .   .   |  |     .   .   |
//    //  |   .   .     |  |   .   .     |
//    //  | .   .       |  | .   .       |
//    //  1   .         |  7   .         |
//    //    4 - - - - - 5    9 - - - - -10
//    //
//    //     0 - - - - - 2 - - - - - 4
//    //     |         . |         . |
//    //     |       .   |       .   |
//    //     |     .     |     .     |
//    //     |   .       |   .       |
//    //     | .         | .         |
//    //     1 - - - - - 3 - - - - - 5
//    //
//    //
//    std::vector<unsigned> aTriangles = {1, 2, 0, 5, 3, 4, 8, 6, 7,10,11, 9};
//
//    const std::vector<unsigned> aOldToNewPoints  = {0, 1, 2, 2, 1, 3, 2, 3, 4, 3, 5, 4};
//
//    size_t nSize = aTriangles.size();
//    std::vector<unsigned> aNewTriangles(nSize);
//
//    // naive way, we use this as a check
//    for (unsigned i = 0; i < nSize; ++i)
//        {
//        aNewTriangles[i] = aOldToNewPoints[aTriangles[i]];
//        }
//
//    // call our fast version
//    SGMInternal::Remap(aOldToNewPoints.begin(), aTriangles.begin(), aTriangles.end());
//
//    for (unsigned i = 0; i < nSize; ++i)
//        {
//        EXPECT_EQ(aTriangles[i], aNewTriangles[i]);
//        }
//    }

#ifdef __clang__
#pragma clang diagnostic pop
#endif