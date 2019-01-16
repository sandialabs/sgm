#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "FacetToBRep.h"
#include "SGMComplex.h"
#include "SGMDisplay.h"
#include "OrderPoints.h"

#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"
#include "SGMTransform.h"
#include "SGMAttribute.h"
#include "SGMInterval.h"
#include "SGMTopology.h"

#include "test_utility.h"

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "cert-err58-cpp"

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

TEST(complex_check, point3d_order)
    {
    std::vector<SGM::Point3D> aPoints0 =
        {
            {0.0000000e+00, 4.5602101e-01, 2.5127900e-01},
            {0.0000000e+00, 1.6629200e-01, 5.0000000e-01},  // point[1] identical with point[4]
            {0.0000000e+00, 2.1484300e-01, 5.2078199e-01},
            {5.2347499e-01, 2.0895001e-02, 7.1533000e-01},
            {0.0000000e+00, 1.6629200e-01, 5.0000000e-01},
            {6.2853903e-01, 1.4633000e-02, 6.1653697e-01}
        };
    buffer<unsigned> aIndexOrdered0 = SGMInternal::OrderPointsLexicographical(aPoints0);
    std::cout << std::endl;
    for (int i = 0; i < aIndexOrdered0.size(); ++i)
        std::cout << "aIndexOrdered0[" << i << "] = " << aIndexOrdered0[i] << std::endl;

    std::vector<SGM::Point3D> aPoints1 =
        {
            { 0.5,              0.5,              2.0},
            {-0.5,             -0.5,             -1.0},
            { 0.5,             -0.5,              2.0},
            { 0.5000000000001,  0.5000000000001,  2.0000000000001},
            {-0.5000000000001,  0.5000000000001, -1.0000000000001},
            {-0.5000000000001, -0.5000000000001, -1.0000000000001}
        };

    buffer<unsigned> aIndexOrdered1 = SGMInternal::OrderPointsZorder(aPoints1);
    std::cout << std::endl;
    for (int i = 0; i < aIndexOrdered1.size(); ++i)
        std::cout << "aIndexOrdered1[" << i << "] = " << aIndexOrdered1[i] << std::endl;

    EXPECT_EQ(aIndexOrdered1[0],5);
    EXPECT_EQ(aIndexOrdered1[1],1);
    EXPECT_EQ(aIndexOrdered1[2],4);
    EXPECT_EQ(aIndexOrdered1[3],2);
    EXPECT_EQ(aIndexOrdered1[4],0);
    EXPECT_EQ(aIndexOrdered1[5],3);

    std::vector<SGM::Point3D> aPoints2 =
        {
            {-3000.0e-16,         -3000.0e-16,           5000.0e-16},
            {-3000.000000001e-16,  5000.000000001e-16,  -3000.000000001e-16},
            {-3000.000000001e-16, -3000.000000001e-16,   5000.000000001e-16},
            {-3000.0e-16,          5000.0e-16,          -3000.0e-16}
        };

    buffer<unsigned> aIndexOrdered2 = SGMInternal::OrderPointsLexicographical(aPoints2);
    std::cout << std::endl;
    for (int i = 0; i < aIndexOrdered2.size(); ++i)
        std::cout << "aIndexOrdered2[" << i << "] = " << aIndexOrdered2[i] << std::endl;


    std::vector<SGM::Point3D> aPoints3 =
        {
            {     0.5,                     0.5,                    0.5},                    // 0
            {    -0.5,                    -0.5,                   -0.5},
            {    -0.5,                     0.5,                    0.5},                    // 2
            {     0.5,                    -0.5,                    0.5},
            {     0.5,                     0.5,                   -0.5},                    // 4
            {    -0.5,                    -0.5,                    0.5},
            {     0.5,                    -0.5,                   -0.5},                    // 6
            {    -0.5,                     0.5,                   -0.5},
            {   -10.0,                    20.0,                   20.0},                    // 8
            {    20.0,                   -10.0,                   20.0},
            {    20.0,                    20.0,                  -10.0},                    // 10
            { -3000.0,                 -3000.0,                  5000.0},
            {  5000.0,                 -3000.0,                 -3000.0},                   // 12
            { -3000.0,                  5000.0,                 -3000.0},
            {     0.5e-16,                 0.5e-16,                 0.5e-16},               // 14
            {    -0.5e-16,                -0.5e-16,                -0.5e-16},
            {    -0.5e-16,                 0.5e-16,                 0.5e-16},               // 16
            {     0.5e-16,                -0.5e-16,                 0.5e-16},
            {     0.5e-16,                 0.5e-16,                -0.5e-16},               // 18
            {    -0.5e-16,                -0.5e-16,                 0.5e-16},
            {     0.5e-16,                -0.5e-16,                -0.5e-16},               // 20
            {    -0.5e-16,                 0.5e-16,                -0.5e-16},
            {   -10.0e-16,                20.0e-16,                20.0e-16},               // 22
            {    20.0e-16,               -10.0e-16,                20.0e-16},
            {    20.0e-16,                20.0e-16,               -10.0e-16},               // 24
            { -3000.0e-16,             -3000.0e-16,              5000.0e-16},
            {  5000.0e-16,             -3000.0e-16,             -3000.0e-16},               // 26
            { -3000.0e-16,              5000.0e-16,             -3000.0e-16},
            {     0.5000000000001,         0.5000000000001,         0.5000000000001},       // 28
            {    -0.5000000000001,        -0.5000000000001,        -0.5000000000001},
            {    -0.5000000000001,         0.5000000000001,         0.5000000000001},       // 30
            {     0.5000000000001,        -0.5000000000001,         0.5000000000001},
            {     0.5000000000001,         0.5000000000001,        -0.5000000000001},       // 32
            {    -0.5000000000001,        -0.5000000000001,         0.5000000000001},
            {     0.5000000000001,        -0.5000000000001,        -0.5000000000001},       // 34
            {    -0.5000000000001,         0.5000000000001,        -0.5000000000001},
            {   -10.00000000001,          20.00000000001,          20.00000000001},         // 36
            {    20.00000000001,         -10.00000000001,          20.00000000001},
            {    20.00000000001,          20.00000000001,         -10.00000000001},         // 38
            { -3000.000000001,         -3000.000000001,          5000.000000001},
            {  5000.000000001,         -3000.000000001,         -3000.000000001},           // 40
            { -3000.000000001,          5000.000000001,         -3000.000000001},
            {     0.5000000000001e-16,     0.5000000000001e-16,     0.5000000000001e-16},   // 42
            {    -0.5000000000001e-16,    -0.5000000000001e-16,    -0.5000000000001e-16},
            {    -0.5000000000001e-16,     0.5000000000001e-16,     0.5000000000001e-16},   // 44
            {     0.5000000000001e-16,    -0.5000000000001e-16,     0.5000000000001e-16},
            {     0.5000000000001e-16,     0.5000000000001e-16,    -0.5000000000001e-16},   // 46
            {    -0.5000000000001e-16,    -0.5000000000001e-16,     0.5000000000001e-16},
            {     0.5000000000001e-16,    -0.5000000000001e-16,    -0.5000000000001e-16},   // 48
            {    -0.5000000000001e-16,     0.5000000000001e-16,    -0.5000000000001e-16},
            {   -10.00000000001e-16,      20.00000000001e-16,      20.00000000001e-16},     // 50
            {    20.00000000001e-16,     -10.00000000001e-16,      20.00000000001e-16},
            {    20.00000000001e-16,      20.00000000001e-16,     -10.00000000001e-16},     // 52
            { -3000.000000001e-16,     -3000.000000001e-16,      5000.000000001e-16},
            {  5000.000000001e-16,     -3000.000000001e-16,     -3000.000000001e-16},       // 54
            { -3000.000000001e-16,      5000.000000001e-16,     -3000.000000001e-16},
        };

    buffer<unsigned> aIndexOrdered3 = SGMInternal::OrderPointsZorder(aPoints3);
    std::cout << std::endl;
    for (int i = 0; i < aIndexOrdered3.size(); ++i)
        std::cout << "aIndexOrdered3[" << i << "] = " << aIndexOrdered3[i] << std::endl;

    // every consecutive pair in the sorted list should be close
    for (int i = 0; i+1 < aIndexOrdered3.size(); i+=2)
        {
        EXPECT_TRUE(SGMInternal::AlmostEqual(aPoints3[aIndexOrdered3[i]],
                                             aPoints3[aIndexOrdered3[i + 1]],
                                             SGMInternal::numeric_tolerance<double>::relative));
        }

    }

void print_ordered_points(std::string name,
                          buffer<unsigned> const &aIndexOrdered,
                          std::vector<SGM::Point3D> const & aPoints)
    {
    std::cout.setf(std::ios::floatfield,std::ios::scientific);
    std::cout << std::setprecision(15);
    std::cout << name << std::endl;
    for (size_t i = 0; i < aPoints.size(); ++i)
        {
        SGM::Point3D const &Point = aPoints[aIndexOrdered[i]];
        std::cout <<
            aIndexOrdered[i] <<
            std::setw(23) << Point[0] <<
            std::setw(23) << Point[1] <<
            std::setw(23) << Point[2] << std::endl;
        }
    std::cout << std::endl;
    }
    
TEST(complex_check, merge_complex_holes_data)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aPoints1 =
        {
            {5.2093670000000, 0.2457350000000, 103.6632050000000},
            {5.2093670000003, 0.2854750000003, 103.6632050000003},
            {5.2093670000000, 0.2457350000000, 103.6632050000000},
            {5.2093670000001, 0.2854750000001, 103.6632050000001},
            {5.2093670000000, 0.2457350000000, 103.6632050000000},
            {5.2093670000002, 0.2854750000002, 103.6632050000002},
            {5.2093670000001, 0.2457350000001, 103.6632050000001},
            {5.2487370000001, 0.2384040000001, 103.6632050000001},
            {5.2487370000001, 0.2775250000001, 103.6632050000001},
            {5.2093670000000, 0.2854750000000, 103.6632050000000}
        };
    {
        buffer<unsigned> aIndexOrdered1 = SGMInternal::OrderPointsLexicographical(aPoints1);
        print_ordered_points("aPoints1 Lexicographical:", aIndexOrdered1, aPoints1);
        buffer<unsigned> aExpectedLexicographical1 = { 0, 2, 4, 9, 6, 3, 5, 1, 7, 8 };
        EXPECT_EQ(aIndexOrdered1, aExpectedLexicographical1);
    }

    {
        buffer<unsigned> aIndexOrdered2 = SGMInternal::OrderPointsZorder(aPoints1);
        print_ordered_points("aPoints1 Z order:", aIndexOrdered2, aPoints1);
        buffer<unsigned> aExpectedZorder2 = { 0, 2, 4, 6, 7, 9, 3, 5, 1, 8 };
        EXPECT_EQ(aIndexOrdered2, aExpectedZorder2);
    }

    /* Lexicographical sorting does not do a great job of merging triangles

    std::vector<unsigned> aTriangles1 = {0,1,2, 3,4,5, 6,7,8};
    bool bMerge = true;
    auto ComplexID1 = SGM::CreateTriangles(rResult, aPoints1, aTriangles1, bMerge);

    auto aPointsComplex1 = SGM::GetComplexPoints(rResult, ComplexID1);
    EXPECT_EQ(aPointsComplex1.size(),5);
    std::cout << "aPointsComplex1" << std::endl;
    for (auto & Point : aPointsComplex1)
        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex1 = SGM::GetComplexTriangles(rResult, ComplexID1);
    EXPECT_EQ(aTrianglesComplex1.size(),6);
    std::cout << "aTrianglesComplex1" << std::endl;
    for (int i = 0; i < aTrianglesComplex1.size(); i += 3)
        std::cout << aTrianglesComplex1[i] << ' ' << aTrianglesComplex1[i+1] << ' ' << aTrianglesComplex1[i+2] << std::endl;
    */

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
    std::cout << "aPointsComplex1" << std::endl;
    for (auto & Point : aPointsComplex1)
        std::cout << Point[0] << ' ' << Point[1] << ' ' << Point[2] << std::endl;

    auto aTrianglesComplex1 = SGM::GetComplexTriangles(rResult, ComplexID1);
    EXPECT_EQ(aTrianglesComplex1.size(),6);
    std::cout << "aTrianglesComplex1" << std::endl;
    for (int i = 0; i < aTrianglesComplex1.size(); i += 3)
        std::cout << aTrianglesComplex1[i] << ' ' << aTrianglesComplex1[i+1] << ' ' << aTrianglesComplex1[i+2] << std::endl;

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



