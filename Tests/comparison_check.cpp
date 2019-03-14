#include <limits>
#include <gtest/gtest.h>

#include "SGMEntityFunctions.h"
#include "SGMPrimitives.h"
#include "SGMTopology.h"
#include "SGMGeometry.h"
#include "SGMInterrogate.h"

#include "test_utility.h"

void SetupFaces(SGM::Result &rResult,
	SGM::Face &Face1,
	SGM::Face &Translated,
	SGM::Face &Different,
    SGM::Face &Scaled,
    SGM::Face &Mirrored)
{
	SGM::Point3D Origin1(0, 0, 1);
	SGM::Point3D Origin2(0, 0, -1);
	SGM::Point3D Origin3(0, 0, 2);
    SGM::Point3D Origin4(1, 1, -3);
    SGM::Point3D Origin5(0, 0, 1.5);
	SGM::UnitVector3D Normal(0, 0, 1);
	std::vector<SGM::EdgeSideType> aTypes = { SGM::FaceOnLeftType, SGM::FaceOnLeftType, SGM::FaceOnLeftType, SGM::FaceOnLeftType };

	std::vector<SGM::Point3D> aPoints1 = { {1, 1, 1},  {2, 1, 1},  {2.1, 1.1, 1},  {1.1, 1.2, 1} };
	std::vector<SGM::Edge> aEdges1(aPoints1.size());
	for (size_t iPoint = 0; iPoint < aPoints1.size(); ++iPoint)
		aEdges1[iPoint] = SGM::CreateLinearEdge(rResult, aPoints1[iPoint], aPoints1[(iPoint + 1) % 4]);
	SGM::Surface Plane1 = SGM::CreatePlane(rResult, Origin1, Normal);
	SGM::Body Body1 = SGM::CreateSheetBody(rResult, Plane1, aEdges1, aTypes);

	std::vector<SGM::Point3D> aPoints2 = { {1, 1, -1}, {2, 1, -1}, {2.1, 1.1, -1}, {1.1, 1.2, -1} };
	std::vector<SGM::Edge> aEdges2(aPoints2.size());
	for (size_t iPoint = 0; iPoint < aPoints2.size(); ++iPoint)
		aEdges2[iPoint] = SGM::CreateLinearEdge(rResult, aPoints2[iPoint], aPoints2[(iPoint + 1) % 4]);
	SGM::Surface Plane2 = SGM::CreatePlane(rResult, Origin2, Normal);
	SGM::Body Body2 = SGM::CreateSheetBody(rResult, Plane2, aEdges2, aTypes);

	std::vector<SGM::Point3D> aPoints3 = { {1, 1, 2},  {2, 1, 2},  {2, 1.1, 2},    {1, 1.1, 2} };
	std::vector<SGM::Edge> aEdges3(aPoints3.size());
	for (size_t iPoint = 0; iPoint < aPoints3.size(); ++iPoint)
		aEdges3[iPoint] = SGM::CreateLinearEdge(rResult, aPoints3[iPoint], aPoints3[(iPoint + 1) % 4]);
	SGM::Surface Plane3 = SGM::CreatePlane(rResult, Origin3, Normal);
	SGM::Body Body3 = SGM::CreateSheetBody(rResult, Plane3, aEdges3, aTypes);

    std::vector<SGM::Point3D> aPoints4 = { {3, 3, -3},  {6, 3, -3},  {6.3, 3.3, -3},    {3.3, 3.6, -3} };
    std::vector<SGM::Edge> aEdges4(aPoints4.size());
    for (size_t iPoint = 0; iPoint < aPoints4.size(); ++iPoint)
        aEdges4[iPoint] = SGM::CreateLinearEdge(rResult, aPoints4[iPoint], aPoints4[(iPoint + 1) % 4]);
    SGM::Surface Plane4 = SGM::CreatePlane(rResult, Origin4, Normal);
    SGM::Body Body4 = SGM::CreateSheetBody(rResult, Plane4, aEdges4, aTypes);

    std::vector<SGM::Point3D> aPoints5 = { {1, 1, 1},  {0, 1, 1},  {-0.1, 1.1, 1},  {0.9, 1.2, 1} };
    std::vector<SGM::Edge> aEdges5(aPoints5.size());
    for (size_t iPoint = 0; iPoint < aPoints5.size(); ++iPoint)
        aEdges5[iPoint] = SGM::CreateLinearEdge(rResult, aPoints5[iPoint], aPoints5[(iPoint + 1) % 4]);
    SGM::Surface Plane5 = SGM::CreatePlane(rResult, Origin5, Normal);
    aTypes = { SGM::FaceOnRightType, SGM::FaceOnRightType, SGM::FaceOnRightType, SGM::FaceOnRightType };
    SGM::Body Body5 = SGM::CreateSheetBody(rResult, Plane5, aEdges5, aTypes);

    std::set<SGM::Face> sFaces;
	SGM::FindFaces(rResult, Body1, sFaces);
	Face1 = (*(sFaces.begin()));

	sFaces.clear();
	SGM::FindFaces(rResult, Body2, sFaces);
	Translated = (*(sFaces.begin()));

	sFaces.clear();
	SGM::FindFaces(rResult, Body3, sFaces);
	Different = (*(sFaces.begin()));

    sFaces.clear();
    SGM::FindFaces(rResult, Body4, sFaces);
    Scaled = (*(sFaces.begin()));

    sFaces.clear();
    SGM::FindFaces(rResult, Body5, sFaces);
    Mirrored = (*(sFaces.begin()));

}

//void SetupTwoBlocks(SGM::Result &rResult,
//                    SGM::Body &Block1,
//                    SGM::Face &Face1,
//                    SGM::Body &Block2,
//                    SGM::Face &Face2)
//{
//    SGM::Point3D Point1(1, 1, 1);
//    SGM::Point3D Point2(2, 1.1, 1.2);
//    SGM::Point3D Point3(3, 1.2, 1.4);
//    Block1 = SGM::CreateBlock(rResult, Point1, Point2);
//    Block2 = SGM::CreateBlock(rResult, Point2, Point3);
//
//    std::set<SGM::Face> sFaces;
//    SGM::FindFaces(rResult, Block1, sFaces);
//    Face1= (*(sFaces.begin()));
//
//    sFaces.clear();
//    SGM::FindFaces(rResult, Block2, sFaces);
//    Face2= (*(sFaces.begin()));
//}

TEST(comparison_check, face_comparison)
{
	SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
	SGM::Result rResult(pThing);

    SGM::Face Face1, Translated, Different, Scaled, Mirrored;
    SetupFaces(rResult, Face1, Translated, Different, Scaled, Mirrored);

    bool bIgnoreScale = false;
	std::vector<SGM::Face> aSimilar;
	SGM::FindSimilarFaces(rResult, Face1, aSimilar, bIgnoreScale);
    EXPECT_EQ(aSimilar.size(), 2);

    aSimilar.clear();
    bIgnoreScale = true;
    SGM::FindSimilarFaces(rResult, Face1, aSimilar, bIgnoreScale);
    EXPECT_EQ(aSimilar.size(), 3);

	SGMTesting::ReleaseTestThing(pThing);
}



