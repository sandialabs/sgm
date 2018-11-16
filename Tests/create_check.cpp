#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <FacetToBRep.h>

#include "SGMGeometry.h"
#include "SGMEntityFunctions.h"
#include "SGMTransform.h"

#include "test_utility.h"

TEST(create_check, create_parabola)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // y=ax^2 parabola
    // a=2

    aPoints.emplace_back(0.0, 0.0, 0.0);
    aPoints.emplace_back(1.0, 2.0, 0.0);
    aPoints.emplace_back(-1.0, 2.0, 0.0);
    aPoints.emplace_back(2.0, 8.0, 0.0);
    aPoints.emplace_back(-3.0, 18.0, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));

    SGM::Curve CopyID=SGM::Curve(SGM::CopyEntity(rResult,CurveID).m_ID);
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,CopyID);
    EXPECT_TRUE(SGM::TestCurve(rResult,CopyID,0.1));
    SGM::DeleteEntity(rResult,CopyID);
    
    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(create_check, create_hyperbola)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // x^2/a^2-y^2/b^2=1 hyperbola
    // a=2 b=3

    aPoints.clear();

    aPoints.emplace_back(2.0, 0.0, 0.0);
    aPoints.emplace_back(3.0, 3.3541019662496845446137605030969, 0.0);
    aPoints.emplace_back(6.0, 8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(6.0, -8.4852813742385702928101323452582, 0.0);
    aPoints.emplace_back(3.0, -3.3541019662496845446137605030969, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));
    
    SGM::Curve CopyID=SGM::Curve(SGM::CopyEntity(rResult,CurveID).m_ID);
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,CopyID);
    EXPECT_TRUE(SGM::TestCurve(rResult,CopyID,0.1));
    SGM::DeleteEntity(rResult,CopyID);

    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }


TEST(create_check, create_ellipse)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(5);

    // x^2/a^2+y^2/b^2=1 ellipse
    // a=2 b=3

    aPoints.clear();

    aPoints.emplace_back(0.0, 3.0, 0.0);
    aPoints.emplace_back(2.0, 0.0, 0.0);
    aPoints.emplace_back(-2.0, 0.0, 0.0);
    aPoints.emplace_back(0.0, -3.0, 0.0);
    aPoints.emplace_back(1.0, 2.5980762113533159402911695122588, 0.0);

    SGM::Curve CurveID=SGM::FindConic(rResult,aPoints,dTolerance);
    EXPECT_TRUE(SGM::TestCurve(rResult,CurveID,0.1));
    
    SGM::Curve CopyID=SGM::Curve(SGM::CopyEntity(rResult,CurveID).m_ID);
    SGM::Transform3D Trans(SGM::Vector3D(1,1,1));
    SGM::TransformEntity(rResult,Trans,CopyID);
    EXPECT_TRUE(SGM::TestCurve(rResult,CopyID,0.1));
    SGM::DeleteEntity(rResult,CopyID);

    SGM::DeleteEntity(rResult,CurveID);
    SGMTesting::ReleaseTestThing(pThing);
    }



TEST(create_check, create_torus_knot)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);
        
    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(0,1,0),YAxis(1,0,0);
    size_t nA=2,nB=3;
    double dR=5.0,dr=2;

    SGM::Curve IDCurve = SGM::CreateTorusKnot(rResult, Center, XAxis, YAxis, dr, dR, nA, nB);
    EXPECT_TRUE(SGM::TestCurve(rResult,IDCurve,1.0));
    SGM::Edge Edge1 = SGM::CreateEdge(rResult, IDCurve);

    SGM::Curve IDCurve2 = SGM::Curve(SGM::CopyEntity(rResult, IDCurve).m_ID);
    SGM::Transform3D Trans;
    SGM::Point3D Origin(0, 0, 0);
    SGM::UnitVector3D Axis(0, 0, 1);
    double dAngle = 0.52359877559829887307710723054658; // 30 degrees.
    SGM::Rotate(Origin, Axis, dAngle, Trans);
    SGM::TransformEntity(rResult, Trans, IDCurve2);
    SGM::Edge Edge2 = SGM::CreateEdge(rResult, IDCurve2);

    SGM::UnitVector3D Normal = XAxis * YAxis;
    SGM::Surface SurfaceID = SGM::CreateTorusSurface(rResult, Center, Normal, dr, dR);
    std::vector<SGM::Edge> aEdges;
    aEdges.push_back(Edge1);
    aEdges.push_back(Edge2);
    std::vector<SGM::EdgeSideType> aTypes;
    aTypes.push_back(SGM::EdgeSideType::FaceOnLeftType);
    aTypes.push_back(SGM::EdgeSideType::FaceOnRightType);
    SGM::Body BodyID = SGM::CreateSheetBody(rResult, SurfaceID, aEdges, aTypes);

    SGM::CheckOptions Options;
    std::vector<std::string> aCheckStrings;
    SGM::CheckEntity(rResult,BodyID,Options,aCheckStrings);

    SGMTesting::ReleaseTestThing(pThing);
    }
    
TEST(create_check, create_attributes)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<int> aIntegers(3,1);
    SGMInternal::IntegerAttribute * pIntegerAttribute = new SGMInternal::IntegerAttribute(rResult,
        "IntegerAttributeTestName", aIntegers);
    EXPECT_TRUE(pIntegerAttribute != nullptr);
    EXPECT_EQ(pIntegerAttribute->GetAttributeType(),SGM::EntityType::IntegerAttributeType);

    std::vector<double> aDoubles(3,1.0);
    SGMInternal::DoubleAttribute * pDoubleAttribute = new SGMInternal::DoubleAttribute(rResult,
        "DoubleAttributeTestName", aDoubles);
    EXPECT_TRUE(pDoubleAttribute != nullptr);
    EXPECT_EQ(pDoubleAttribute->GetAttributeType(),SGM::EntityType::DoubleAttributeType);

    std::vector<char> aChars(3,'c');
    SGMInternal::CharAttribute * pCharAttribute = new SGMInternal::CharAttribute(rResult,
        "CharAttributeTestName", aChars);
    EXPECT_TRUE(pCharAttribute != nullptr);
    EXPECT_EQ(pCharAttribute->GetAttributeType(),SGM::EntityType::CharAttributeType);

    SGMInternal::StringAttribute * pStringAttribute = new SGMInternal::StringAttribute(rResult,
        "StringAttributeTestName", "string_test");
    EXPECT_TRUE(pStringAttribute != nullptr);
    EXPECT_EQ(pStringAttribute->GetAttributeType(),SGM::EntityType::StringAttributeType);

    SGMTesting::ReleaseTestThing(pThing);
    }

TEST(create_check, miscellaneous_entity)
    {
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    // cloning a thing is not currently allowed
    EXPECT_THROW(pThing->Clone(rResult), std::logic_error);

    SGMTesting::ReleaseTestThing(pThing);
    }


