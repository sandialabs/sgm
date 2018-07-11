#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMIntersector.h"

#include "test_utility.h"

TEST(intersection_check, intersect_ellipse_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(1,2,3);
    SGM::UnitVector3D XAxis(1,0,0);
    SGM::UnitVector3D YAxis(0,1,0);
    double dXRadius = 5;
    double dYRadius = 7;
    SGM::Curve EllipseCurve = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // coincident plane
    SGM::Point3D PlaneOrigin(1,2,3);
    SGM::UnitVector3D PlaneNormal(0,0,1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    SGM::Point3D ExpectedPoint = Center + 5*(XAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);

    // plane rotated about the y axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(1,0,2);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    SGM::Point3D ExpectedPoint1 = Center - dYRadius*(YAxis);
    SGM::Point3D ExpectedPoint2 = Center + dYRadius*(YAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    // ellipse rotated about z
    XAxis = SGM::UnitVector3D(1,2,0);
    YAxis = SGM::UnitVector3D(2,-1,0);
    SGM::Curve EllipseCurve2 = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // plane rotated about x axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(2,-1,1);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    ExpectedPoint1 = Center + dXRadius*(XAxis);
    ExpectedPoint2 = Center - dXRadius*(XAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    // plane tangent to the ellipse
    SGM::Point3D PosOnEllipse;
    SGM::Vector3D Tangent;
    double dUvalue = SGM_PI/5.0;
    SGM::EvaluateCurve(rResult, EllipseCurve2, dUvalue, &PosOnEllipse, &Tangent);

    SGM::UnitVector3D EllipseNormal = XAxis*YAxis;
    PlaneOrigin = PosOnEllipse + EllipseNormal*11.0;
    PlaneNormal = Tangent*EllipseNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // plane misses ellipse by slightly more than tolerance
    PlaneOrigin = PlaneOrigin + (1.0e-5)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);

    //// plane misses ellipse by slightly less than tolerance
    //PlaneOrigin = PosOnEllipse + (1.0e-11)*PlaneNormal;

    //aPoints.clear();
    //aTypes.clear();
    //SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    //EXPECT_EQ(aPoints.size(), 1);
    //EXPECT_EQ(aTypes.size(), 1);
    //EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    //EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_line_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos(-1,2,-3);
    SGM::UnitVector3D Direction(1,2,3);
    SGM::Curve LineID = SGM::CreateLine(rResult, Pos, Direction);

    SGM::Point3D PlaneOrigin(0,0,0);
    SGM::UnitVector3D PlaneNorm(-1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    SGM::Point3D ExpectedPos(0,4,0);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);


    PlaneOrigin = SGM::Point3D(-.5,3,-1.5);
    PlaneNorm = Direction.Orthogonal();

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_circle_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(7,6,-5);
    SGM::UnitVector3D Normal(1,2,1);
    double dRadius=5.5;
    SGM::Curve CircleID = SGM::CreateCircle(rResult, Center, Normal, dRadius);

    // coincident plane
    SGM::Point3D PlaneOrigin = Center + 5*Normal.Orthogonal();
    SGM::UnitVector3D PlaneNorm(-1,-2,-1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);

    // two intersection points
    PlaneOrigin = Center + SGM::Vector3D(1,0,0);
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    for (SGM::Point3D TestPos : aPoints)
    {
      EXPECT_NEAR(Center.Distance(TestPos), 5.5, dTolerance);
    }

    // tangent intersection
    SGM::Point3D PosOnCircle;
    SGM::UnitVector3D Tangent;
    SGM::EvaluateCurve(rResult, CircleID, SGM_PI*0.7, &PosOnCircle, &Tangent);
    PlaneOrigin = PosOnCircle + 3*Tangent;
    PlaneNorm = Tangent*Normal + Normal; // 45 degrees to plane of circle

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    EXPECT_TRUE(SGM::NearEqual(PosOnCircle, aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_parabola_and_plane)
{
    SGMInternal::thing *pThing=SGM::CreateThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(-3,2,1);
    SGM::UnitVector3D XAxis(1,0,2);
    SGM::UnitVector3D YAxis(2,0,-1);
    double dA = -0.3;
    SGM::Curve ParabolaID = SGM::CreateParabola(rResult, Center, XAxis, YAxis, dA);

    // coincident plane
    SGM::Point3D PlaneOrigin(6.5,2,16.2);
    SGM::UnitVector3D PlaneNorm(0,1,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);

    // single clean intersection point
    PlaneNorm = XAxis;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // single tangent intersection point
    SGM::Point3D Pos;
    SGM::Vector3D Tangent;
    SGM::UnitVector3D Normal = (XAxis*YAxis);
    SGM::EvaluateCurve(rResult, ParabolaID, 1.4, &Pos, &Tangent);

    PlaneNorm = Normal*Tangent + 2*Normal;  // not perpendicular
    PlaneOrigin = Pos + 20.6*Tangent;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    EXPECT_TRUE(SGM::NearEqual(aPoints[0], Pos, dTolerance));

    // two intersection points
    PlaneNorm = SGM::Vector3D(-1,0,3);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    SGM::DeleteThing(pThing);
}

TEST(intersection_check, intersect_hyperbola_and_plane)
{
    SGMInternal::thing *pThing=SGM::CreateThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);//3,-4,-5);
    SGM::UnitVector3D XAxis(1,0,0);//0,2,1);
    SGM::UnitVector3D YAxis(0,1,0);//0,1,-2);
    double dA = 2.0;//1.1;
    double dB = 1.0;//0.7;
    SGM::Curve HyperbolaID = SGM::CreateHyperbola(rResult, Center, XAxis, YAxis, dA, dB);

    // coincident plane
    SGM::Point3D PlaneOrigin(0,10,0);//3,16,15);
    SGM::UnitVector3D PlaneNorm(0,0,1);//-1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(4.898979, 10, 0), aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-4.898979, 10, 0), aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    PlaneOrigin = SGM::Point3D(-10,0,0);
    PlaneNorm = SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-10, 20.099751, 0), aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // in y-z plane, not aligned with axes
    SGM::Point3D Center2(3,-4,-5);
    SGM::UnitVector3D XAxis2(0,2,1);
    SGM::UnitVector3D YAxis2(0,1,-2);
    double dA2 = .7;
    double dB2 = 1.1;
    SGM::Curve HyperbolaID2 = SGM::CreateHyperbola(rResult, Center2, XAxis2, YAxis2, dA2, dB2);

    PlaneOrigin = SGM::Point3D(0,0,-10);
    PlaneNorm = SGM::UnitVector3D(0,.75,-2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    SGM::DeleteThing(pThing);
}
