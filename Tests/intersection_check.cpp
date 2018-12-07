#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMIntersector.h"
#include "SGMSegment.h"

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
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint = Center + 5*(XAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // plane rotated about the y axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(1,0,2);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint1 = Center - dYRadius*(YAxis);
        SGM::Point3D ExpectedPoint2 = Center + dYRadius*(YAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

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
    if (aPoints.size() > 1)
    {
        SGM::Point3D ExpectedPoint1 = Center + dXRadius*(XAxis);
        SGM::Point3D ExpectedPoint2 = Center - dXRadius*(XAxis);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // plane tangent to the ellipse
    SGM::Point3D PosOnEllipse;
    SGM::Vector3D Tangent;
    double dUvalue = SGM_PI/5.0;
    SGM::EvaluateCurve(rResult, EllipseCurve2, dUvalue, &PosOnEllipse, &Tangent);

    SGM::UnitVector3D EllipseNormal = XAxis*YAxis;
    PlaneNormal = Tangent*EllipseNormal;
    PlaneOrigin = PosOnEllipse + EllipseNormal*11.0;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // plane misses ellipse by slightly more than tolerance
    PlaneOrigin = PlaneOrigin + (dTolerance*10.0)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // plane misses ellipse by slightly less than tolerance
    PlaneOrigin = PosOnEllipse + (dTolerance*0.1)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // simple case for easier debugging
    SGM::Point3D Center3(0,0,0);
    SGM::UnitVector3D XAxis3(1,1,0);
    SGM::UnitVector3D YAxis3(-1,1,0);
    double dXRadius3 = 2;
    double dYRadius3 = 1;
    SGM::Curve EllipseCurve3 = SGM::CreateEllipse(rResult, Center3, XAxis3, YAxis3, dXRadius3, dYRadius3);

    // tangent plane that just misses 
    SGM::UnitVector3D PlaneNormal3(1,1,0);
    SGM::Point3D PlaneOrigin3(sqrt(2.0), sqrt(2.0), 0);
    PlaneOrigin3 = PlaneOrigin3 + (dTolerance*10.0)*PlaneNormal3;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve3, PlaneOrigin3, PlaneNormal3, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

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
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // coincident intersection
    PlaneOrigin = Pos + 0.5*SGM::Vector3D(1,2,3);
    PlaneNorm = Direction.Orthogonal();

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // parallel plane offset by more than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // parallel plane offset by less than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, LineID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

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
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // two intersection points
    PlaneOrigin = Center + SGM::Vector3D(1,0,0);
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
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
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnCircle, aPoints[0], dTolerance));

    // plane misses by just more than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // plane misses by just less than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, CircleID, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnCircle, aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_parabola_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
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
    if (aTypes.size() > 1)
    {
      EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
      EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }

    // single clean intersection point
    PlaneNorm = XAxis;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
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
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(aPoints[0], Pos, dTolerance));

    // two intersection points
    PlaneNorm = SGM::Vector3D(-1,0,3);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // parallel to tangent plane, but just outside of tolerance
    PlaneNorm = Normal*Tangent;
    PlaneOrigin = Pos + 7.7*Normal + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // tangent just within tolerance
    PlaneOrigin = Pos + 7.7*Normal + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // tangent just within tolerance - tilt the plane
    SGM::UnitVector3D TiltedNormal = PlaneNorm + 0.4*Normal;
    PlaneOrigin = Pos + 7.7*Tangent;
    PlaneOrigin = PlaneOrigin + (dTolerance*0.1)*TiltedNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);


    SGM::Point3D Center2(0.0,-1.0, -1.0);
    SGM::UnitVector3D XAxis2(0.62457719407916179, 0.78096307763952644, 0.0);
    SGM::UnitVector3D YAxis2(0.0, 0.0, -1.0);
    double dA2 = 0.63263978149317612;
    SGM::Curve ParabolaID2 = SGM::CreateParabola(rResult, Center2, XAxis2, YAxis2, dA2);

    //SGM::Interval1D Domain(-10.0, 10.0);
    //SGM::CreateEdge(rResult, ParabolaID2, &Domain);

    SGM::Point3D PlaneOrigin2(10.0, 0.0, -1.0);
    SGM::UnitVector3D PlaneNorm2(0.0, 0.0, 1.0);

    //SGM::CreateDisk(rResult, PlaneOrigin2, PlaneNorm2, 10.0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, ParabolaID2, PlaneOrigin2, PlaneNorm2, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(Center2, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_hyperbola_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(0,0,0);
    SGM::UnitVector3D XAxis(1,0,0);
    SGM::UnitVector3D YAxis(0,1,0);
    double dA = 2.0;
    double dB = 1.0;
    SGM::Curve HyperbolaID = SGM::CreateHyperbola(rResult, Center, XAxis, YAxis, dA, dB);

    // coincident plane
    SGM::Point3D PlaneOrigin(0,10,0);
    SGM::UnitVector3D PlaneNorm(0,0,1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    
    // intersects two points
    PlaneNorm = SGM::UnitVector3D(0,1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(4.898979, 10, 0), aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-4.898979, 10, 0), aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // intersects one point
    PlaneOrigin = SGM::Point3D(-10,0,0);
    PlaneNorm = SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(-10, 20.099751, 0), aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // hyperbola in y-z plane, not aligned with axes
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
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // tangent plane
    SGM::Point3D PosOnHyperbola;
    SGM::Vector3D Tangent;
    SGM::EvaluateCurve(rResult, HyperbolaID2, 1.5, &PosOnHyperbola, &Tangent);

    PlaneOrigin = PosOnHyperbola + 3*Tangent;
    PlaneNorm = Tangent * (XAxis2*YAxis2);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnHyperbola, aPoints[0], dTolerance));

    // tangent offset less than tolerance
    SGM::Point3D PlaneOriginOffset = PlaneOrigin + (dTolerance*0.1)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(PosOnHyperbola, aPoints[0], dTolerance));

    // tangent offset more than tolerance
    PlaneOriginOffset = PlaneOrigin + (dTolerance*10.0)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, HyperbolaID2, PlaneOriginOffset, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_segment_and_segment)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D PosA(1,0,0);
    SGM::Point3D PosB(2,0,0);
    SGM::Point3D PosC(1.5,-1,0);
    SGM::Point3D PosD(1.5,1,0);

    SGM::Segment3D SegmentAB(PosA, PosB);
    SGM::Segment3D SegmentCD(PosC, PosD);

    SGM::Point3D PosAB;
    SGM::Point3D PosCD;
    double sAB;
    double tCD;
    SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    double dTolerance = SGM_MIN_TOL;
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosAB, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosCD, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.5, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // flipped order should give same results
    bool bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosCD, PosAB, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosAB, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1.5,0,0), PosCD, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.5, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // parallel segments
    PosC = SGM::Point3D(1,1,0);
    PosD = SGM::Point3D(2,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel just overlaps at start points
    PosC = SGM::Point3D(1,1,0);
    PosD = SGM::Point3D(-1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel just overlaps at one start and one end point
    PosC = SGM::Point3D(0,1,0);
    PosD = SGM::Point3D(1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // parallel just overlaps at one start and one end point - reverse order
    PosC = SGM::Point3D(0,1,0);
    PosD = SGM::Point3D(1,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentCD.Intersect(SegmentAB, PosCD, PosAB, &tCD, &sAB);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SGM::Point3D(0,0,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, -1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // parallel no overlap
    PosC = SGM::Point3D(-1.5,1,0);
    PosD = SGM::Point3D(0.5,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SGM::Point3D(1,1,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.25, dTolerance, false));

    // parallel no overlap - again
    PosC = SGM::Point3D(2.5,1,0);
    PosD = SGM::Point3D(4,1,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SGM::Point3D(1,1,0), dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, -1.0, dTolerance, false));

    // collinear no overlap
    PosC = SGM::Point3D(2.5,0,0);
    PosD = SGM::Point3D(4,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, -1.0, dTolerance, false));

    // collinear no overlap - reverse one
    PosC = SGM::Point3D(4,0,0);
    PosD = SGM::Point3D(2.5,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_FALSE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 2.0, dTolerance, false));

    // collinear - overlap at single point
    PosC = SGM::Point3D(2,0,0);
    PosD = SGM::Point3D(5,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.0, dTolerance, false));

    // collinear - overlap at single point - reverse second curve
    PosC = SGM::Point3D(5,0,0);
    PosD = SGM::Point3D(2,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // collinear - overlap at single point - reverse first and second curve
    PosC = SGM::Point3D(5,0,0);
    PosD = SGM::Point3D(2,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;

    SGM::Segment3D SegmentABreversed(SegmentAB.m_End, SegmentAB.m_Start);
    
    bSegmentsIntersect = SegmentABreversed.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentABreversed.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentCD.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 1.0, dTolerance, false));

    // collinear - overlap
    PosC = SGM::Point3D(1.75,0,0);
    PosD = SGM::Point3D(3.75,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_End, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 1.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.125, dTolerance, false));

    // collinear - first within second
    PosC = SGM::Point3D(-0.25,0,0);
    PosD = SGM::Point3D(2.25,0,0);
    SegmentCD.m_Start = PosC;
    SegmentCD.m_End = PosD;
    
    bSegmentsIntersect = SegmentAB.Intersect(SegmentCD, PosAB, PosCD, &sAB, &tCD);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    // collinear - second within first
    bSegmentsIntersect = SegmentCD.Intersect(SegmentAB, PosCD, PosAB, &tCD, &sAB);

    EXPECT_TRUE(bSegmentsIntersect);
    EXPECT_TRUE(SGM::NearEqual(PosAB, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(PosCD, SegmentAB.m_Start, dTolerance));
    EXPECT_TRUE(SGM::NearEqual(sAB, 0.0, dTolerance, false));
    EXPECT_TRUE(SGM::NearEqual(tCD, 0.5, dTolerance, false));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_planar_NUBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aInterpolate;
    aInterpolate.reserve(5);

    aInterpolate.emplace_back(0,-2,0);
    aInterpolate.emplace_back(0,-1,0.5);
    aInterpolate.emplace_back(0,0.0,0.25);
    aInterpolate.emplace_back(0,1,0.1);
    aInterpolate.emplace_back(0,2,.5);
    
    SGM::Curve NUBcurveID = SGM::CreateNUBCurve(rResult, aInterpolate);

    // plane with single intersection
    SGM::Point3D PlaneOrigin(1,1,0);
    SGM::UnitVector3D PlaneNormal(0,1,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(0,1,0.1), aPoints[0], dTolerance));

    // coincident plane
    PlaneOrigin = SGM::Point3D(0,21,0);
    PlaneNormal = SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    SGM::Point3D NUBstart;
    SGM::Point3D NUBend;
    SGM::EvaluateCurve(rResult, NUBcurveID, SGM::GetCurveDomain(rResult, NUBcurveID).m_dMin,&NUBstart);
    SGM::EvaluateCurve(rResult, NUBcurveID, SGM::GetCurveDomain(rResult, NUBcurveID).m_dMax,&NUBend);
    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(NUBstart, aPoints[0], dTolerance));
        EXPECT_TRUE(SGM::NearEqual(NUBend, aPoints[1], dTolerance));
    }

    // multiple intersection points
    PlaneOrigin = SGM::Point3D(1,-2,0);
    PlaneNormal = SGM::UnitVector3D(0,-1,4);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (aPoints.size())
    {
        // just checking the first point
        EXPECT_TRUE(SGM::NearEqual(aInterpolate[0], aPoints[0], dTolerance));
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_nonplanar_NUBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point3D> aInterpolate;
    aInterpolate.reserve(6);

    aInterpolate.emplace_back(-2,3.01,0.01);
    aInterpolate.emplace_back(-1,4,0);
    aInterpolate.emplace_back(0,5,0);
    aInterpolate.emplace_back(1,6,0);
    aInterpolate.emplace_back(2,7,0);
    aInterpolate.emplace_back(3,8,0);
    
    SGM::Curve NUBcurveID = SGM::CreateNUBCurve(rResult, aInterpolate);

    SGM::Point3D PlaneOrigin(1,1,0);
    SGM::UnitVector3D PlaneNormal(1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,6,0), aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // tangent intersection - touches without crossing plane
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,-2,-3);
    aInterpolate.emplace_back(-1,-2,-2);
    aInterpolate.emplace_back(0,-1,-1);
    aInterpolate.emplace_back(1,0,-2);
    aInterpolate.emplace_back(2,0,-3);

    SGM::Curve NUBcurveID2 = SGM::CreateNUBCurve(rResult, aInterpolate);
    //SGM::CreateEdge(rResult, NUBcurveID2);

    PlaneOrigin = SGM::Point3D(10,0,-1);
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(0,-1,-1), aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // single intersection point
    SGM::Point3D CurvePos;
    SGM::Vector3D CurveTangent;
    SGM::EvaluateCurve(rResult, NUBcurveID2, 0.3, &CurvePos, &CurveTangent);
    SGM::Vector3D Curvature = SGM::CurveCurvature(rResult, NUBcurveID2, 0.3);
    double dA = Curvature.Magnitude() * 0.5;

    SGM::Curve ParabolaID = SGM::CreateParabola(rResult, CurvePos, CurveTangent, Curvature, dA);
    //SGM::Interval1D Domain(-5,5);
    //SGM::CreateEdge(rResult, ParabolaID, &Domain);


    PlaneOrigin = SGM::Point3D(CurvePos.m_x,0,0);
    PlaneNormal = SGM::UnitVector3D(-1,0,0);

    //SGM::CreateDisk(rResult, PlaneOrigin, PlaneNormal, 5.0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    // two intersection points
    PlaneOrigin = SGM::Point3D(11,4,CurvePos.m_z);
    PlaneNormal = SGM::UnitVector3D(0,0,-1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 1)
    {
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
        // expected 2nd intersection point
        // x is symmetric about 0 so change sign
        // y is symmetric about -1 so compute difference and add back twice
        // z is equal
        SGM::Point3D ExpectedPos(-CurvePos.m_x,CurvePos.m_y+2*(-1-CurvePos.m_y),CurvePos.m_z);
        EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[1], dTolerance));
    }
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    // 1 tangent intersection and one point intersection
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,2,-2);
    aInterpolate.emplace_back(-1.3,2.5,-2);
    aInterpolate.emplace_back(-1.3,2,-2);
    aInterpolate.emplace_back(-1.3,1.5,-2);
    aInterpolate.emplace_back(-0.6,2,-1);
    SGM::Curve NUBcurveID3 = SGM::CreateNUBCurve(rResult, aInterpolate);

    SGM::Vector3D CurveD1, CurveD2;
    SGM::EvaluateCurve(rResult, NUBcurveID3, 0.35, &CurvePos, &CurveD1, &CurveD2);
    PlaneOrigin = CurvePos+.5*CurveD1;
    PlaneNormal = (CurveD1*CurveD2)*CurveD1;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID3, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }


    // tangent intersection - crossing plane
    aInterpolate.clear();
    aInterpolate.emplace_back(-5 ,3,21.2);
    aInterpolate.emplace_back(0  ,4,21);
    aInterpolate.emplace_back(4.5,4,21);
    aInterpolate.emplace_back(5.5,2,21);
    aInterpolate.emplace_back(10 ,2,21);
    aInterpolate.emplace_back(15 ,3,20.8);
    SGM::Curve NUBcurveID4 = SGM::CreateNUBCurve(rResult, aInterpolate);

    //SGM::CreateEdge(rResult, NUBcurveID4);

    SGM::EvaluateCurve(rResult, NUBcurveID4, 0.5, &CurvePos, &CurveD1, &CurveD2);
    PlaneOrigin = CurvePos + CurveD1;
    PlaneNormal = CurveD1.Orthogonal();

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID4, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // near miss - outside tolerance
    aInterpolate.clear();
    aInterpolate.emplace_back(-2,-2,-3);
    aInterpolate.emplace_back(-1.8,-2,-2);
    aInterpolate.emplace_back(-1.6,-1.9,-1);
    aInterpolate.emplace_back(-1.4,-1.8,-2);
    aInterpolate.emplace_back(-1.2,-1.8,-3);
    SGM::Curve NUBcurveID5 = SGM::CreateNUBCurve(rResult, aInterpolate);

    PlaneOrigin = SGM::Point3D(10,0,-1+(5*dTolerance));
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID5, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    // near duplicate, but distinct
    PlaneOrigin = SGM::Point3D(10,0,-1.0000005);
    PlaneNormal = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NUBcurveID5, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_pointcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Pos(55,123,42);
    SGM::Curve PointCurveID = SGM::CreatePointCurve(rResult, Pos);

    SGM::Point3D Origin(-1,1,-1);
    SGM::Point3D PlaneOrigin = Origin;
    SGM::UnitVector3D PlaneNorm = (Pos-Origin).Orthogonal();

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));

    PlaneOrigin = Origin + 0.1*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);
    EXPECT_EQ(aTypes.size(), 0);

    PlaneOrigin = Origin - (0.2e-6)*PlaneNorm;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, PointCurveID, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(Pos, aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_planar_NURBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2);
    aControlPoints.emplace_back(1,1,-1,1);
    aControlPoints.emplace_back(1,1,-2,sqrt(2)/2);
    aControlPoints.emplace_back(1,0,-2,1);
    aControlPoints.emplace_back(1,-1,-2,sqrt(2)/2);
    aControlPoints.emplace_back(1,-1,-3,1);
        
    std::vector<double> aKnots;
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);

    SGM::Curve NURBcurve = SGM::CreateNURBCurve(rResult, aControlPoints, aKnots);

    const double dTolerance = SGM_FIT_SMALL;

    // coincident plane
    SGM::Point3D PlaneOrigin(1,0,0);
    SGM::UnitVector3D PlaneNorm (-1,0,0);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);
    }
    if (aPoints.size() > 1)
    {
      SGM::Point3D ExpectedPos1, ExpectedPos2;
      SGM::Interval1D Domain = SGM::GetDomainOfCurve(rResult, NURBcurve);
      SGM::EvaluateCurve(rResult, NURBcurve, Domain.m_dMin, &ExpectedPos1);
      SGM::EvaluateCurve(rResult, NURBcurve, Domain.m_dMax, &ExpectedPos2);
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos1, aPoints[0], dTolerance));
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos2, aPoints[1], dTolerance));
    }

    // single intersection point
    PlaneOrigin = SGM::Point3D(0, 0, (sqrt(2)/2.0-1));
    PlaneNorm = SGM::UnitVector3D(0,0,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    if (aPoints.size() > 0)
    {
      SGM::Point3D ExpectedPos(1, sqrt(2)/2.0, (sqrt(2)/2.0-1.0));
      EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    }

    // multiple intersection points
    PlaneOrigin = SGM::Point3D(1,-0.1,-2);
    PlaneNorm = SGM::UnitVector3D(0,-1,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 3);
    EXPECT_EQ(aTypes.size(), 3);
    for (int i=0; i<aTypes.size(); ++i)
        EXPECT_EQ(aTypes[i], SGM::IntersectionType::PointType);

    // tangent intersection point
    PlaneOrigin = SGM::Point3D(5, sqrt(2)/2.0, (sqrt(2)/2.0-1.0));
    PlaneNorm = SGM::UnitVector3D(0,1,1);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    SGM::Point3D ClosePos, Pos;
    SGM::Vector3D Tangent;
    double dNURBt = SGM::CurveInverse(rResult, NURBcurve, SGM::Point3D(0, sqrt(2)/2.0, (sqrt(2)/2.0-1.0)), &ClosePos);
    SGM::EvaluateCurve(rResult, NURBcurve, dNURBt, &Pos, &Tangent);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
    if (aPoints.size() > 0)
        EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1, sqrt(2)/2.0, (sqrt(2)/2.0-1.0)), aPoints[0], dTolerance));

    SGMTesting::ReleaseTestThing(pThing);
}

TEST(intersection_check, intersect_nonplanar_NURBcurve_and_plane)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<SGM::Point4D> aControlPoints;
    aControlPoints.emplace_back(1,0,0,1);
    aControlPoints.emplace_back(1,1,0,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,1,-1,1);
    aControlPoints.emplace_back(1,1,-2,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,0,-2,1);
    aControlPoints.emplace_back(1,-1,-2,sqrt(2)/2.0);
    aControlPoints.emplace_back(1,-1,-3,1);
    aControlPoints.emplace_back(1,-1,-4,sqrt(2)/2.0);
    aControlPoints.emplace_back(2,-1,-4,1);
    //aControlPoints.emplace_back(0,1,0,1);
    //aControlPoints.emplace_back(-1,1,0,sqrt(2)/2);
    //aControlPoints.emplace_back(-1,1,-1,1);
    //aControlPoints.emplace_back(-1,1,-2,sqrt(2)/2);
    //aControlPoints.emplace_back(-1,0,-2,1);
        
    std::vector<double> aKnots;
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(0);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_HALF_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_PI*1.5);
    aKnots.push_back(SGM_TWO_PI);
    aKnots.push_back(SGM_TWO_PI);
    aKnots.push_back(SGM_TWO_PI);

    SGM::Curve NURBcurve = SGM::CreateNURBCurve(rResult, aControlPoints, aKnots);

    // one intersection point
    SGM::Point3D PlaneOrigin(1.5,0,-1.5);
    SGM::UnitVector3D PlaneNorm (-1,0,0);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    if (aTypes.size() > 0)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    }
    if (aPoints.size() > 0)
    {
        EXPECT_TRUE(SGM::NearEqual(aPoints[0].m_x, 1.5, dTolerance, false));
        EXPECT_TRUE(SGM::NearEqual(aPoints[0].m_y, -1.0, dTolerance, false));
    }

    // two intersection points
    PlaneOrigin = SGM::Point3D(0,0,-1.5);
    PlaneNorm = SGM::UnitVector3D(0,-1,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (aPoints.size() > 1)
    {
      EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,0,0), aPoints[0], dTolerance));
      EXPECT_TRUE(SGM::NearEqual(SGM::Point3D(1,0,-2), aPoints[1], dTolerance));
    }

    // two intersections, one tangent
    SGM::Point3D CurvePos;
    SGM::Vector3D CurveTangent;
    SGM::EvaluateCurve(rResult, NURBcurve, SGM_PI*0.8, &CurvePos, &CurveTangent);
    PlaneOrigin = CurvePos + SGM::Vector3D(-5,0,0);
    PlaneNorm = CurveTangent * SGM::UnitVector3D(1,0,0);

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, NURBcurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    if (aTypes.size() > 1)
    {
        EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);
        EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);
    }
    if (aPoints.size() > 1)
    {
      EXPECT_TRUE(SGM::NearEqual(CurvePos, aPoints[0], dTolerance));
    }

    //SGM::Interval1D domain = SGM::GetDomainOfCurve(rResult, NURBcurve);
    //SGM::CreateEdge(rResult, NURBcurve, &domain);
    //SGM::Curve curve = SGM::CreatePointCurve(rResult, aPoints[0]);
    //SGM::CreateEdge(rResult, curve);
    //curve = SGM::CreatePointCurve(rResult, aPoints[1]);
    //SGM::CreateEdge(rResult, curve);

    //SGM::CreateDisk(rResult, aPoints[0], PlaneNorm, 3);

    SGMTesting::ReleaseTestThing(pThing); 
}


TEST(intersection_check, intersect_line_and_cylinder)
{
    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Bottom(0,0,-5),Top(0,0,5);
    double dRadius=7.0;
    SGM::Body BodyID = CreateCylinder(rResult, Bottom, Top, dRadius);
    
    SGM::Point3D Origin(4,-4,0);
    SGM::UnitVector3D Axis(1,-1,0);

    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
      
    double dTolerance = SGM_MIN_TOL;
    const bool bUseWholeLine = false;

    SGM::Interval1D Domain(1.e-6,1.e12);

    RayFire(rResult,Origin,Axis, BodyID, aPoints, aTypes, dTolerance, bUseWholeLine);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);

    SGM::Point3D ExpectedPos(sqrt(49./2.),-sqrt(49./2.),0);

    EXPECT_TRUE(SGM::NearEqual(ExpectedPos, aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);

    SGM::Curve LineID = SGM::CreateLine(rResult, Origin, Axis);
    //SGM::Interval1D LineInterval(0.0, 10.0); 
    //SGM::CreateEdge(rResult, LineID, &LineInterval);

    SGM::Curve PointCurveID = SGM::CreatePointCurve(rResult, aPoints[0]);
    //SGM::CreateEdge(rResult, PointCurveID);

    SGMTesting::ReleaseTestThing(pThing); 
}


