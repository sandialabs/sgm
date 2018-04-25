#include "SGMDataClasses.h"
#include "SGMIntersecors.h"
#include "Intersectors.h"
#include "EntityClasses.h"
#include <cfloat>
#include <cmath>
#include <vector>

void IntersectNonParallelPlanes(SGM::Point3D      const &Origin1,
                                SGM::UnitVector3D const &Normal1,
                                SGM::Point3D      const &Origin2,
                                SGM::UnitVector3D const &Normal2,
                                SGM::Point3D            &Origin,
                                SGM::UnitVector3D       &Axis)
    {
    double AxisX=Normal1.m_y*Normal2.m_z-Normal1.m_z*Normal2.m_y;
    double AxisY=Normal1.m_z*Normal2.m_x-Normal1.m_x*Normal2.m_z;
    double AxisZ=Normal1.m_x*Normal2.m_y-Normal1.m_y*Normal2.m_x;

    double dx=fabs(AxisX);
    double dy=fabs(AxisY);
    double dz=fabs(AxisZ);

    double dLength=sqrt(AxisX*AxisX+AxisY*AxisY+AxisZ*AxisZ);
    Axis.m_x=AxisX/dLength;
    Axis.m_y=AxisY/dLength;
    Axis.m_z=AxisZ/dLength;

    double dPlaneDist1=-Origin1.m_x*Normal1.m_x-Origin1.m_y*Normal1.m_y-Origin1.m_z*Normal1.m_z;
    double dPlaneDist2=-Origin2.m_x*Normal1.m_x-Origin2.m_y*Normal1.m_y-Origin2.m_z*Normal1.m_z;
    if(dy<=dx && dz<=dx)
        {
        Origin.m_x=0.0;
        Origin.m_y=(dPlaneDist2*Normal1.m_z-dPlaneDist1*Normal2.m_z)/AxisX;
        Origin.m_z=(dPlaneDist1*Normal2.m_y-dPlaneDist2*Normal1.m_y)/AxisX;
        }
    else if(dz<dy)
        {
        Origin.m_x=(dPlaneDist1*Normal2.m_z-dPlaneDist2*Normal1.m_z)/AxisY;
        Origin.m_y=0.0;
        Origin.m_z=(dPlaneDist2*Normal1.m_x-dPlaneDist1*Normal2.m_x)/AxisY;
        }
    else
        {
        Origin.m_x=(dPlaneDist2*Normal1.m_y-dPlaneDist1*Normal2.m_y)/AxisZ;
        Origin.m_y=(dPlaneDist1*Normal2.m_x-dPlaneDist2*Normal1.m_x)/AxisZ;
        Origin.m_z=0.0;
        }
    }

double FindLocalMin(curve  const *pCurve1,    // Input
                    curve  const *pCurve2,    // Input
                    double       &t1,         // Input and output
                    double       &t2,         // Input and output
                    SGM::Point3D &Pos1,       // Output
                    SGM::Point3D &Pos2)       // Output
    {
    pCurve1->Evaluate(t1,&Pos1);
    pCurve2->Evaluate(t2,&Pos2);
    double dLast=DBL_MAX;
    double dDist2=Pos1.DistanceSquared(Pos2);
    size_t nCount=0;
    while(1E-24<fabs(dLast-dDist2) && nCount<100)
        {
        dLast=dDist2;
        t1=pCurve1->Inverse(Pos2,&Pos1,&t1);
        t2=pCurve2->Inverse(Pos1,&Pos2,&t2);
        dDist2=Pos1.DistanceSquared(Pos2);
        ++nCount;
        }
    return sqrt(dDist2);
    }

size_t IntersectLineAndCylinder(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                cylinder                     const *pCylinder,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, coincident, one tangent point, or two points.

    SGM::Point3D Center=pCylinder->m_Origin;
    SGM::UnitVector3D CylinderAxis=pCylinder->m_ZAxis;
    double dRadius=pCylinder->m_dRadius;
    SGM::Segment3D Seg1(Origin,Origin+Axis);
    SGM::Segment3D Seg2(Center,Center+CylinderAxis);
    SGM::Point3D Pos1,Pos2;
    Seg1.Intersect(Seg2,Pos1,Pos2);
    if(SGM::NearEqual(Pos1,Pos2,dTolerance))
        {
        if(fabs(Axis%CylinderAxis)<1.0-dTolerance)
            {
            SGM::UnitVector3D LineAxis=CylinderAxis*Axis*CylinderAxis;
            SGM::Point3D LineOrigin=Center-((Origin-Center)%CylinderAxis)*CylinderAxis;
            double dCloseT=(Center-LineOrigin)%LineAxis;
            SGM::Point3D Pos=LineOrigin+dCloseT*LineAxis;
            double dDist=Pos.Distance(Center);
            SGM::Point3D TestPos=Origin+Axis;
            SGM::Point3D ProjectedTestPos=TestPos-((TestPos-Center)%CylinderAxis)*CylinderAxis;
            double dScale=LineOrigin.Distance(ProjectedTestPos);
            if(fabs(dDist-dRadius)<dTolerance)
                {
                aPoints.push_back(Origin+(dCloseT*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            else if(dDist<dRadius)
                {
                SGM::Point3D ClosePos=Origin+(dCloseT*dScale)*Axis;
                double t=sqrt(dRadius*dRadius-dDist*dDist);
                aPoints.push_back(ClosePos+(t*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::PointType);
                aPoints.push_back(ClosePos-(t*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::PointType);
                }
            }
        else 
            {
            SGM::Point3D CylinderPos;
            pCylinder->Inverse(Origin,&CylinderPos);
            if(CylinderPos.DistanceSquared(Origin)<dTolerance*dTolerance)
                {
                aPoints.push_back(Origin+Domain.m_dMin*Axis);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                aPoints.push_back(Origin+Domain.m_dMax*Axis);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                }
            }
        return aPoints.size();
        }

    return 0;
    }

size_t IntersectLineAndLine(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &Domain,
                            line                         const *pLine,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes)
    {
    SGM::Segment3D Seg1(Origin,Origin+Axis);
    SGM::Point3D const &Origin2=pLine->GetOrigin();
    SGM::UnitVector3D const &Axis2=pLine->GetAxis();
    SGM::Segment3D Seg2(Origin2,Origin2+Axis2);
    SGM::Point3D Pos1,Pos2;
    Seg1.Intersect(Seg2,Pos1,Pos2);
    if(SGM::NearEqual(Pos1,Pos2,dTolerance))
        {
        if(fabs(Axis%Axis2)<1.0-dTolerance)
            {
            aPoints.push_back(SGM::MidPoint(Pos1,Pos2));
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else
            {
            aPoints.push_back(Origin+Axis*Domain.m_dMin);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            aPoints.push_back(Origin+Axis*Domain.m_dMax);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             SGM::Point3D                 const &PlaneOrigin,
                             SGM::UnitVector3D            const &PlaneNorm,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, coincident, or one point.

    SGM::Point3D Pos0=Origin+Axis*Domain.m_dMin;
    SGM::Point3D Pos1=Origin+Axis*Domain.m_dMin;
    double dDist0=(Pos0-PlaneOrigin)%PlaneNorm;
    double dDist1=(Pos1-PlaneOrigin)%PlaneNorm;
    if(fabs(dDist0)<dTolerance && fabs(dDist1)<dTolerance)
        {
        aPoints.push_back(Pos0);
        aPoints.push_back(Pos1);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        return 2;
        }
    if(dDist0*dDist1<0)
        {
        double dFraction;
        if(dDist0<0)
            {
            dFraction=dDist0/(dDist0-dDist1);
            }
        else
            {
            dFraction=dDist0/(dDist1-dDist0);
            }
        aPoints.push_back(SGM::MidPoint(Pos0,Pos1,dFraction));
        aTypes.push_back(SGM::IntersectionType::PointType);
        return 1;
        }
    else
        {
        if(fabs(dDist0)<dTolerance)
            {
            aPoints.push_back(Pos0);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        else if(fabs(dDist1)<dTolerance)
            {
            aPoints.push_back(Pos1);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        return 0;
        }
    }

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             plane                        const *pPlane,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    return IntersectLineAndPlane(Origin,Axis,Domain,pPlane->m_Origin,pPlane->m_ZAxis,
                                 dTolerance,aPoints,aTypes);
    }

size_t IntersectLineAndCircle(SGM::Point3D                  const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               circle                       const *pCircle,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    IntersectLineAndPlane(Origin,Axis,Domain,pCircle->m_Center,pCircle->m_Normal,dTolerance,aPoints2,aTypes2);
    if(aPoints2.size()==2)
        {
        SGM::Point3D const &Center=pCircle->m_Center;
        SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
        double dDist=Pos.DistanceSquared(Center);
        double dRadius=pCircle->m_dRadius;
        if(dDist<=dRadius-dTolerance)
            {
            double t=sqrt(dRadius*dRadius-dDist*dDist);
            aPoints.push_back(Pos-t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            aPoints.push_back(Pos+t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else if(fabs(dDist-dRadius)<dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::TangentType);
            }
        }
    else if(aPoints2.size()==0)
        {
        SGM::Point3D Pos;
        pCircle->Inverse(aPoints2[0],&Pos);
        if(Pos.DistanceSquared(aPoints2[0])<dTolerance*dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndSphere(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &,//Domain,
                              sphere                       const *pSphere,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, one tangent point or two points.

    SGM::Point3D const &Center=pSphere->m_Center;
    SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
    double dDist=Pos.DistanceSquared(Center);
    double dRadius=pSphere->m_dRadius;
    if(dDist<=dRadius-dTolerance)
        {
        double t=sqrt(dRadius*dRadius-dDist*dDist);
        aPoints.push_back(Pos-t*Axis);
        aTypes.push_back(SGM::IntersectionType::PointType);
        aPoints.push_back(Pos+t*Axis);
        aTypes.push_back(SGM::IntersectionType::PointType);
        }
    else if(fabs(dDist-dRadius)<dTolerance)
        {
        aPoints.push_back(Pos);
        aTypes.push_back(SGM::IntersectionType::TangentType);
        }
    return aPoints.size();
    }

size_t IntersectLineAndCone(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &,//Domain,
                            cone                         const *pCone,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes)
    {
    //  Empty, one point tangent or not, two points.

    // A point is on a cone with apex at the origin and axis the positive 
    // z-axis if and only if x^2+y^2=(cos(half angle)/sin(half angle))^2*z^2.

    double dCosHalfAngle=pCone->m_dCosHalfAngle;
    double dSinHalfAngle=pCone->m_dSinHalfAngle;
    double dCoTan=dCosHalfAngle/dSinHalfAngle;
    double s=dCoTan*dCoTan;

    // x^2+y^2=s*z^2
    //
    // x=a+t*b
    // y=c+t*d
    // z=e+t*f
    //
    // (a+t*b)^2+(c+t*d)^2-s*(e+t*f)^2=0

    double a=Origin.m_x;
    double c=Origin.m_y;
    double e=Origin.m_z;
    double b=Axis.m_x;
    double d=Axis.m_y;
    double f=Axis.m_z;

    double A = d*d-f*f*s;
    double B = 2*a*b+2*c*d-2*e*f*s;
    double C = a*a+c*c-e*e*s;

    std::vector<SGM::Point3D> aHits;
    std::vector<double> aRoots;
    size_t nRoots=SGM::Quadratic(A,B,C,aRoots);
    if(nRoots==0 && SGM_ZERO<fabs(A))
        {
        double x=-B/(2.0*A);
        aHits.push_back(Origin+x*Axis);
        }
    else if(nRoots==1)
        {
        aHits.push_back(Origin+aRoots[0]*Axis);
        }
    else
        {
        aHits.push_back(Origin+aRoots[0]*Axis);
        aHits.push_back(Origin+aRoots[1]*Axis);
        }

    // Check all the hits.

    size_t nHits=aHits.size();
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D const &Pos=aHits[Index1];
        SGM::Point3D CPos;
        pCone->Inverse(Pos,&CPos);
        if(SGM::NearEqual(Pos,CPos,dTolerance))
            {
            aPoints.push_back(CPos);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             double                              dMajorRadius,
                             double                              dMinorRadius,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &)//aTypes)
    {
    // Empty, one tangent point, two points, two tangents points, two points and one tangent point, 
    // three points one tangent, four points.

    // A point is on a torus centered at the origin, with normal (0,0,1), major radius s and 
    // minor radius r if and only if (x^2+y^2+z^2+s^2-r^2)^2 = 4*s^2(x^2+y^2).

    // Finding x, y and z in terms of the parameter t of the given line we have the following;
    // x=a+t*b
    // y=c+t*d
    // z=e+t*f

    // Hence, the intersection of the line and the torus are at the roots of the following quartic;
    // ((a+t*b)^2+(c+t*d)^2+(e+t*f)^2+s^2-r^2)^2-4*s^2((a+t*b)^2+(c+t*d)^2)=0

    double a=Origin.m_x;
    double c=Origin.m_y;
    double e=Origin.m_z;
    double b=Axis.m_x;
    double d=Axis.m_y;
    double f=Axis.m_z;
    double r=dMinorRadius;
    double s=dMajorRadius;

    double a2=a*a,a3=a2*a,a4=a2*a2;
    double b2=b*b,b3=b2*b,b4=b2*b2;
    double c2=c*c,c3=c2*c,c4=c2*c2;
    double d2=d*d,d3=d2*d,d4=d2*d2;
    double e2=e*e,e3=e2*e,e4=e2*e2;
    double f2=f*f,f3=f2*f,f4=f2*f2;
    double r2=r*r,r4=r2*r2;
    double s2=s*s,s4=s2*s2;

    double E=a4+2*a2*c2+c4+2*a2*e2+2*c2*e2+e4-2*a2*r2-2*c2*r2-2*e2*r2+r4-2*a2*s2-2*c2*s2+2*e2*s2-2*r2*s2+s4;
    double D=4*a3*b+4*a*b*c2+4*a2*c*d+4*c3*d+4*a*b*e2+4*c*d*e2+4*a2*e*f+4*c2*e*f+4*e3*f-4*a*b*r2-4*c*d*r2-4*e*f*r2-4*a*b*s2-4*c*d*s2+4*e*f*s2;
    double C=6*a2*b2+2*b2*c2+8*a*b*c*d+2*a2*d2+6*c2*d2+2*b2*e2+2*d2*e2+8*a*b*e*f+8*c*d*e*f+2*a2*f2+2*c2*f2+6*e2*f2-2*b2*r2-2*d2*r2-2*f2*r2-2*b2*s2-2*d2*s2+2*f2*s2;
    double B=4*a*b3+4*b2*c*d+4*a*b*d2+4*c*d3+4*b2*e*f+4*d2*e*f+4*a*b*f2+4*c*d*f2+4*e*f3;
    double A=b4+2*b2*d2+d4+2*b2*f2+2*d2*f2+f4;

    std::vector<double> aRoots;
    size_t nRoots=SGM::Quartic(A,B,C,D,E,aRoots);
    size_t Index1;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        aPoints.push_back(Origin+aRoots[Index1]*Axis);
        }
    return aPoints.size();
    }

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &,//Domain,
                             torus                        const *pTorus,
                             double                              ,//dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, one tangent point, two points, two tangents points, two points and one tangent point, 
    // three points one tangent, four points.

    double dMajorRadius=pTorus->m_dMajorRadius;
    double dMinorRadius=pTorus->m_dMinorRadius;
    size_t nHits=IntersectLineAndTorus(Origin,Axis,dMajorRadius,dMinorRadius,aPoints,aTypes);
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D Pos;
        pTorus->Inverse(aPoints[Index1],&Pos);
        aPoints[Index1]=Pos;
        }
    return nHits;
    }

size_t IntersectLineAndNUBSurface(SGM::Point3D                 const &,//Origin,
                                  SGM::UnitVector3D            const &,//Axis,
                                  SGM::Interval1D              const &,//Domain,
                                  NUBsurface                   const *,//pNUBSurface,
                                  double                              ,//dTolerance,
                                  std::vector<SGM::Point3D>          &,//aPoints,
                                  std::vector<SGM::IntersectionType> &)//aTypes)
    {
    return 0;
    }

size_t IntersectLineAndSurface(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::CylinderType:
            {
            return IntersectLineAndCylinder(Origin,Axis,Domain,(cylinder const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::PlaneType:
            {
            return IntersectLineAndPlane(Origin,Axis,Domain,(plane const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::SphereType:
            {
            return IntersectLineAndSphere(Origin,Axis,Domain,(sphere const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::TorusType:
            {
            return IntersectLineAndTorus(Origin,Axis,Domain,(torus const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::NUBSurfaceType:
            {
            return IntersectLineAndNUBSurface(Origin,Axis,Domain,(NUBsurface const *)pSurface,dTolerance,aPoints,aTypes);
            }
        default:
            {
            throw;
            }
        }
    }

size_t IntersectLineAndSurface(line                         const *pLine,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes,
                               edge                         const *pEdge,
                               face                         const *pFace)
    {
    std::vector<SGM::Point3D> aTempPoints;
    std::vector<SGM::IntersectionType> aTempTypes;
    //size_t nSize=
        ::IntersectLineAndSurface(pLine->m_Origin,pLine->m_Axis,pLine->GetDomain(),
                                           pSurface,dTolerance,aTempPoints,aTempTypes);
    if(pEdge)
        {

        }
    if(pFace)
        {
        
        }
    aPoints=aTempPoints;
    aTypes=aTempTypes;
    return aPoints.size();
    }

size_t IntersectCircleAndPlane(SGM::Point3D                 const &Center,
                               SGM::UnitVector3D            const &Normal,
                               double                              dRadius,
                               plane                        const *pPlane,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    SGM::UnitVector3D PlaneNormal=pPlane->m_ZAxis;
    SGM::Point3D PlaneOrigin=pPlane->m_Origin;
    if(SGM::NearEqual(fabs(Normal%PlaneNormal),1.0,SGM_MIN_TOL,false))
        {
        if(fabs((Center-PlaneOrigin)%PlaneNormal)<dTolerance)
            {
            SGM::UnitVector3D XAxis=Normal.Orthogonal();
            SGM::Point3D CirclePos=Center+XAxis*dRadius;
            aPoints.push_back(CirclePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            aPoints.push_back(CirclePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    else
        {
        SGM::Point3D Origin;
        SGM::UnitVector3D Axis;
        IntersectNonParallelPlanes(Center,Normal,PlaneOrigin,PlaneNormal,Origin,Axis);
        SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
        double dDist=Pos.DistanceSquared(Center);
        if(dDist<=dRadius-dTolerance)
            {
            double t=sqrt(dRadius*dRadius-dDist*dDist);
            aPoints.push_back(Pos-t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            aPoints.push_back(Pos+t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else if(fabs(dDist-dRadius)<dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::TangentType);
            }
        }
    return aPoints.size();
    }

 size_t IntersectCurveAndSurface(SGM::Result                        &rResult,
                                 curve                        const *pCurve,
                                 surface                      const *pSurface,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes,
                                 edge                         const *pEdge,
                                 face                         const *pFace)
     {
     switch(pCurve->GetCurveType())
         {
         case SGM::LineType:
             {
             double dTol=pEdge ? pEdge->GetTolerance() : SGM_MIN_TOL;
             IntersectLineAndSurface((line *)pCurve,pSurface,dTol,aPoints,aTypes,pEdge,pFace);
             break;
             }
         case SGM::CircleType:
             {
             break;
             }
         case SGM::EllipseType:
             {
             break;
             }
         case SGM::ParabolaType:
             {
             break;
             }
         case SGM::HyperboleType:
             {
             break;
             }
         case SGM::NUBCurveType:
             {
             break;
             }
         case SGM::NURBCurveType:
             {
             break;
             }
         case SGM::PointCurveType:
             {
             break;
             }
         default:
             {
             rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
             }
         }
     return aPoints.size();
     }

 size_t IntersectLineAndCurve(SGM::Result                        &rResult,
                              SGM::Point3D                 const &,//Origin,
                              SGM::UnitVector3D            const &,//Axis,
                              SGM::Interval1D              const &,//Domain,
                              curve                        const *pCurve,
                              double                              ,//dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &)//aTypes)
     {
     switch(pCurve->GetCurveType())
         {
         case SGM::LineType:
             {
             break;
             }
         case SGM::CircleType:
             {
             break;
             }
         case SGM::EllipseType:
             {
             break;
             }
         case SGM::ParabolaType:
             {
             break;
             }
         case SGM::HyperboleType:
             {
             break;
             }
         case SGM::NUBCurveType:
             {
             break;
             }
         case SGM::NURBCurveType:
             {
             break;
             }
         case SGM::PointCurveType:
             {
             break;
             }
         default:
             {
             rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
             }
         }
     return aPoints.size();
     }

 size_t IntersectCurves(SGM::Result                        &rResult,
                        curve                        const *pCurve1,
                        curve                        const *pCurve2,
                        std::vector<SGM::Point3D>          &aPoints,
                        std::vector<SGM::IntersectionType> &aTypes,
                        edge                         const *pEdge1,
                        edge                         const *pEdge2)
      {
      double dTol1=pEdge1 ? pEdge1->GetTolerance() : SGM_MIN_TOL;
      double dTol2=pEdge1 ? pEdge2->GetTolerance() : SGM_MIN_TOL;
      double dTol=dTol1+dTol2;

      switch(pCurve1->GetCurveType())
         {
         case SGM::LineType:
             {
             line *pLine=(line *)pCurve1;
             SGM::Interval1D Domain=pCurve1->GetDomain();
             if(pEdge1)
                 {
                 Domain&=pEdge1->GetDomain();
                 }
             IntersectLineAndCurve(rResult,pLine->GetOrigin(),pLine->GetAxis(),Domain,pCurve2,dTol,aPoints,aTypes);
             break;
             }
         case SGM::CircleType:
             {
             break;
             }
         case SGM::EllipseType:
             {
             break;
             }
         case SGM::ParabolaType:
             {
             break;
             }
         case SGM::HyperboleType:
             {
             break;
             }
         case SGM::NUBCurveType:
             {
             break;
             }
         case SGM::NURBCurveType:
             {
             break;
             }
         case SGM::PointCurveType:
             {
             break;
             }
         default:
             {
             rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
             }
         }
      return aPoints.size();
      }