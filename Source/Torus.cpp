#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include <cmath>

namespace SGMInternal
{
torus::torus(SGM::Result             &rResult,
             SGM::Point3D      const &Center,
             SGM::UnitVector3D const &ZAxis,
             double                   dMinorRadius,
             double                   dMajorRadius,
             bool                     bApple,
             SGM::UnitVector3D const *XAxis):
    surface(rResult,SGM::TorusType),m_Center(Center),m_ZAxis(ZAxis),
    m_dMinorRadius(dMinorRadius),m_dMajorRadius(dMajorRadius)
    {
    if(XAxis)
        {
        m_XAxis=*XAxis;
        }
    else
        {
        m_XAxis=ZAxis.Orthogonal();
        }

    m_YAxis=m_ZAxis*m_XAxis;
    m_bClosedU=true;
    m_Domain.m_UDomain.m_dMin=0.0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;

    if(m_dMajorRadius+SGM_ZERO<m_dMinorRadius)
        {
        double dT=SGM::SAFEacos(m_dMajorRadius/m_dMinorRadius);
        if(bApple) // Apple Torus
            {
            m_Domain.m_VDomain.m_dMin=dT-SGM_PI;
            m_Domain.m_VDomain.m_dMax=SGM_PI-dT;
            m_bSingularLowV=true;
            m_bSingularHighV=true;
            m_nKind=SGM::TorusKindType::AppleType;
            }
        else // Lemon Torus
            {
            m_Domain.m_VDomain.m_dMin=SGM_PI-dT;
            m_Domain.m_VDomain.m_dMax=SGM_PI+dT;
            m_bSingularLowV=true;
            m_bSingularHighV=true;
            m_nKind=SGM::TorusKindType::LemonType;
            }
        }
    else if(fabs(m_dMajorRadius-m_dMinorRadius)<SGM_ZERO) // Pinched Torus
        {
        m_bClosedV=true;
        m_Domain.m_VDomain.m_dMin=-SGM_PI;
        m_Domain.m_VDomain.m_dMax=SGM_PI;
        m_bSingularLowV=true;
        m_bSingularHighV=true;
        m_nKind=SGM::TorusKindType::PinchedType;
        }
    else // Normal Torus
        {
        m_bClosedV=true;
        m_Domain.m_VDomain.m_dMin=0.0;
        m_Domain.m_VDomain.m_dMax=SGM_TWO_PI;
        m_nKind=SGM::TorusKindType::DonutType;
        }
    }

void torus::Transform(SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    m_dMinorRadius*=Trans.Scale();
    m_dMajorRadius*=Trans.Scale();
    }

curve* torus::UParamLine(SGM::Result &rResult, double dU) const
    {
    SGM::Interval2D const &Domain=GetDomain();
    SGM::Point2D uv0(dU,Domain.m_VDomain.MidPoint(0.0));
    SGM::Point2D uv1(dU,Domain.m_VDomain.MidPoint(0.3));
    SGM::Point2D uv2(dU,Domain.m_VDomain.MidPoint(0.7));
    SGM::Point3D Pos0,Pos1,Pos2;
    Evaluate(uv0,&Pos0);
    Evaluate(uv1,&Pos1);
    Evaluate(uv2,&Pos2);
    SGM::Point3D Center;
    SGM::UnitVector3D Normal;
    double dRadius;
    SGM::FindCircle(Pos0,Pos1,Pos2,Center,Normal,dRadius);
    SGM::UnitVector3D XAxis=Pos0-Center;
    return new circle(rResult,Center,Normal,dRadius,&XAxis);
    }

curve *torus::VParamLine(SGM::Result &rResult, double dV) const
    {
    SGM::Interval2D const &Domain=GetDomain();
    SGM::Point2D uv0(Domain.m_UDomain.MidPoint(0.0),dV);
    SGM::Point2D uv1(Domain.m_UDomain.MidPoint(0.3),dV);
    SGM::Point2D uv2(Domain.m_UDomain.MidPoint(0.7),dV);
    SGM::Point3D Pos0,Pos1,Pos2;
    Evaluate(uv0,&Pos0);
    Evaluate(uv1,&Pos1);
    Evaluate(uv2,&Pos2);
    SGM::Point3D Center;
    SGM::UnitVector3D Normal;
    double dRadius;
    SGM::FindCircle(Pos0,Pos1,Pos2,Center,Normal,dRadius);
    SGM::UnitVector3D XAxis=Pos0-Center;
    return new circle(rResult,Center,Normal,dRadius,&XAxis);
    }

void FindSeedPoints(torus               const *pTorus,
                    std::vector<SGM::Point3D> &aSeedPoints,
                    std::vector<SGM::Point2D> &aSeedParams)
    {
    SGM::Interval2D const &Domain=pTorus->GetDomain();
    size_t Index1,Index2;
    for(Index1=0;Index1<4;++Index1)
        {
        double u=Domain.m_UDomain.MidPoint(0.0125+Index1/4.0);
        for(Index2=0;Index2<4;++Index2)
            {
            double v=Domain.m_VDomain.MidPoint(Index2/4.0);
            SGM::Point2D uv(u,v);
            aSeedParams.push_back(uv);
            SGM::Point3D Pos;
            pTorus->Evaluate(uv,&Pos);
            aSeedPoints.push_back(Pos);
            }
        }
    }

std::vector<SGM::Point3D> const &torus::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FindSeedPoints(this,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<SGM::Point2D> const &torus::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FindSeedPoints(this,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedParams;
    }
}