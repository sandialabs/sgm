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

torus::torus(SGM::Result &rResult, torus const &other) :
        surface(rResult, other),
        m_Center(other.m_Center),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_ZAxis(other.m_ZAxis),
        m_dMinorRadius(other.m_dMinorRadius),
        m_dMajorRadius(other.m_dMajorRadius),
        m_nKind(other.m_nKind),
        m_aSeedPoints(other.m_aSeedPoints),
        m_aSeedParams(other.m_aSeedParams)
{}

torus* torus::Clone(SGM::Result &rResult) const
{ return new torus(rResult, *this); }

void torus::Evaluate(SGM::Point2D const &uv,
                     SGM::Point3D       *Pos,
                     SGM::Vector3D      *Du,
                     SGM::Vector3D      *Dv,
                     SGM::UnitVector3D  *Norm,
                     SGM::Vector3D      *Duu,
                     SGM::Vector3D      *Duv,
                     SGM::Vector3D      *Dvv) const
    {
    double du=uv.m_u;
    if(m_nKind==SGM::TorusKindType::LemonType)
        {
        du+=SGM_PI;
        }

    double dCosU=cos(du);
    double dSinU=sin(du);
    double dCosV=cos(uv.m_v);
    double dSinV=sin(uv.m_v);

    double dCosVMR = dCosV*m_dMinorRadius;
    double dMCV= m_dMajorRadius + dCosVMR;
    if(Pos)
        {
        double dSM=dSinV*m_dMinorRadius;
        Pos->m_x=m_Center.m_x+(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dMCV+m_ZAxis.m_x*dSM;
        Pos->m_y=m_Center.m_y+(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dMCV+m_ZAxis.m_y*dSM;
        Pos->m_z=m_Center.m_z+(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dMCV+m_ZAxis.m_z*dSM;
        }
    if(Du)
        {
        Du->m_x=(m_YAxis.m_x*dCosU-m_XAxis.m_x*dSinU)*dMCV;
        Du->m_y=(m_YAxis.m_y*dCosU-m_XAxis.m_y*dSinU)*dMCV;
        Du->m_z=(m_YAxis.m_z*dCosU-m_XAxis.m_z*dSinU)*dMCV;
        }
    if(Dv)
        {
        double dVdMCV=-dSinV*m_dMinorRadius;
        double dVdSM=dCosVMR;
        Dv->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dVdMCV+m_ZAxis.m_x*dVdSM;
        Dv->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dVdMCV+m_ZAxis.m_y*dVdSM;
        Dv->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dVdMCV+m_ZAxis.m_z*dVdSM;
        }
    if(Norm)
        {
        double dCM=dCosVMR;
        double dSM=dSinV*m_dMinorRadius;
        Norm->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dCM+m_ZAxis.m_x*dSM;
        Norm->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dCM+m_ZAxis.m_y*dSM;
        Norm->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dCM+m_ZAxis.m_z*dSM;
        double dScale=1.0/sqrt(Norm->m_x*Norm->m_x+Norm->m_y*Norm->m_y+Norm->m_z*Norm->m_z);
        Norm->m_x*=dScale;
        Norm->m_y*=dScale;
        Norm->m_z*=dScale;
        }
    if(Duu)
        {
        Duu->m_x=(-m_YAxis.m_x*dSinU-m_XAxis.m_x*dCosU)*dMCV;
        Duu->m_y=(-m_YAxis.m_y*dSinU-m_XAxis.m_y*dCosU)*dMCV;
        Duu->m_z=(-m_YAxis.m_z*dSinU-m_XAxis.m_z*dCosU)*dMCV;
        }
    if(Duv)
        {
        double dVdMCV=-dSinV*m_dMinorRadius;
        Duv->m_x=(m_YAxis.m_x*dCosU-m_XAxis.m_x*dSinU)*dVdMCV;
        Duv->m_y=(m_YAxis.m_y*dCosU-m_XAxis.m_y*dSinU)*dVdMCV;
        Duv->m_z=(m_YAxis.m_z*dCosU-m_XAxis.m_z*dSinU)*dVdMCV;
        }
    if(Dvv)
        {
        double ddVdMCV=-dCosVMR;
        double ddVdSM=-dSinV*m_dMinorRadius;
        Dvv->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*ddVdMCV+m_ZAxis.m_x*ddVdSM;
        Dvv->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*ddVdMCV+m_ZAxis.m_y*ddVdSM;
        Dvv->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*ddVdMCV+m_ZAxis.m_z*ddVdSM;
        }
    }

SGM::Point2D torus::Inverse(SGM::Point3D const &Pos,
                            SGM::Point3D       *ClosePos,
                            SGM::Point2D const *pGuess) const
    {
    // Find the u value.

    double x=Pos.m_x-m_Center.m_x;
    double y=Pos.m_y-m_Center.m_y;
    double z=Pos.m_z-m_Center.m_z;

    double dUx=x*m_XAxis.m_x+y*m_XAxis.m_y+z*m_XAxis.m_z;
    double dUy=x*m_YAxis.m_x+y*m_YAxis.m_y+z*m_YAxis.m_z;
    double dU=SGM::SAFEatan2(dUy,dUx);

    // Find the v value.

    SGM::UnitVector3D Spoke=(Pos-m_ZAxis*((Pos-m_Center)%m_ZAxis))-m_Center;
    if(GetKind()==SGM::TorusKindType::LemonType)
        {
        Spoke.Negate();
        }
    double cx=Pos.m_x-m_Center.m_x-Spoke.m_x*m_dMajorRadius;
    double cy=Pos.m_y-m_Center.m_y-Spoke.m_y*m_dMajorRadius;
    double cz=Pos.m_z-m_Center.m_z-Spoke.m_z*m_dMajorRadius;
    double dVx=cx*Spoke.m_x+cy*Spoke.m_y+cz*Spoke.m_z;
    double dVy=cx*m_ZAxis.m_x+cy*m_ZAxis.m_y+cz*m_ZAxis.m_z;
    double dV=SGM::SAFEatan2(dVy,dVx);

    // Adjust to the domain.

    while(dV<m_Domain.m_VDomain.m_dMin)
        {
        dV+=SGM_TWO_PI;
        }
    while(dU<m_Domain.m_UDomain.m_dMin)
        {
        dU+=SGM_TWO_PI;
        }
    while(dV>m_Domain.m_VDomain.m_dMax)
        {
        dV-=SGM_TWO_PI;
        }
    while(dU>m_Domain.m_UDomain.m_dMax)
        {
        dU-=SGM_TWO_PI;
        }

    // fix apples and lemons

    if( m_nKind==SGM::TorusKindType::AppleType ||
        m_nKind==SGM::TorusKindType::LemonType)
        {
        if(!m_Domain.m_VDomain.InInterval(dV, SGM_ZERO))
            {
            SGM::Point3D NorthPole,SouthPole;
            SGM::Point2D uv1(dU,m_Domain.m_VDomain.m_dMax);
            Evaluate(uv1,&NorthPole);
            SGM::Point2D uv2(dU,m_Domain.m_VDomain.m_dMin);
            Evaluate(uv2,&SouthPole);
            double dNorth=NorthPole.DistanceSquared(Pos);
            double dSouth=SouthPole.DistanceSquared(Pos);
            if(dNorth<dSouth)
                {
                dV=m_Domain.m_VDomain.m_dMax;
                }
            else
                {
                dV=m_Domain.m_VDomain.m_dMin;
                }
            }
        }

    if(pGuess)
        {
        // Check for points on the axis, and on the seam.

        SGM::Vector3D TestVec=Pos-m_Center;
        if( SGM::NearEqual(TestVec%m_YAxis,0,SGM_MIN_TOL,false) )
            {
            double dXDot=TestVec%m_XAxis;
            if( SGM_MIN_TOL<dXDot)
                {
                // On seam.
                if(SGM_HALF_PI<pGuess->m_u)
                    {
                    dU=SGM_TWO_PI;
                    }
                else
                    {
                    dU=0.0;
                    }
                }
            else if(-SGM_MIN_TOL<dXDot)
                {
                // On axis.
                dU=pGuess->m_u;
                }
            }
        }

    SGM::Point2D uv = {dU,dV};
    if(ClosePos)
        {
        Evaluate(uv,ClosePos);
        }

    return uv;
    }

bool torus::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    bool bAnswer=true;
    auto pTorus2=(torus const *)pOther;
    if(SGM::NearEqual(m_dMajorRadius,pTorus2->m_dMajorRadius,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else if(SGM::NearEqual(m_dMinorRadius,pTorus2->m_dMinorRadius,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else if(SGM::NearEqual(fabs(m_ZAxis%pTorus2->m_ZAxis),1.0,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else if(SGM::NearEqual(m_Center,pTorus2->m_Center,dTolerance)==false)
        {
        bAnswer=false;
        }
    return bAnswer;
    }

void torus::Transform(SGM::Result            &,//rResult,
                      SGM::Transform3D const &Trans)
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