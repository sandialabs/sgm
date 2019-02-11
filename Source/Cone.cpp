#include "SGMVector.h"
#include "SGMTransform.h"

#include "Curve.h"
#include "Surface.h"

namespace SGMInternal {

cone::cone(SGM::Result &rResult,
           SGM::Point3D const &Center,
           SGM::UnitVector3D const &ZAxis,
           double dRadius,
           double dHalfAngle,
           SGM::UnitVector3D const *XAxis) :
    surface(rResult, SGM::ConeType),
    m_Origin(Center),
    m_ZAxis(ZAxis)
    {
    m_dSinHalfAngle = sin(dHalfAngle);
    m_dCosHalfAngle = cos(dHalfAngle);

    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin = -SGM_MAX;
    m_Domain.m_VDomain.m_dMax = 1.0 / m_dSinHalfAngle;

    m_bClosedU = true;
    m_bClosedV = false;
    m_bSingularHighV = true;
    m_dRadius = dRadius;

    if (XAxis)
        {
        m_XAxis = *XAxis;
        }
    else
        {
        m_XAxis = m_ZAxis.Orthogonal();
        }
    m_YAxis = m_ZAxis * m_XAxis;
    }

cone::cone(SGM::Result             &rResult,
           SGM::Point3D      const &Bottom,
           SGM::Point3D      const &Top,
           double                   dBottomRadius,
           double                   dTopRadius,
           SGM::UnitVector3D const *XAxis):
    surface(rResult, SGM::ConeType),
    m_Origin(SGM::MidPoint(Bottom,Top)),
    m_ZAxis(SGM::UnitVector3D(Top-Bottom))
    {
    if(dBottomRadius<dTopRadius)
        {
        m_ZAxis.Negate();
        }
    m_dRadius=(dBottomRadius+dTopRadius)*0.5;

    double dHalfAngle=SGM::SAFEatan2(fabs(dTopRadius-dBottomRadius),Bottom.Distance(Top));

    m_dSinHalfAngle = sin(dHalfAngle);
    m_dCosHalfAngle = cos(dHalfAngle);

    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin = -SGM_MAX;
    m_Domain.m_VDomain.m_dMax = 1.0 / m_dSinHalfAngle;

    m_bClosedU = true;
    m_bClosedV = false;
    m_bSingularHighV = true;

    if (XAxis)
        {
        m_XAxis = *XAxis;
        }
    else
        {
        m_XAxis = m_ZAxis.Orthogonal();
        }
    m_YAxis = m_ZAxis * m_XAxis;
    }
    
cone::cone(SGM::Result &rResult, const SGMInternal::cone &other) :
    surface(rResult, other),
    m_Origin(other.m_Origin),
    m_XAxis(other.m_XAxis),
    m_YAxis(other.m_YAxis),
    m_ZAxis(other.m_ZAxis),
    m_dSinHalfAngle(other.m_dSinHalfAngle),
    m_dCosHalfAngle(other.m_dCosHalfAngle),
    m_dRadius(other.m_dRadius)
    {
    }

cone *cone::Clone(SGM::Result &rResult) const
    { 
    return new cone(rResult, *this); 
    }

double cone::FindHalfAngle() const
    {
    return SGM::SAFEacos(m_dCosHalfAngle);
    }

void cone::Evaluate(SGM::Point2D const &uv,
                    SGM::Point3D       *Pos,
                    SGM::Vector3D      *Du,
                    SGM::Vector3D      *Dv,
                    SGM::UnitVector3D  *Norm,
                    SGM::Vector3D      *Duu,
                    SGM::Vector3D      *Duv,
                    SGM::Vector3D      *Dvv) const
    {
    double dVScale=m_dRadius*(1.0-uv.m_v*m_dSinHalfAngle);
    double dCosU=cos(uv.m_u);
    double dSinU=sin(uv.m_u);
    double dZScale=uv.m_v*m_dRadius*m_dCosHalfAngle;

    if(Pos)
        {
        Pos->m_x=m_Origin.m_x+(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dVScale+m_ZAxis.m_x*dZScale;
        Pos->m_y=m_Origin.m_y+(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dVScale+m_ZAxis.m_y*dZScale;
        Pos->m_z=m_Origin.m_z+(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dVScale+m_ZAxis.m_z*dZScale;
        }
    if(Du)
        {
        Du->m_x=(m_YAxis.m_x*dCosU-m_XAxis.m_x*dSinU)*dVScale;
        Du->m_y=(m_YAxis.m_y*dCosU-m_XAxis.m_y*dSinU)*dVScale;
        Du->m_z=(m_YAxis.m_z*dCosU-m_XAxis.m_z*dSinU)*dVScale;
        }
    if(Dv)
        {
        double dDVScale=-m_dRadius*m_dSinHalfAngle;
        double dDZScale=m_dRadius*m_dCosHalfAngle;
        Dv->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dDVScale+m_ZAxis.m_x*dDZScale;
        Dv->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dDVScale+m_ZAxis.m_y*dDZScale;
        Dv->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dDVScale+m_ZAxis.m_z*dDZScale;
        }
    if(Norm)
        {
        Norm->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*m_dCosHalfAngle+m_ZAxis.m_x*m_dSinHalfAngle;
        Norm->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*m_dCosHalfAngle+m_ZAxis.m_y*m_dSinHalfAngle;
        Norm->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*m_dCosHalfAngle+m_ZAxis.m_z*m_dSinHalfAngle;
        }
    if(Duu)
        {
        Duu->m_x=(-m_YAxis.m_x*dSinU-m_XAxis.m_x*dCosU)*dVScale;
        Duu->m_y=(-m_YAxis.m_y*dSinU-m_XAxis.m_y*dCosU)*dVScale;
        Duu->m_z=(-m_YAxis.m_z*dSinU-m_XAxis.m_z*dCosU)*dVScale;
        }
    if(Duv)
        {
        double dVScaleD=-m_dRadius*m_dSinHalfAngle;
        Duv->m_x=(m_YAxis.m_x*dCosU-m_XAxis.m_x*dSinU)*dVScaleD;
        Duv->m_y=(m_YAxis.m_y*dCosU-m_XAxis.m_y*dSinU)*dVScaleD;
        Duv->m_z=(m_YAxis.m_z*dCosU-m_XAxis.m_z*dSinU)*dVScaleD;
        }
    if(Dvv)
        {
        Dvv->m_x=0;
        Dvv->m_y=0;
        Dvv->m_z=0;
        }
    }

double cone::PointInside(SGM::Point3D const &Pos) const
    {
    SGM::Point3D SurfPos;
    SGM::Point2D uv=Inverse(Pos,&SurfPos);
    double dDist=Pos.Distance(SurfPos);
    SGM::UnitVector3D Norm1,Norm2=SurfPos-Pos;
    Evaluate(uv,nullptr,nullptr,nullptr,&Norm1);
    if(Norm1%Norm2<0)
        {
        dDist=-dDist;
        }
    return dDist;
    }

cone *cone::Offset(SGM::Result &rResult,
                   double       dValue) const
    {
    // Move the orign up or down to produce the offset.

    double dHalfAngle=FindHalfAngle();
    SGM::Point3D Apex=FindApex();
    double m=Apex.Distance(m_Origin);
    double w=-dValue/m_dSinHalfAngle;
    SGM::Point3D NewOrigin=Apex-m_ZAxis*(w+m);
    return new cone(rResult,NewOrigin,m_ZAxis,m_dRadius,dHalfAngle,&m_XAxis);
    }

SGM::Point2D cone::Inverse(SGM::Point3D const &Pos,
                           SGM::Point3D       *ClosePos,
                           SGM::Point2D const *pGuess) const
    {
    double x=Pos.m_x-m_Origin.m_x;
    double y=Pos.m_y-m_Origin.m_y;
    double z=Pos.m_z-m_Origin.m_z;

    double dx=x*m_XAxis.m_x+y*m_XAxis.m_y+z*m_XAxis.m_z;
    double dy=x*m_YAxis.m_x+y*m_YAxis.m_y+z*m_YAxis.m_z;
    double dU=SGM::SAFEatan2(dy,dx);

    // Find the closest point to the parameter line at dU.

    SGM::Point3D Apex=FindApex();
    SGM::Point3D BasePos=m_Origin+m_XAxis*(cos(dU)*m_dRadius)+m_YAxis*(sin(dU)*m_dRadius);
    SGM::UnitVector3D LineVec=BasePos-Apex;
    SGM::Point3D LinePos=Apex+LineVec*((Pos-Apex)%LineVec);
    z=LinePos.m_z-m_Origin.m_z;
    double dz=x*m_ZAxis.m_x+y*m_ZAxis.m_y+z*m_ZAxis.m_z;

    double dV=dz/(m_dCosHalfAngle*m_dRadius);

    double dMaxV=1.0/m_dSinHalfAngle;
    if(dMaxV<dV)
        {
        dV=dMaxV;
        }

    if(dU<m_Domain.m_UDomain.m_dMin)
        {
        dU+=SGM_TWO_PI;
        }
    if(pGuess)
        {
        // Check for points on the axis, and on the seam.

        if(m_Domain.m_UDomain.OnBoundary(dU,SGM_ZERO))
            {
            if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false) &&
                SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                {
                dU=m_Domain.m_UDomain.m_dMax;
                }
            else if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false) &&
                     SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                {
                dU=m_Domain.m_UDomain.m_dMin;
                }
            }
        else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-m_Origin)%m_ZAxis),1.0,SGM_MIN_TOL,false))
            {
            dU=pGuess->m_u;
            }
        }

    if(ClosePos)
        {
        double dVScale=m_dRadius*(1.0-dV*m_dSinHalfAngle);
        double dZScale=dV*m_dRadius*m_dCosHalfAngle;

        double dCosU=cos(dU);
        double dSinU=sin(dU);
        ClosePos->m_x=m_Origin.m_x+(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dVScale+m_ZAxis.m_x*dZScale;
        ClosePos->m_y=m_Origin.m_y+(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dVScale+m_ZAxis.m_y*dZScale;
        ClosePos->m_z=m_Origin.m_z+(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dVScale+m_ZAxis.m_z*dZScale;
        }
    return {dU, dV};
    }

bool cone::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    bool bAnswer=true;
    auto pCone2=(cone const *)pOther;
    if(!SGM::NearEqual(m_dCosHalfAngle, pCone2->m_dCosHalfAngle, dTolerance, false))
        {
        bAnswer=false;
        }
    else if(!SGM::NearEqual(m_ZAxis % pCone2->m_ZAxis, 1.0, dTolerance, false))
        {
        bAnswer=false;
        }
    else
        {
        SGM::Point3D const &Pos1=FindApex();
        SGM::Point3D const &Pos2=pCone2->FindApex();
        if(!SGM::NearEqual(Pos1.Distance(Pos2), 0.0, dTolerance, false))
            {
            bAnswer=false;
            }
        }
    return bAnswer;
    }

void cone::PrincipleCurvature(SGM::Point2D const &uv,
                              SGM::UnitVector3D  &Vec1,
                              SGM::UnitVector3D  &Vec2,
                              double             &k1,
                              double             &k2) const
    {
    SGM::Vector3D dU,dV;
    SGM::Point3D Pos;
    Evaluate(uv,&Pos,&dU,&dV);
    double dDist=Pos.Distance(m_Origin+(m_ZAxis)*((Pos-m_Origin)%m_ZAxis));
    if(SGM_ZERO<dDist)
        {
        k1=1.0/dDist;
        }
    else
        {
        k1=SGM_MAX;
        }
    k2=0.0;
    Vec1=dU;
    Vec2=dV;
    }

void cone::Transform(SGM::Result            &,//rResult,
                     SGM::Transform3D const &Trans)
    {
    m_Origin = Trans * m_Origin;
    m_XAxis = Trans * m_XAxis;
    m_YAxis = Trans * m_YAxis;
    m_ZAxis = Trans * m_ZAxis;
    m_dRadius *= Trans.Scale();
    }

curve *cone::UParamLine(SGM::Result &rResult, double dU) const
    {
    SGM::Point3D Apex=FindApex();
    SGM::Point3D UZero;
    SGM::Point2D uv(dU,0.0);
    Evaluate(uv,&UZero);
    return new line(rResult,Apex,UZero-Apex);
    }

curve *cone::VParamLine(SGM::Result &, double) const
    { 
    return nullptr; 
    }

}