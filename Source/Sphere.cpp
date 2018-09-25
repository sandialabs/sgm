#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "Curve.h"
#include "EntityClasses.h"
#include "Surface.h"

namespace SGMInternal
{
sphere::sphere(SGM::Result             &rResult,
               SGM::Point3D      const &Center,
               double                   dRadius,
               SGM::UnitVector3D const *XAxis,
               SGM::UnitVector3D const *YAxis):
    surface(rResult,SGM::SphereType),m_Center(Center),m_dRadius(dRadius)
    {
    m_bClosedU=true;
    m_bSingularHighV=true;
    m_bSingularLowV=true;
    m_Domain.m_UDomain.m_dMin=0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin=-SGM_HALF_PI;
    m_Domain.m_VDomain.m_dMax=SGM_HALF_PI;

    if(XAxis && YAxis)
        {
        m_XAxis=*XAxis;
        m_YAxis=*YAxis;
        m_ZAxis=m_XAxis*m_YAxis;
        }
    else
        {
        m_XAxis=SGM::UnitVector3D(1.0,0.0,0.0);
        m_YAxis=SGM::UnitVector3D(0.0,1.0,0.0);
        m_ZAxis=SGM::UnitVector3D(0.0,0.0,1.0);
        }
    }

sphere::sphere(SGM::Result &rResult, sphere const &other) :
    surface(rResult, other),
    m_Center(other.m_Center),
    m_XAxis(other.m_XAxis),
    m_YAxis(other.m_YAxis),
    m_ZAxis(other.m_ZAxis),
    m_dRadius(other.m_dRadius)
{}

sphere *sphere::Clone(SGM::Result &rResult) const
{ return new sphere(rResult, *this); }

void sphere::Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du,
                      SGM::Vector3D      *Dv,
                      SGM::UnitVector3D  *Norm,
                      SGM::Vector3D      *Duu,
                      SGM::Vector3D      *Duv,
                      SGM::Vector3D      *Dvv) const
    {
    double dCosU=cos(uv.m_u);
    double dSinU=sin(uv.m_u);
    double dCosV=cos(uv.m_v);
    double dSinV=sin(uv.m_v);

    if(Pos)
        {
        Pos->m_x=m_Center.m_x+((m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dCosV+m_ZAxis.m_x*dSinV)*m_dRadius;
        Pos->m_y=m_Center.m_y+((m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dCosV+m_ZAxis.m_y*dSinV)*m_dRadius;
        Pos->m_z=m_Center.m_z+((m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dCosV+m_ZAxis.m_z*dSinV)*m_dRadius;
        }
    if(Du)
        {
        Du->m_x=(m_YAxis.m_x*dCosU-m_XAxis.m_x*dSinU)*dCosV*m_dRadius;
        Du->m_y=(m_YAxis.m_y*dCosU-m_XAxis.m_y*dSinU)*dCosV*m_dRadius;
        Du->m_z=(m_YAxis.m_z*dCosU-m_XAxis.m_z*dSinU)*dCosV*m_dRadius;
        }
    if(Dv)
        {
        Dv->m_x=(m_ZAxis.m_x*dCosV-(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dSinV)*m_dRadius;
        Dv->m_y=(m_ZAxis.m_y*dCosV-(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dSinV)*m_dRadius;
        Dv->m_z=(m_ZAxis.m_z*dCosV-(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dSinV)*m_dRadius;
        }
    if(Norm)
        {
        Norm->m_x=(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dCosV+m_ZAxis.m_x*dSinV;
        Norm->m_y=(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dCosV+m_ZAxis.m_y*dSinV;
        Norm->m_z=(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dCosV+m_ZAxis.m_z*dSinV;
        }
    if(Duu)
        {
        Duu->m_x=(-m_YAxis.m_x*dSinU-m_XAxis.m_x*dCosU)*dCosV*m_dRadius;
        Duu->m_y=(-m_YAxis.m_y*dSinU-m_XAxis.m_y*dCosU)*dCosV*m_dRadius;
        Duu->m_z=(-m_YAxis.m_z*dSinU-m_XAxis.m_z*dCosU)*dCosV*m_dRadius;
        }
    if(Duv)
        {
        Duv->m_x=(m_XAxis.m_x*dSinU-m_YAxis.m_x*dCosU)*dSinV*m_dRadius;
        Duv->m_y=(m_XAxis.m_y*dSinU-m_YAxis.m_y*dCosU)*dSinV*m_dRadius;
        Duv->m_z=(m_XAxis.m_z*dSinU-m_YAxis.m_z*dCosU)*dSinV*m_dRadius;
        }
    if(Dvv)
        {
        Dvv->m_x=(-m_ZAxis.m_x*dSinV-(m_XAxis.m_x*dCosU+m_YAxis.m_x*dSinU)*dCosV)*m_dRadius;
        Dvv->m_y=(-m_ZAxis.m_y*dSinV-(m_XAxis.m_y*dCosU+m_YAxis.m_y*dSinU)*dCosV)*m_dRadius;
        Dvv->m_z=(-m_ZAxis.m_z*dSinV-(m_XAxis.m_z*dCosU+m_YAxis.m_z*dSinU)*dCosV)*m_dRadius;
        }
    }

SGM::Point2D sphere::Inverse(SGM::Point3D const &Pos,
                              SGM::Point3D       *ClosePos,
                              SGM::Point2D const *pGuess) const
    {
    double x=Pos.m_x-m_Center.m_x;
    double y=Pos.m_y-m_Center.m_y;
    double z=Pos.m_z-m_Center.m_z;

    double dUx=x*m_XAxis.m_x+y*m_XAxis.m_y+z*m_XAxis.m_z;
    double dUy=x*m_YAxis.m_x+y*m_YAxis.m_y+z*m_YAxis.m_z;

    double dU=SGM::SAFEatan2(dUy,dUx);
    double dV=0.0;
    SGM::Vector3D VSpoke=(Pos-m_ZAxis*((Pos-m_Center)%m_ZAxis))-m_Center;
    if(VSpoke.Magnitude()<SGM_ZERO)
        {
        if((Pos-m_Center)%m_ZAxis<0)
            {
            // South Pole
            dV=-SGM_HALF_PI;
            }
        else
            {
            // North Pole
            dV=SGM_HALF_PI;
            }
        }
    else
        {
        SGM::UnitVector3D Spoke=VSpoke;
        double dVx=x*Spoke.m_x+y*Spoke.m_y+z*Spoke.m_z;
        double dVy=x*m_ZAxis.m_x+y*m_ZAxis.m_y+z*m_ZAxis.m_z;
        dV=SGM::SAFEatan2(dVy,dVx);
        }

    if(dU<m_Domain.m_UDomain.m_dMin)
        {
        dU+=SGM_TWO_PI;
        }
    if(pGuess)
        {
        // Check for points on the north-south pole axis, at the
        // center, and on the seam.

        if(SGM::NearEqual(Pos,m_Center,SGM_MIN_TOL))
            {
            dU=pGuess->m_u;
            dV=pGuess->m_v;
            }
        else if(m_Domain.m_UDomain.OnBoundary(dU,SGM_MIN_TOL))
            {
            if(m_Domain.m_UDomain.MidPoint()<pGuess->m_u)
                {
                dU=m_Domain.m_UDomain.m_dMax;
                }
            else 
                {
                dU=m_Domain.m_UDomain.m_dMin;
                }
            }
        else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-m_Center)%m_ZAxis),1.0,SGM_MIN_TOL,false))
            {
            dU=pGuess->m_u;
            }
        }

    if(ClosePos)
        {
        SGM::UnitVector3D Ray=Pos-m_Center;
        *ClosePos=m_Center+Ray*m_dRadius;
        }
    return {dU, dV};
    }

bool sphere::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    bool bAnswer=true;
    sphere const *pSphere2=(sphere const *)pOther;
    if(SGM::NearEqual(m_dRadius,pSphere2->m_dRadius,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else if(SGM::NearEqual(m_Center,pSphere2->m_Center,dTolerance)==false)
        {
        bAnswer=false;
        }
    return bAnswer;
    }

void sphere::PrincipleCurvature(SGM::Point2D const &uv,
                                SGM::UnitVector3D  &Vec1,
                                SGM::UnitVector3D  &Vec2,
                                double             &k1,
                                double             &k2) const
    {
    k1=1.0/m_dRadius;
    k2=1.0/m_dRadius;
    SGM::Vector3D dU,dV;
    Evaluate(uv,nullptr,&dU,&dV);
    Vec1=dU;
    Vec2=dV;
    }

void sphere::Transform(SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    m_dRadius*=Trans.Scale();
    }

curve* sphere::UParamLine(SGM::Result &rResult, double dU) const
    {
    SGM::Point2D uv(dU,0);
    SGM::Point3D Pos;
    Evaluate(uv,&Pos);
    SGM::Point3D const &Center=m_Center;
    SGM::UnitVector3D const &ZAxis=m_ZAxis;
    double dRadius=m_dRadius;
    SGM::UnitVector3D XAxis=Pos-Center;
    SGM::UnitVector3D Normal=XAxis*ZAxis;
    SGM::Interval2D const &Domain=GetDomain();
    return new circle(rResult,Center,Normal,dRadius,&XAxis,&Domain.m_VDomain);
    }

curve *sphere::VParamLine(SGM::Result &, double) const
    { return nullptr; }


}
