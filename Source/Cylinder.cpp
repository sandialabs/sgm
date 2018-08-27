#include "SGMTransform.h"

#include "Curve.h"
#include "Mathematics.h"
#include "Surface.h"

namespace SGMInternal 
{

cylinder::cylinder(SGM::Result             &rResult,
                   SGM::Point3D      const &Origin,
                   SGM::UnitVector3D const &Axis,
                   double                   dRadius,
                   SGM::UnitVector3D const *XAxis) :
    surface(rResult, SGM::CylinderType), m_Origin(Origin)
    {
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin = -SGM_MAX;
    m_Domain.m_VDomain.m_dMax = SGM_MAX;
    m_bClosedU = true;
    m_bClosedV = false;

    m_dRadius = dRadius;
    m_ZAxis = Axis;
    if (XAxis)
        {
        m_XAxis=*XAxis;
        }
    else
        {
        m_XAxis=m_ZAxis.Orthogonal();
        }
    m_YAxis=m_ZAxis*m_XAxis;
    }

cylinder::cylinder(SGM::Result             &rResult,
                   SGM::Point3D      const &Bottom,
                   SGM::Point3D      const &Top,
                   double                   dRadius,
                   SGM::UnitVector3D const *XAxis) :
    surface(rResult, SGM::CylinderType), m_Origin(SGM::MidPoint(Bottom,Top))
    {
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin = -SGM_MAX;
    m_Domain.m_VDomain.m_dMax = SGM_MAX;
    m_bClosedU = true;
    m_bClosedV = false;

    m_dRadius = dRadius;
    m_ZAxis = Top - Bottom;
    if(XAxis)
        {
        m_XAxis=Snap(*XAxis);
        }
    else
        {
        m_XAxis=m_ZAxis.Orthogonal();
        }
    m_YAxis=m_ZAxis*m_XAxis;
    }

void cylinder::Evaluate(SGM::Point2D const &uv,
                        SGM::Point3D       *Pos,
                        SGM::Vector3D      *Du,
                        SGM::Vector3D      *Dv,
                        SGM::UnitVector3D  *Norm,
                        SGM::Vector3D      *Duu,
                        SGM::Vector3D      *Duv,
                        SGM::Vector3D      *Dvv) const
    {
    double dCos=cos(uv.m_u);
    double dSin=sin(uv.m_u);

    if(Pos)
        {
        Pos->m_x=m_Origin.m_x+(m_XAxis.m_x*dCos+m_YAxis.m_x*dSin+m_ZAxis.m_x*uv.m_v)*m_dRadius;
        Pos->m_y=m_Origin.m_y+(m_XAxis.m_y*dCos+m_YAxis.m_y*dSin+m_ZAxis.m_y*uv.m_v)*m_dRadius;
        Pos->m_z=m_Origin.m_z+(m_XAxis.m_z*dCos+m_YAxis.m_z*dSin+m_ZAxis.m_z*uv.m_v)*m_dRadius;
        }
    if(Du)
        {
        Du->m_x=(m_YAxis.m_x*dCos-m_XAxis.m_x*dSin)*m_dRadius;
        Du->m_y=(m_YAxis.m_y*dCos-m_XAxis.m_y*dSin)*m_dRadius;
        Du->m_z=(m_YAxis.m_z*dCos-m_XAxis.m_z*dSin)*m_dRadius;
        }
    if(Dv)
        {
        Dv->m_x=m_ZAxis.m_x*m_dRadius;
        Dv->m_y=m_ZAxis.m_y*m_dRadius;
        Dv->m_z=m_ZAxis.m_z*m_dRadius;
        }
    if(Norm)
        {
        Norm->m_x=m_XAxis.m_x*dCos+m_YAxis.m_x*dSin;
        Norm->m_y=m_XAxis.m_y*dCos+m_YAxis.m_y*dSin;
        Norm->m_z=m_XAxis.m_z*dCos+m_YAxis.m_z*dSin;
        }
    if(Duu)
        {
        Duu->m_x=(-m_XAxis.m_x*dCos-m_YAxis.m_x*dSin)*m_dRadius;
        Duu->m_y=(-m_XAxis.m_y*dCos-m_YAxis.m_y*dSin)*m_dRadius;
        Duu->m_z=(-m_XAxis.m_z*dCos-m_YAxis.m_z*dSin)*m_dRadius;
        }
    if(Duv)
        {
        Duv->m_x=0;
        Duv->m_y=0;
        Duv->m_z=0;
        }
    if(Dvv)
        {
        Dvv->m_x=0;
        Dvv->m_y=0;
        Dvv->m_z=0;
        }
    }

SGM::Point2D cylinder::Inverse(SGM::Point3D const &Pos,
                               SGM::Point3D       *ClosePos,
                               SGM::Point2D const *pGuess) const
    {
    double x=Pos.m_x-m_Origin.m_x;
    double y=Pos.m_y-m_Origin.m_y;
    double z=Pos.m_z-m_Origin.m_z;

    double dx=x*m_XAxis.m_x+y*m_XAxis.m_y+z*m_XAxis.m_z;
    double dy=x*m_YAxis.m_x+y*m_YAxis.m_y+z*m_YAxis.m_z;
    double dU=SGM::SAFEatan2(dy,dx);
    double dV=(x*m_ZAxis.m_x+y*m_ZAxis.m_y+z*m_ZAxis.m_z)/m_dRadius;

    while(dU<m_Domain.m_UDomain.m_dMin)
        {
        dU+=SGM_TWO_PI;
        }
    while(m_Domain.m_UDomain.m_dMax<dU)
        {
        dU-=SGM_TWO_PI;
        }

    if(pGuess)
        {
        // Check for points on the axis, and on the seam.

        if(!m_Domain.m_UDomain.InInterval(dU, SGM_ZERO))
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
        else if(SGM::NearEqual(std::abs(SGM::UnitVector3D(Pos-m_Origin)%m_ZAxis),1.0,SGM_MIN_TOL,false))
            {
            dU=pGuess->m_u;
            }
        }

    if(ClosePos)
        {
        double dCos=cos(dU);
        double dSin=sin(dU);
        ClosePos->m_x=m_Origin.m_x+(m_XAxis.m_x*dCos+m_YAxis.m_x*dSin+m_ZAxis.m_x*dV)*m_dRadius;
        ClosePos->m_y=m_Origin.m_y+(m_XAxis.m_y*dCos+m_YAxis.m_y*dSin+m_ZAxis.m_y*dV)*m_dRadius;
        ClosePos->m_z=m_Origin.m_z+(m_XAxis.m_z*dCos+m_YAxis.m_z*dSin+m_ZAxis.m_z*dV)*m_dRadius;
        }
    return {dU, dV};
    }

bool cylinder::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }

    bool bAnswer=true;
    cylinder const *pCylinder1=(cylinder const *)this;
    cylinder const *pCylinder2=(cylinder const *)pOther;
    if(SGM::NearEqual(pCylinder1->m_dRadius,pCylinder2->m_dRadius,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else if(SGM::NearEqual(fabs(pCylinder1->m_ZAxis%pCylinder2->m_ZAxis),1.0,dTolerance,false)==false)
        {
        bAnswer=false;
        }
    else
        {
        SGM::Point3D const &Pos1=pCylinder1->m_Origin;
        SGM::Point3D const &Pos2=pCylinder2->m_Origin;
        SGM::UnitVector3D const &Axis1=pCylinder1->m_ZAxis;
        if(SGM::NearEqual(Pos2.Distance(Pos1+Axis1*((Pos2-Pos1)%Axis1)),0.0,dTolerance,false)==false)
            {
            bAnswer=false;
            }
        }
    return bAnswer;
    }

void cylinder::PrincipleCurvature(SGM::Point2D const &uv,
                                 SGM::UnitVector3D  &Vec1,
                                 SGM::UnitVector3D  &Vec2,
                                 double             &k1,
                                 double             &k2) const
    {
    k1=1.0/m_dRadius;
    k2=0.0;
    SGM::Vector3D dU,dV;
    Evaluate(uv,nullptr,&dU,&dV);
    Vec1=dU;
    Vec2=dV;
    }

void cylinder::Transform(SGM::Transform3D const &Trans)
    {
        m_Origin = Trans * m_Origin;
        m_XAxis = Trans * m_XAxis;
        m_YAxis = Trans * m_YAxis;
        m_ZAxis = Trans * m_ZAxis;
        m_dRadius *= Trans.Scale();
    }

curve *cylinder::UParamLine(SGM::Result &rResult, double dU) const
    {
    SGM::Point2D uv(dU,0);
    SGM::Point3D Pos;
    SGM::Vector3D Vec;
    Evaluate(uv,&Pos,nullptr,&Vec);
    return new line(rResult,Pos,Vec);
    }

curve *cylinder::VParamLine(SGM::Result &, double) const
    { 
    return nullptr; 
    }
}