#include "SGMEnums.h"
#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"

namespace SGMInternal
{
plane::plane(SGM::Result             &rResult,
             SGM::Point3D      const &Origin,
             SGM::UnitVector3D const &XAxis,
             SGM::UnitVector3D const &YAxis,
             SGM::UnitVector3D const &ZAxis) :
        surface(rResult,SGM::PlaneType),
        m_Origin(Origin),
        m_XAxis(XAxis),
        m_YAxis(YAxis),
        m_ZAxis(ZAxis)
    {
    m_Domain.m_UDomain.m_dMin=-SGM_MAX;
    m_Domain.m_UDomain.m_dMax=SGM_MAX;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=SGM_MAX;
    m_bClosedU=false;
    m_bClosedV=false;
    }

plane::plane(SGM::Result        &rResult,
             SGM::Point3D const &Origin,
             SGM::Point3D const &XPos,
             SGM::Point3D const &YPos) :
        surface(rResult,SGM::PlaneType),
        m_Origin(Origin),
        m_XAxis(XPos-Origin),
        m_YAxis(YPos-Origin),
        m_ZAxis(m_XAxis*m_YAxis)
    {
    m_Domain.m_UDomain.m_dMin=-SGM_MAX;
    m_Domain.m_UDomain.m_dMax=SGM_MAX;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=SGM_MAX;
    m_bClosedU=false;
    m_bClosedV=false;
    }

plane::plane(SGM::Result &rResult, plane const &other):
        surface(rResult,other),
        m_Origin(other.m_Origin),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_ZAxis(other.m_ZAxis)
    {}

plane *plane::Clone(SGM::Result &rResult) const
    {
    return new plane(rResult, *this);
    }

void plane::Evaluate(SGM::Point2D const &uv,
                     SGM::Point3D       *Pos,
                     SGM::Vector3D      *Du,
                     SGM::Vector3D      *Dv,
                     SGM::UnitVector3D  *Norm,
                     SGM::Vector3D      *Duu,
                     SGM::Vector3D      *Duv,
                     SGM::Vector3D      *Dvv) const
    {
    if(Pos)
        {
        Pos->m_x=m_Origin.m_x+(m_XAxis.m_x*uv.m_u+m_YAxis.m_x*uv.m_v);
        Pos->m_y=m_Origin.m_y+(m_XAxis.m_y*uv.m_u+m_YAxis.m_y*uv.m_v);
        Pos->m_z=m_Origin.m_z+(m_XAxis.m_z*uv.m_u+m_YAxis.m_z*uv.m_v);
        }
    if(Du)
        {
        Du->m_x=m_XAxis.m_x;
        Du->m_y=m_XAxis.m_y;
        Du->m_z=m_XAxis.m_z;
        }
    if(Dv)
        {
        Dv->m_x=m_YAxis.m_x;
        Dv->m_y=m_YAxis.m_y;
        Dv->m_z=m_YAxis.m_z;
        }
    if(Norm)
        {
        *Norm=m_ZAxis;
        }
    if(Duu)
        {
        Duu->m_x=0;
        Duu->m_y=0;
        Duu->m_z=0;
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

SGM::Point2D plane::Inverse(SGM::Point3D const &Pos,
                            SGM::Point3D       *ClosePos,
                            SGM::Point2D const *) const
    {
        double dx=Pos.m_x-m_Origin.m_x;
        double dy=Pos.m_y-m_Origin.m_y;
        double dz=Pos.m_z-m_Origin.m_z;
        double dU=(dx*m_XAxis.m_x+dy*m_XAxis.m_y+dz*m_XAxis.m_z);
        double dV=(dx*m_YAxis.m_x+dy*m_YAxis.m_y+dz*m_YAxis.m_z);

        if(ClosePos)
            {
            ClosePos->m_x=m_Origin.m_x+(m_XAxis.m_x*dU+m_YAxis.m_x*dV);
            ClosePos->m_y=m_Origin.m_y+(m_XAxis.m_y*dU+m_YAxis.m_y*dV);
            ClosePos->m_z=m_Origin.m_z+(m_XAxis.m_z*dU+m_YAxis.m_z*dV);
            }
        return {dU,dV};
    }

void plane::PrincipleCurvature(SGM::Point2D const   &,
                                 SGM::UnitVector3D  &Vec1,
                                 SGM::UnitVector3D  &Vec2,
                                 double             &k1,
                                 double             &k2) const
    {
    Vec1=m_XAxis;
    Vec2=m_YAxis;
    k1=0;
    k2=0;
    }

void plane::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    }

curve *plane::UParamLine(SGM::Result &, double) const
    { return nullptr; } // no curve

curve *plane::VParamLine(SGM::Result &, double) const
    { return nullptr; }


}
