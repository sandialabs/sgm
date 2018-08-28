#include "SGMVector.h"
#include "SGMTransform.h"
#include "Curve.h"

namespace SGMInternal
{
line::line(SGM::Result        &rResult,
           SGM::Point3D const &Pos0,
           SGM::Point3D const &Pos1):
        curve(rResult,SGM::LineType),
        m_Origin(Pos0),
        m_Axis(Pos1-Pos0)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

line::line(SGM::Result             &rResult,
           SGM::Point3D      const &Origin,
           SGM::UnitVector3D const &Axis):
        curve(rResult,SGM::LineType),
        m_Origin(Origin),
        m_Axis(Axis)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

bool line::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=m_CurveType)
        {
        return false;
        }
    line const *pLine2=(line const *)pOther;
    if(SGM::NearEqual(m_Origin,pLine2->m_Origin,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_Axis,pLine2->m_Axis,dTolerance)==false)
        {
        return false;
        }
    return true;
    }

line::line(SGM::Result &rResult, line const &other):
        curve(rResult, other),
        m_Origin(other.m_Origin),
        m_Axis(other.m_Axis)
    {}

line *line::Clone(SGM::Result &rResult) const
    { return new line(rResult, *this); }

void line::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    if(Pos)
        {
        Pos->m_x=m_Origin.m_x+m_Axis.m_x*t;
        Pos->m_y=m_Origin.m_y+m_Axis.m_y*t;
        Pos->m_z=m_Origin.m_z+m_Axis.m_z*t;
        }
    if(D1)
        {
        D1->m_x=m_Axis.m_x;
        D1->m_y=m_Axis.m_y;
        D1->m_z=m_Axis.m_z;
        }
    if(D2)
        {
        D2->m_x=0;
        D2->m_y=0;
        D2->m_z=0;
        }
    }

double line::Inverse(SGM::Point3D const  &Pos,
                      SGM::Point3D       *ClosePos,
                      double       const *) const
    {
    SGM::Point3D const &Origin=GetOrigin();
    SGM::UnitVector3D const &Axis=GetAxis();
    double t=((Pos.m_x-Origin.m_x)*Axis.m_x+(Pos.m_y-Origin.m_y)*Axis.m_y+(Pos.m_z-Origin.m_z)*Axis.m_z);

    if(ClosePos)
        {
        ClosePos->m_x=Origin.m_x+Axis.m_x*t;
        ClosePos->m_y=Origin.m_y+Axis.m_y*t;
        ClosePos->m_z=Origin.m_z+Axis.m_z*t;
        }
    return t;
    }

void line::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_Axis=Trans*m_Axis;
    }

double line::FindLength(SGM::Interval1D const &Domain,double ) const
    {
    return Domain.Length();
    }

}
