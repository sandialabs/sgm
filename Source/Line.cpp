#include "SGMVector.h"
#include "SGMTransform.h"
#include "Curve.h"

namespace SGMInternal
{
line::line(SGM::Result        &rResult,
           SGM::Point3D const &Pos0,
           SGM::Point3D const &Pos1):
    curve(rResult,SGM::LineType),m_Origin(Pos0),
    m_Axis(Pos1-Pos0),m_dScale(1.0)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

line::line(SGM::Result             &rResult,
           SGM::Point3D      const &Origin,
           SGM::UnitVector3D const &Axis,
           double                   dScale):
    curve(rResult,SGM::LineType),m_Origin(Origin),
    m_Axis(Axis),m_dScale(dScale)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

line::line(SGM::Result &rResult, line const &other):
        curve(rResult, other),
        m_Origin(other.m_Origin),
        m_Axis(other.m_Axis),
        m_dScale(other.m_dScale)
    {}

line *line::Clone(SGM::Result &rResult) const
    { return new line(rResult, *this); }

void line::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    if(Pos)
        {
        Pos->m_x=m_Origin.m_x+m_Axis.m_x*t*m_dScale;
        Pos->m_y=m_Origin.m_y+m_Axis.m_y*t*m_dScale;
        Pos->m_z=m_Origin.m_z+m_Axis.m_z*t*m_dScale;
        }
    if(D1)
        {
        D1->m_x=m_Axis.m_x*m_dScale;
        D1->m_y=m_Axis.m_y*m_dScale;
        D1->m_z=m_Axis.m_z*m_dScale;
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
    double dScale=GetScale();
    double t=((Pos.m_x-Origin.m_x)*Axis.m_x+(Pos.m_y-Origin.m_y)*Axis.m_y+(Pos.m_z-Origin.m_z)*Axis.m_z)/dScale;

    if(ClosePos)
        {
        ClosePos->m_x=Origin.m_x+Axis.m_x*t*dScale;
        ClosePos->m_y=Origin.m_y+Axis.m_y*t*dScale;
        ClosePos->m_z=Origin.m_z+Axis.m_z*t*dScale;
        }
    return t;
    }

void line::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_Axis=Trans*m_Axis;
    m_dScale*=Trans.Scale(m_Axis);
    }


}