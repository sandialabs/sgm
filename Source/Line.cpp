#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "Curve.h"

namespace SGMInternal
{
line::line(SGM::Result        &rResult,
           SGM::Point3D const &Pos0,
           SGM::Point3D const &Pos1):
    curve(rResult,SGM::LineType),m_Origin(Pos0),
    m_Axis(Pos1-Pos0)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

line::line(SGM::Result &rResult,
           line const  *pLine):
    curve(rResult,SGM::LineType),m_Origin(pLine->m_Origin),
    m_Axis(pLine->m_Axis)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

line::line(SGM::Result             &rResult,
           SGM::Point3D      const &Origin,
           SGM::UnitVector3D const &Axis):
    curve(rResult,SGM::LineType),m_Origin(Origin),
    m_Axis(Axis)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }
}