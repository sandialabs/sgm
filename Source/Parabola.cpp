#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Topology.h"

parabola::parabola(SGM::Result             &rResult,
                   SGM::Point3D      const &Center,
                   SGM::UnitVector3D const &XAxis,
                   SGM::UnitVector3D const &YAxis,
                   double                   dA):  
    curve(rResult,SGM::ParabolaType),m_Center(Center),m_XAxis(XAxis),m_YAxis(YAxis),
    m_Normal(XAxis*YAxis),m_dA(dA)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }
