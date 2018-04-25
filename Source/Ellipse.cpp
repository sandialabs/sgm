#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "SGMEnums.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Topology.h"

ellipse::ellipse(SGM::Result             &rResult,
                 SGM::Point3D      const &Center,
                 SGM::UnitVector3D const &XAxis,
                 SGM::UnitVector3D const &YAxis,
                 double                   dA,
                 double                   dB):  
    curve(rResult,SGM::EllipseType),m_Center(Center),m_XAxis(XAxis),m_YAxis(YAxis),m_dA(dA),m_dB(dB)
    {
    m_Domain.m_dMin=0;
    m_Domain.m_dMax=SGM_TWO_PI;
    m_bClosed=true;
    }

