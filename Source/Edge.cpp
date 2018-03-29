#include "EntityClasses.h"
#include "Faceter.h"

edge::edge(SGM::Result &rResult):
    topology(rResult,SGM::EntityType::EdgeType),m_pVolume(NULL),
    m_pStart(NULL),m_pEnd(NULL),m_pCurve(NULL)
    {
    }

void edge::SetStart(vertex *pStart) 
    {
    if(m_pStart)
        {
        m_pStart->RemoveEdge(this);
        }
    m_pStart=pStart;
    if(pStart)
        {
        pStart->AddEdge(this);
        }
    }

void edge::SetEnd(vertex *pEnd) 
    {
    if(m_pEnd)
        {
        m_pEnd->RemoveEdge(this);
        }
    m_pEnd=pEnd;
    if(pEnd)
        {
        pEnd->AddEdge(this);
        }
    }

SGM::Interval1D const &edge::GetDomain() const 
    {
    if(m_Domain.Empty())
        {
        m_Domain.m_dMin=m_pCurve->Inverse(m_pStart->GetPoint());
        m_Domain.m_dMax=m_pCurve->Inverse(m_pEnd->GetPoint());
        if(m_Domain.Empty())
            {
            std::swap((vertex *)m_pStart,(vertex *)m_pEnd);
            std::swap(m_Domain.m_dMin,m_Domain.m_dMax);
            }
        }
    return m_Domain;
    }

void edge::SetCurve(curve *pCurve)
    {
    if(m_pCurve)
        {
        m_pCurve->RemoveEdge(this);
        }
    m_pCurve=pCurve;
    m_pCurve->AddEdge(this);
    }

std::vector<SGM::Point3D> const &edge::GetFacets() const
    {
    if(m_aPoints3D.empty())
        {
        FacetOptions Options;
        FacetEdge(this,Options,m_aPoints3D);
        }
    return m_aPoints3D;
    }

SGM::Point3D const &edge::FindStartPoint() const
    {
    return m_pStart->GetPoint();
    }

SGM::Point3D const &edge::FindEndPoint() const
    {
    return m_pEnd->GetPoint();
    }

SGM::Point3D edge::FindMidPoint(double dFraction) const
    {
    SGM::Point3D Pos;
    double t=m_Domain.MidPoint(dFraction);
    m_pCurve->Evaluate(t,&Pos);
    return Pos;
    }