#include "EntityClasses.h"
#include "Faceter.h"
#include "Curve.h"
#include <utility>
namespace SGMInternal
{
edge::edge(SGM::Result &rResult):
    topology(rResult,SGM::EntityType::EdgeType),
    m_pStart(nullptr),m_pEnd(nullptr),m_pVolume(nullptr),m_pCurve(nullptr)
    {
    m_dTolerance=SGM_MIN_TOL;
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
        SGM::Interval1D const &CurveDomain=m_pCurve->GetDomain();
        m_Domain.m_dMin=m_pCurve->Inverse(m_pStart->GetPoint());
        m_Domain.m_dMax=m_pCurve->Inverse(m_pEnd->GetPoint());
        if(m_Domain.Empty())
            {
            if(m_pCurve->GetClosed())
                {
                if(SGM::NearEqual(m_Domain.m_dMax,CurveDomain.m_dMin,SGM_MIN_TOL,false))
                    {
                    m_Domain.m_dMax=CurveDomain.m_dMax;
                    }
                else if(SGM::NearEqual(m_Domain.m_dMin,CurveDomain.m_dMax,SGM_MIN_TOL,false))
                    {
                    m_Domain.m_dMin=CurveDomain.m_dMin;
                    }
                }
            if(m_Domain.Empty())
                {
                throw;
                }
            }
        if( m_Domain.Length()<SGM_ZERO && 
            m_pStart==m_pEnd && 
            m_pCurve->GetCurveType()!=SGM::PointCurveType)
            {
            m_Domain=m_pCurve->GetDomain();
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

std::vector<SGM::Point3D> const &edge::GetFacets(SGM::Result &rResult) const
    {
    if(m_aPoints3D.empty())
        {
        FacetOptions Options;
        FacetEdge(rResult,this,Options,m_aPoints3D,m_aParams);
        }
    return m_aPoints3D;
    }

std::vector<double> const &edge::GetParams(SGM::Result &rResult) const
    {
    if(m_aPoints3D.empty())
        {
        FacetOptions Options;
        FacetEdge(rResult,this,Options,m_aPoints3D,m_aParams);
        }
    return m_aParams;
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

double edge::FindLength(double dTolerance) const
    {
    return m_pCurve->FindLength(m_Domain,dTolerance);
    }
}