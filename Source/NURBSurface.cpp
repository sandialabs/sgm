#include "SGMVector.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include "Faceter.h"

#include "Primitive.h"

namespace SGMInternal
{
NURBsurface::NURBsurface(SGM::Result                                   &rResult,
                         std::vector<std::vector<SGM::Point4D> > const &aControlPoints,
                         std::vector<double>                     const &aUKnots,
                         std::vector<double>                     const &aVKnots):
    surface(rResult,SGM::NURBSurfaceType),m_aaControlPoints(aControlPoints),m_aUKnots(aUKnots),m_aVKnots(aVKnots)
    {
    m_Domain.m_UDomain.m_dMin=aUKnots.front();
    m_Domain.m_UDomain.m_dMax=aUKnots.back();
    m_Domain.m_VDomain.m_dMin=aVKnots.front();
    m_Domain.m_VDomain.m_dMax=aVKnots.back();

    SGM::Point3D Pos0,Pos1,Pos2,Pos3;
    Evaluate(m_Domain.MidPoint(0.5,0),&Pos0);
    Evaluate(m_Domain.MidPoint(0.5,1),&Pos1);
    Evaluate(m_Domain.MidPoint(0,0.5),&Pos2);
    Evaluate(m_Domain.MidPoint(1,0.5),&Pos3);
    if(SGM::NearEqual(Pos0,Pos1,SGM_MIN_TOL))
        {
        m_bClosedV=true;
        }
    if(SGM::NearEqual(Pos2,Pos3,SGM_MIN_TOL))
        {
        m_bClosedU=true;
        }

    curve *pUCurve=UParamLine(rResult,m_Domain.m_UDomain.MidPoint(0.25));
    curve *pVCurve=VParamLine(rResult,m_Domain.m_VDomain.MidPoint(0.25));
    FacetOptions Options;
    Options.m_dEdgeAngleTol=0.17453292519943295769236907684886; // 10 degrees.
    std::vector<double> aUParams,aVParams;
    std::vector<SGM::Point3D> aUPoints,aVPoints;
    FacetCurve(pUCurve,pUCurve->GetDomain(),Options,aUPoints,aVParams);
    FacetCurve(pVCurve,pVCurve->GetDomain(),Options,aVPoints,aUParams);
    rResult.GetThing()->DeleteEntity(pUCurve);
    rResult.GetThing()->DeleteEntity(pVCurve);

    size_t Index1,Index2;
    m_nUParams=aUParams.size();
    m_nVParams=aVParams.size();
    size_t nParams=m_nUParams*m_nVParams;
    m_aSeedParams.reserve(nParams);
    m_aSeedPoints.reserve(nParams);
    for(Index1=0;Index1<m_nUParams;++Index1)
        {
        double u=aUParams[Index1];
        for(Index2=0;Index2<m_nVParams;++Index2)
            {
            double v=aVParams[Index2];
            SGM::Point2D uv(u,v);
            m_aSeedParams.push_back(uv);
            SGM::Point3D Pos;
            Evaluate(uv,&Pos);
            m_aSeedPoints.push_back(Pos);
            }
        }
    }

void NURBsurface::Transform(SGM::Transform3D const &Trans)
    {
    size_t nSize1 = m_aaControlPoints.size();
    size_t nSize2 = m_aaControlPoints[0].size();
    for(size_t Index1=0;Index1<nSize1;++Index1)
        {
        for(size_t Index2=0;Index2<nSize2;++Index2)
            {
            SGM::Point4D &pos4D = m_aaControlPoints[Index1][Index2];
            SGM::Point3D pos3D = {pos4D.m_x,pos4D.m_y,pos4D.m_z};
            pos3D = Trans * pos3D;
            m_aaControlPoints[Index1][Index2] = {pos3D.m_x,pos3D.m_y,pos3D.m_z,pos4D.m_w};
            }
        }
    }

curve *NURBsurface::UParamLine(SGM::Result &rResult, double dU) const
    {
    if(m_bSingularHighU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
        {
        SGM::Point3D Pos;
        Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
        return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
        }
    else if(m_bSingularLowU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
        {
        SGM::Point3D Pos;
        Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
        return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
        }
    else
        {
        std::vector<double> const &aUKnots=GetUKnots();
        std::vector<std::vector<SGM::Point4D> > const &aaControlPoints=GetControlPoints();
        size_t nUDegree=GetUDegree();
        size_t nSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,dU,aUKnots);

        double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
        double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
        size_t Index1,Index2;
        for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
            {
            aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
            }
        FindBasisFunctions(nSpanIndex,dU,nUDegree,0,&aUKnots[0],aaBasisFunctions);

        std::vector<SGM::Point4D> aControlPoints;
        size_t nControlPoints=aaControlPoints[0].size();
        aControlPoints.assign(nControlPoints,SGM::Point4D(0.0,0.0,0.0,0.0));
        for(Index1=0;Index1<nControlPoints;++Index1)
            {
            for(Index2=0;Index2<=nUDegree;++Index2)
                {
                SGM::Point4D const &XYZW=aaControlPoints[nSpanIndex-nUDegree+Index2][Index1];
                double dBasisWeight=XYZW.m_w*aaBasisFunctions[0][Index2];
                aControlPoints[Index1]+=SGM::Vector4D(XYZW.m_x*dBasisWeight,XYZW.m_y*dBasisWeight,XYZW.m_z*dBasisWeight,dBasisWeight);
                }
            SGM::Point4D const &Pos=aControlPoints[Index1];
            double dRWeight=1.0/Pos.m_w;
            aControlPoints[Index1]=SGM::Point4D(Pos.m_x*dRWeight,Pos.m_y*dRWeight,Pos.m_z*dRWeight,Pos.m_w);
            }
        curve *pParamCurve=new NURBcurve(rResult,aControlPoints,GetVKnots());
        return pParamCurve;
        }
    }

curve *NURBsurface::VParamLine(SGM::Result &rResult, double dV) const
    {
    if(m_bSingularHighV && SGM::NearEqual(m_Domain.m_VDomain.m_dMax,dV,SGM_MIN_TOL,false))
        {
        SGM::Point3D Pos;
        Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
        return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
        }
    else if(m_bSingularLowV && SGM::NearEqual(m_Domain.m_VDomain.m_dMin,dV,SGM_MIN_TOL,false))
        {
        SGM::Point3D Pos;
        Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
        return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
        }
    else
        {
        std::vector<double> const &aVKnots=GetVKnots();
        std::vector<std::vector<SGM::Point4D> > const &aaControlPoints=GetControlPoints();
        size_t nVDegree=GetVDegree();
        size_t nSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nVDegree,dV,aVKnots);

        double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
        double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
        size_t Index1,Index2;
        for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
            {
            aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
            }
        FindBasisFunctions(nSpanIndex,dV,nVDegree,0,&aVKnots[0],aaBasisFunctions);

        std::vector<SGM::Point4D> aControlPoints;
        size_t nControlPoints=aaControlPoints.size();
        aControlPoints.assign(nControlPoints,SGM::Point4D(0.0,0.0,0.0,0.0));
        for(Index1=0;Index1<nControlPoints;++Index1)
            {
            for(Index2=0;Index2<=nVDegree;++Index2)
                {
                SGM::Point4D const &XYZW=aaControlPoints[Index1][nSpanIndex-nVDegree+Index2];
                double dBasisWeight=XYZW.m_w*aaBasisFunctions[0][Index2];
                aControlPoints[Index1]+=SGM::Vector4D(XYZW.m_x*dBasisWeight,XYZW.m_y*dBasisWeight,XYZW.m_z*dBasisWeight,dBasisWeight);
                }
            SGM::Point4D const &Pos=aControlPoints[Index1];
            double dRWeight=1.0/Pos.m_w;
            aControlPoints[Index1]=SGM::Point4D(Pos.m_x*dRWeight,Pos.m_y*dRWeight,Pos.m_z*dRWeight,Pos.m_w);
            }
        return new NURBcurve(rResult,aControlPoints,GetUKnots());
        }
    }

std::vector<SGM::Point3D> const &NURBsurface::GetSeedPoints() const
    {
    return m_aSeedPoints;
    }

std::vector<SGM::Point2D> const &NURBsurface::GetSeedParams() const
    {
    return m_aSeedParams;
    }

int NURBsurface::UContinuity() const
    {
    int nDegree=(int)GetUDegree();
    size_t Index1;
    size_t nKnots=m_aUKnots.size();
    int nCount=0;
    int nMaxCount=0;
    double dLookFor=m_aUKnots.front();
    for(Index1=1;Index1<nKnots;++Index1)
        {
        if(SGM::NearEqual(m_aUKnots[Index1],m_aUKnots.front(),SGM_MIN_TOL,false)==false && 
           SGM::NearEqual(m_aUKnots[Index1],m_aUKnots.back(),SGM_MIN_TOL,false)==false)
            {
            ++nCount;
            dLookFor=m_aUKnots[Index1];
            }
        else if(SGM::NearEqual(m_aUKnots[Index1],dLookFor,SGM_MIN_TOL,false))
            {
            ++nCount;
            if(nMaxCount<nCount)
                {
                nMaxCount=nCount;
                }
            }
        else
            {
            nCount=0;
            }
        }
    return std::max(0,nDegree-nMaxCount);
    }

int NURBsurface::VContinuity() const
    {
    int nDegree=(int)GetVDegree();
    size_t Index1;
    size_t nKnots=m_aVKnots.size();
    int nCount=0;
    int nMaxCount=0;
    double dLookFor=m_aVKnots.front();
    for(Index1=1;Index1<nKnots;++Index1)
        {
        if(SGM::NearEqual(m_aVKnots[Index1],m_aVKnots.front(),SGM_MIN_TOL,false)==false && 
           SGM::NearEqual(m_aVKnots[Index1],m_aVKnots.back(),SGM_MIN_TOL,false)==false)
            {
            ++nCount;
            dLookFor=m_aVKnots[Index1];
            }
        else if(SGM::NearEqual(m_aVKnots[Index1],dLookFor,SGM_MIN_TOL,false))
            {
            ++nCount;
            if(nMaxCount<nCount)
                {
                nMaxCount=nCount;
                }
            }
        else
            {
            nCount=0;
            }
        }
    return std::max(0,nDegree-nMaxCount);
    }

} // End of SGMInternal namespace