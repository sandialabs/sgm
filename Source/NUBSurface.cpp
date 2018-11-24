#include "SGMVector.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include "Faceter.h"

namespace SGMInternal
{
NUBsurface::NUBsurface(SGM::Result                                   &rResult,
                       std::vector<std::vector<SGM::Point3D> > const &aControlPoints,
                       std::vector<double>                     const &aUKnots,
                       std::vector<double>                     const &aVKnots):
        surface(rResult,SGM::NUBSurfaceType),
        m_aaControlPoints(std::move(aControlPoints)),
        m_aUKnots(std::move(aUKnots)),
        m_aVKnots(std::move(aVKnots))
    {
    m_Domain.m_UDomain.m_dMin=m_aUKnots.front();
    m_Domain.m_UDomain.m_dMax=m_aUKnots.back();
    m_Domain.m_VDomain.m_dMin=m_aVKnots.front();
    m_Domain.m_VDomain.m_dMax=m_aVKnots.back();

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

    SGM::Point3D Pos4,Pos5;
    Evaluate(m_Domain.MidPoint(0,0),&Pos4);
    Evaluate(m_Domain.MidPoint(1,1),&Pos5);
    if(SGM::NearEqual(Pos0,Pos4,SGM_MIN_TOL))
        {
        m_bSingularLowV=true;
        }
    if(SGM::NearEqual(Pos1,Pos5,SGM_MIN_TOL))
        {
        m_bSingularHighV=true;
        }
    if(SGM::NearEqual(Pos2,Pos4,SGM_MIN_TOL))
        {
        m_bSingularLowU=true;
        }
    if(SGM::NearEqual(Pos3,Pos5,SGM_MIN_TOL))
        {
        m_bSingularHighU=true;
        }

    curve *pUCurve=UParamLine(rResult,m_Domain.m_UDomain.MidPoint(0.25));
    curve *pVCurve=VParamLine(rResult,m_Domain.m_VDomain.MidPoint(0.25));
    FacetOptions Options;
    Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
    std::vector<double> aUParams,aVParams;
    std::vector<SGM::Point3D> aUPoints,aVPoints;
    FacetCurve(pUCurve,pUCurve->GetDomain(),Options,aUPoints,aUParams);
    FacetCurve(pVCurve,pVCurve->GetDomain(),Options,aVPoints,aVParams);
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
        double v=aUParams[Index1];
        for(Index2=0;Index2<m_nVParams;++Index2)
            {
            double u=aVParams[Index2];
            SGM::Point2D uv(u,v);
            m_aSeedParams.push_back(uv);
            SGM::Point3D Pos;
            Evaluate(uv,&Pos);
            m_aSeedPoints.push_back(Pos);
            }
        }
    }

bool NUBsurface::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    NUBsurface const *pNUB2=(NUBsurface const *)pOther;
    if(m_aUKnots.size()!=pNUB2->m_aUKnots.size())
        {
        return false;
        }
    if(m_aVKnots.size()!=pNUB2->m_aVKnots.size())
        {
        return false;
        }
    if(m_aaControlPoints.size()!=pNUB2->m_aaControlPoints.size())
        {
        return false;
        }
    if(m_aaControlPoints[0].size()!=pNUB2->m_aaControlPoints[0].size())
        {
        return false;
        }
    size_t Index1,Index2;
    size_t nUKnots=m_aUKnots.size();
    for(Index1=0;Index1<nUKnots;++Index1)
        {
        if(m_aUKnots[Index1]!=pNUB2->m_aUKnots[Index1])
            {
            return false;
            }
        }
    size_t nVKnots=m_aVKnots.size();
    for(Index1=0;Index1<nVKnots;++Index1)
        {
        if(m_aVKnots[Index1]!=pNUB2->m_aVKnots[Index1])
            {
            return false;
            }
        }
    size_t nSize1=m_aaControlPoints.size();
    size_t nSize2=m_aaControlPoints[0].size();
    for(Index1=0;Index1<nSize1;++Index1)
        {
        std::vector<SGM::Point3D> const &aControlPoints1=m_aaControlPoints[Index1];
        std::vector<SGM::Point3D> const &aControlPoints2=pNUB2->m_aaControlPoints[Index1];
        for(Index2=0;Index2<nSize2;++Index2)
            {
            if(SGM::NearEqual(aControlPoints1[Index2],aControlPoints2[Index2],dTolerance)==false)
                {
                return false;
                }
            }
        }
    return true;
    }

NUBsurface::NUBsurface(SGM::Result &rResult, NUBsurface const &other) :
        surface(rResult, other),
        m_aaControlPoints(other.m_aaControlPoints),
        m_aUKnots(other.m_aUKnots),
        m_aVKnots(other.m_aVKnots),
        m_aSeedPoints(other.m_aSeedPoints),
        m_aSeedParams(other.m_aSeedParams),
        m_nUParams(other.m_nUParams),
        m_nVParams(other.m_nVParams)
{}

NUBsurface* NUBsurface::Clone(SGM::Result &rResult) const
{ return new NUBsurface(rResult, *this); }

void NUBsurface::Evaluate(SGM::Point2D const &uv,
                          SGM::Point3D       *Pos,
                          SGM::Vector3D      *Du,
                          SGM::Vector3D      *Dv,
                          SGM::UnitVector3D  *Norm,
                          SGM::Vector3D      *Duu,
                          SGM::Vector3D      *Duv,
                          SGM::Vector3D      *Dvv) const
    {
    // From "The NURBs Book" Algorithm A3.6.

    size_t nUDegree=GetUDegree();
    size_t nUSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,uv.m_u,m_aUKnots);

    size_t nVDegree=GetVDegree();
    size_t nVSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nVDegree,uv.m_v,m_aVKnots);

    size_t nUDerivatives=0;
    if(Du || Norm || Duv) nUDerivatives=1;
    if(Duu) nUDerivatives=2;

    size_t nVDerivatives=0;
    if(Dv || Norm || Duv) nVDerivatives=1;
    if(Dvv) nVDerivatives=2;

    size_t Index1,Index2,Index3;

    double aUMemory[SGM_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
    double *aaUBasisFunctions[SGM_MAX_NURB_DEGREE_PLUS_ONE];
    for(Index1=0;Index1<SGM_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
        {
        aaUBasisFunctions[Index1]=aUMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nUSpanIndex,uv.m_u,nUDegree,nUDerivatives,&m_aUKnots[0],aaUBasisFunctions);

    double aVMemory[SGM_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
    double *aaVBasisFunctions[SGM_MAX_NURB_DEGREE_PLUS_ONE];
    for(Index1=0;Index1<SGM_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
        {
        aaVBasisFunctions[Index1]=aVMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nVSpanIndex,uv.m_v,nVDegree,nVDerivatives,&m_aVKnots[0],aaVBasisFunctions);

    SGM::Point3D temp[SGM_MAX_NURB_DEGREE_PLUS_ONE];
    SGM::Point3D SKL[3][3];
    for(Index1=0;Index1<=nUDerivatives;++Index1)
        {
        for(Index2=0;Index2<=nVDegree;++Index2)
            {
            temp[Index2]=SGM::Point3D(0.0,0.0,0.0);
            for(Index3=0;Index3<=nUDegree;++Index3)
                {
                double dFactor=aaUBasisFunctions[Index1][Index3];
                SGM::Point3D const &ControlPos=m_aaControlPoints[nUSpanIndex-nUDegree+Index3]
                [nVSpanIndex-nVDegree+Index2];
                temp[Index2].m_x+=dFactor*ControlPos.m_x;
                temp[Index2].m_y+=dFactor*ControlPos.m_y;
                temp[Index2].m_z+=dFactor*ControlPos.m_z;
                }
            }

        for(Index2=0;Index2<=nVDerivatives;++Index2)
            {
            SKL[Index1][Index2].m_x=0.0;
            SKL[Index1][Index2].m_y=0.0;
            SKL[Index1][Index2].m_z=0.0;
            for(Index3=0;Index3<=nVDegree;++Index3)
                {
                SKL[Index1][Index2].m_x+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_x;
                SKL[Index1][Index2].m_y+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_y;
                SKL[Index1][Index2].m_z+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_z;
                }
            }
        }

    // Fill in the answers.

    if(Pos)
        {
        *Pos=SKL[0][0];
        }
    if(Du)
        {
        *Du=SGM::Vector3D(SKL[1][0]);
        }
    if(Dv)
        {
        *Dv=SGM::Vector3D(SKL[0][1]);
        }
    if(Norm)
        {
        *Norm=SGM::Vector3D(SKL[1][0])*SGM::Vector3D(SKL[0][1]);
        }
    if(Duu)
        {
        *Duu=SGM::Vector3D(SKL[2][0]);
        }
    if(Duv)
        {
        *Duv=SGM::Vector3D(SKL[1][1]);
        }
    if(Dvv)
        {
        *Dvv=SGM::Vector3D(SKL[0][2]);
        }
    }

SGM::Point2D NUBsurface::Inverse(SGM::Point3D const &Pos,
                                 SGM::Point3D       *ClosePos,
                                 SGM::Point2D const *pGuess) const
    {
    SGM::Point2D uv;

    SGM::Point2D StartUV(0.0,0.0);
    if(pGuess)
        {
        StartUV=*pGuess;
        }
    else
        {
        std::vector<SGM::Point3D> const &aSeedPoints=GetSeedPoints();
        std::vector<SGM::Point2D> const &aSeedParams=GetSeedParams();
        size_t nSeedPoints=aSeedPoints.size();
        size_t Index1;
        double dMin=std::numeric_limits<double>::max();
        for(Index1=0;Index1<nSeedPoints;++Index1)
            {
            double dDist=aSeedPoints[Index1].DistanceSquared(Pos);
            if(dDist<dMin)
                {
                dMin=dDist;
                StartUV=aSeedParams[Index1];
                }
            }
        }

    uv=NewtonsMethod(StartUV,Pos);
    if(ClosePos)
        {
        Evaluate(uv,ClosePos);
        }
    if(pGuess)
        {
        //TODO: implement pGuess in NUBSurface::Inverse
        throw std::logic_error("pGuess not implemented in NUBsurface::Inverse");
        }

    return uv;
    }

void NUBsurface::Transform(SGM::Result            &,//rResult,
                           SGM::Transform3D const &Trans)
    {
    size_t nSize1=m_aaControlPoints.size();
    size_t nSize2=m_aaControlPoints[0].size();
    for(size_t Index1=0;Index1<nSize1;++Index1)
        {
        for(size_t Index2=0;Index2<nSize2;++Index2)
            {
            m_aaControlPoints[Index1][Index2]=Trans*m_aaControlPoints[Index1][Index2];
            }
        }
    }

curve *NUBsurface::UParamLine(SGM::Result &rResult, double dU) const
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
        std::vector<std::vector<SGM::Point3D> > const &aaControlPoints=GetControlPoints();
        size_t nUDegree=GetUDegree();
        size_t nSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,dU,aUKnots);

        double aMemory[SGM_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
        double *aaBasisFunctions[SGM_MAX_NURB_DEGREE_PLUS_ONE];
        size_t Index1,Index2;
        for(Index1=0;Index1<SGM_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
            {
            aaBasisFunctions[Index1]=aMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
            }
        FindBasisFunctions(nSpanIndex,dU,nUDegree,0,&aUKnots[0],aaBasisFunctions);

        std::vector<SGM::Point3D> aControlPoints;
        size_t nControlPoints=aaControlPoints[0].size();
        aControlPoints.assign(nControlPoints,SGM::Point3D(0,0,0));
        for(Index1=0;Index1<nControlPoints;++Index1)
            {
            for(Index2=0;Index2<=nUDegree;++Index2)
                {
                aControlPoints[Index1]+=aaBasisFunctions[0][Index2]*
                                        SGM::Vector3D(aaControlPoints[nSpanIndex-nUDegree+Index2][Index1]);
                }
            }
        curve *pCurve=new NUBcurve(rResult,std::move(aControlPoints),std::vector<double>(GetVKnots()));
        return pCurve;
        }
    }

curve *NUBsurface::VParamLine(SGM::Result &rResult, double dV) const
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
        std::vector<std::vector<SGM::Point3D> > const &aaControlPoints=GetControlPoints();
        size_t nUDegree=GetUDegree();
        size_t nSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nUDegree,dV,aVKnots);

        double aMemory[SGM_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
        double *aaBasisFunctions[SGM_MAX_NURB_DEGREE_PLUS_ONE];
        size_t Index1,Index2;
        for(Index1=0;Index1<SGM_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
            {
            aaBasisFunctions[Index1]=aMemory+Index1*SGM_MAX_NURB_DEGREE_PLUS_ONE;
            }
        FindBasisFunctions(nSpanIndex,dV,nUDegree,0,&aVKnots[0],aaBasisFunctions);

        std::vector<SGM::Point3D> aControlPoints;
        size_t nControlPoints=aaControlPoints.size();
        aControlPoints.assign(nControlPoints,SGM::Point3D(0,0,0));
        for(Index1=0;Index1<nControlPoints;++Index1)
            {
            for(Index2=0;Index2<=nUDegree;++Index2)
                {
                aControlPoints[Index1]+=aaBasisFunctions[0][Index2]*
                                        SGM::Vector3D(aaControlPoints[Index1][nSpanIndex-nUDegree+Index2]);
                }
            }
        return new NUBcurve(rResult,std::move(aControlPoints),std::vector<double>(GetUKnots()));
        }
    }

std::vector<SGM::Point3D> const &NUBsurface::GetSeedPoints() const
    {
    return m_aSeedPoints;
    }

std::vector<SGM::Point2D> const &NUBsurface::GetSeedParams() const
    {
    return m_aSeedParams;
    }

int NUBsurface::UContinuity() const
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

int NUBsurface::VContinuity() const
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