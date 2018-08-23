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
    Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
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

void NURBsurface::Evaluate(SGM::Point2D const &uv,
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

    double aUMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
    double *aaUBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
        {
        aaUBasisFunctions[Index1]=aUMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nUSpanIndex,uv.m_u,nUDegree,nUDerivatives,&m_aUKnots[0],aaUBasisFunctions);

    double aVMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
    double *aaVBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
        {
        aaVBasisFunctions[Index1]=aVMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
        }
    FindBasisFunctions(nVSpanIndex,uv.m_v,nVDegree,nVDerivatives,&m_aVKnots[0],aaVBasisFunctions);

    SGM::Point4D temp[SMG_MAX_NURB_DEGREE_PLUS_ONE];
    SGM::Point4D SKL[3][3];
    for(Index1=0;Index1<=nUDerivatives;++Index1)
        {
        for(Index2=0;Index2<=nVDegree;++Index2)
            {
            temp[Index2]=SGM::Point4D(0.0,0.0,0.0,0.0);
            for(Index3=0;Index3<=nUDegree;++Index3)
                {
                SGM::Point4D const &ControlPos=m_aaControlPoints[nUSpanIndex-nUDegree+Index3]
                [nVSpanIndex-nVDegree+Index2];
                double dBasis=ControlPos.m_w*aaUBasisFunctions[Index1][Index3];
                temp[Index2].m_x+=dBasis*ControlPos.m_x;
                temp[Index2].m_y+=dBasis*ControlPos.m_y;
                temp[Index2].m_z+=dBasis*ControlPos.m_z;
                temp[Index2].m_w+=dBasis;
                }
            }

        for(Index2=0;Index2<=nVDerivatives;++Index2)
            {
            SKL[Index1][Index2].m_x=0.0;
            SKL[Index1][Index2].m_y=0.0;
            SKL[Index1][Index2].m_z=0.0;
            SKL[Index1][Index2].m_w=0.0;
            for(Index3=0;Index3<=nVDegree;++Index3)
                {
                SKL[Index1][Index2].m_x+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_x;
                SKL[Index1][Index2].m_y+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_y;
                SKL[Index1][Index2].m_z+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_z;
                SKL[Index1][Index2].m_w+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_w;
                }
            }
        }

    // Convert to three-dimensional points.

    SGM::Vector3D values[3][3],Aders[3][3];
    double wders[3][3];

    Aders[0][0].m_x=SKL[0][0].m_x;
    Aders[0][0].m_y=SKL[0][0].m_y;
    Aders[0][0].m_z=SKL[0][0].m_z;
    wders[0][0]=SKL[0][0].m_w;

    int nMaxDerivatives=0;
    if(0<nUDerivatives || 0<nVDerivatives)
        {
        ++nMaxDerivatives;

        Aders[1][0].m_x=SKL[1][0].m_x;
        Aders[1][0].m_y=SKL[1][0].m_y;
        Aders[1][0].m_z=SKL[1][0].m_z;
        wders[1][0]=SKL[1][0].m_w;

        Aders[0][1].m_x=SKL[0][1].m_x;
        Aders[0][1].m_y=SKL[0][1].m_y;
        Aders[0][1].m_z=SKL[0][1].m_z;
        wders[0][1]=SKL[0][1].m_w;

        if(1<nUDerivatives || 1<nVDerivatives)
            {
            ++nMaxDerivatives;

            Aders[2][0].m_x=SKL[2][0].m_x;
            Aders[2][0].m_y=SKL[2][0].m_y;
            Aders[2][0].m_z=SKL[2][0].m_z;
            wders[2][0]=SKL[2][0].m_w;

            Aders[1][1].m_x=SKL[1][1].m_x;
            Aders[1][1].m_y=SKL[1][1].m_y;
            Aders[1][1].m_z=SKL[1][1].m_z;
            wders[1][1]=SKL[1][1].m_w;

            Aders[0][2].m_x=SKL[0][2].m_x;
            Aders[0][2].m_y=SKL[0][2].m_y;
            Aders[0][2].m_z=SKL[0][2].m_z;
            wders[0][2]=SKL[0][2].m_w;
            }
        }

    // Algorithm A4.4 from page 137-138 of the "NURB" book.

    int i,j,k,s;
    for(k=0;k<=nMaxDerivatives;++k)
        {
        for(s=0;s<=nMaxDerivatives-k;++s)
            {
            SGM::Vector3D v(Aders[k][s].m_x,Aders[k][s].m_y,Aders[k][s].m_z);
            for(j=1;j<=s;++j)
                {
                if(s==2 && j==1)
                    {
                    v=v-2*wders[0][j]*values[k][s-j];
                    }
                else
                    {
                    v=v-wders[0][j]*values[k][s-j];
                    }
                }
            for(i=1;i<=k;++i)
                {
                if(k==2 && i==1)
                    {
                    v=v-2*wders[i][0]*values[k-i][s];
                    }
                else
                    {
                    v=v-wders[i][0]*values[k-i][s];
                    }
                SGM::Vector3D v2(0.0,0.0,0.0);
                for(j=1;j<=s;++j)
                    {
                    if(s==2 && j==1)
                        {
                        v2=v2+2*wders[i][j]*values[k-i][s-j];
                        }
                    else
                        {
                        v2=v2+wders[i][j]*values[k-i][s-j];
                        }
                    }
                if(k==2 && i==1)
                    {
                    v=v-2*v2;
                    }
                else
                    {
                    v=v-v2;
                    }
                }
            double denom=1.0/wders[0][0];
            values[k][s].m_x=v.m_x*denom;
            values[k][s].m_y=v.m_y*denom;
            values[k][s].m_z=v.m_z*denom;
            }
        }

    // Fill in the answers.

    if(Pos)
        {
        *Pos=SGM::Point3D(values[0][0]);
        }
    if(Du)
        {
        *Du=values[1][0];
        }
    if(Dv)
        {
        *Dv=values[0][1];
        }
    if(Norm)
        {
        *Norm=values[1][0]*values[0][1];
        }
    if(Duu)
        {
        *Duu=values[2][0];
        }
    if(Duv)
        {
        *Duv=values[1][1];
        }
    if(Dvv)
        {
        *Dvv=values[0][2];
        }
    }

SGM::Point2D NURBsurface::Inverse(SGM::Point3D const &Pos,
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
        //TODO: implement pGuess in NURBSurface::Inverse
        throw std::logic_error("pGuess not implemented in NURBsurface::Inverse");
        }

    return uv;
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
