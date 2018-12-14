#include <cmath>
#include <algorithm>
#include <limits>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include "Topology.h"
#include "Mathematics.h"

namespace SGMInternal
{
surface::surface(SGM::Result &rResult,SGM::EntityType nType) :
        entity(rResult,SGM::EntityType::SurfaceType),
        m_SurfaceType(nType)
    {
    m_bClosedU=false;
    m_bClosedV=false;
    m_bSingularLowU=false;
    m_bSingularHighU=false;
    m_bSingularLowV=false;
    m_bSingularHighV=false;
    }
    
surface::surface(SGM::Result &rResult, surface const &other) :
        entity(rResult, other),
        m_sFaces(other.m_sFaces),
        m_Domain(other.m_Domain),
        m_SurfaceType(other.m_SurfaceType),
        m_bClosedU(other.m_bClosedU),
        m_bClosedV(other.m_bClosedV),
        m_bSingularLowU(other.m_bSingularLowU),
        m_bSingularHighU(other.m_bSingularHighU),
        m_bSingularLowV(other.m_bSingularLowV),
        m_bSingularHighV(other.m_bSingularHighV)
{}

void surface::FindAllChildren(std::set<entity *, EntityCompare> &) const
    {
    // do nothing, derived classes can override
    }

void surface::GetParents(std::set<entity *, EntityCompare> &sParents) const
{
    for (auto pFace : m_sFaces)
    {
      sParents.emplace(pFace);
    }
    entity::GetParents(sParents);
}

void surface::RemoveParentsInSet(SGM::Result &rResult,
                        std::set<entity *,EntityCompare>  const &sParents)
{
    for (auto pFace : m_sFaces)
    {
        if (sParents.find(pFace) != sParents.end())
        {
            pFace->SetSurface(nullptr);
            m_sFaces.erase(pFace);
        }
    }
    entity::RemoveParentsInSet(rResult, sParents);
}

void surface::RemoveParents(SGM::Result &rResult)
{
    for (auto pFace : m_sFaces)
    {
        pFace->SetSurface(nullptr);
    }
    m_sFaces.clear();
    entity::RemoveParents(rResult);
}

void surface::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.
    std::set<face *,EntityCompare> m_sFixedFaces;
    for(auto pFace : m_sFaces)
        {
        auto MapValue=mEntityMap.find(pFace);
        if(MapValue!=mEntityMap.end())
            m_sFixedFaces.insert((face *)MapValue->second);
        //else
        //    m_sFixedFaces.insert(pFace);
        }
    m_sFaces=m_sFixedFaces;
    OwnerAndAttributeReplacePointers(mEntityMap);
    }

///////////////////////////////////////////////////////////////////////////////
//
// surface member functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Point2D surface::NewtonsMethod(SGM::Point2D const &StartUV,
                                    SGM::Point3D const &Pos) const
    {
    double DeltaU=std::numeric_limits<double>::max();
    double DeltaV=std::numeric_limits<double>::max();
    double dDot=std::numeric_limits<double>::max();
    size_t nCount=0;
    SGM::Point3D SurfPos;
    SGM::Vector3D DU,DV;
    SGM::UnitVector3D Norm;
    SGM::Point2D Answer=StartUV;
    while(nCount<100 && fabs(dDot) >= SGM_ZERO &&
       (SGM_ZERO<DeltaU || DeltaU<-SGM_ZERO ||
        SGM_ZERO<DeltaV || DeltaV<-SGM_ZERO))
        {
        Evaluate(Answer,&SurfPos,&DU,&DV,&Norm);
        dDot=(Pos-SurfPos)%Norm;
        SGM::Point3D ProjectPos=Pos-Norm*dDot;
        SGM::Vector3D S=ProjectPos-SurfPos;
        double dUMag=DU.MagnitudeSquared();
        double dVMag=DV.MagnitudeSquared();
        if(SGM_ZERO<dUMag)
            {
            DeltaU=(S%DU)/DU.MagnitudeSquared();
            }
        else
            {
            DeltaU=0.0;
            }
        if(SGM_ZERO<dVMag)
            {
            DeltaV=(S%DV)/DV.MagnitudeSquared();
            }
        else
            {
            DeltaV=0.0;
            }
        Answer.m_u+=DeltaU;
        Answer.m_v+=DeltaV;
        SnapToDomain(Answer);
        ++nCount;
        }
    return Answer;
    }

bool surface::IsSingularity(SGM::Point2D const &uv,double dTolerance) const
    {
    if(m_bSingularHighU)
        {
        if(SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularHighV)
        {
        if(SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMax,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularLowU)
        {
        if(SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularLowV)
        {
        if(SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMin,dTolerance,false))
            {
            return true;
            }
        }
    return false;
    }

void surface::PrincipleCurvature(SGM::Point2D const &uv,
                                 SGM::UnitVector3D  &Vec1,
                                 SGM::UnitVector3D  &Vec2,
                                 double             &k1,
                                 double             &k2) const
    {
    // Non override version:
    // Find the eigen vectors and values of the second fundamental form.
    //
    //  | L M |  L = Duu % Norm, M = Duv % Norm, N = Dvv % Norm.
    //  | M N |

    SGM::UnitVector3D Norm;
    SGM::Vector3D Du,Dv,Duu,Duv,Dvv;
    Evaluate(uv,nullptr,&Du,&Dv,&Norm,&Duu,&Duv,&Dvv);
    double L=Duu%Norm;
    double M=Duv%Norm;
    double N=Dvv%Norm;
    const double aaMatrix[2][2] =
        {
        L, M,
        M, N
        };
    std::vector<double> aValues;
    std::vector<SGM::UnitVector2D> aVectors;
    size_t nValues=SGM::FindEigenVectors2D(&aaMatrix[0],aValues,aVectors);
    if(nValues==2)
        {
        SGM::UnitVector3D UDu=Du,UDv=Dv;
        k1=aValues[0];
        k2=aValues[1];
        Vec1=aVectors[0].m_u*UDu+aVectors[0].m_v*UDv;
        Vec2=aVectors[1].m_u*UDu+aVectors[1].m_v*UDv;
        }
    }

double surface::DirectionalCurvature(SGM::Point2D      const &uv,
                                     SGM::UnitVector3D const &Direction) const
    {
    SGM::UnitVector3D Vec1,Vec2,Norm;
    double k1,k2;
    PrincipleCurvature(uv,Vec1,Vec2,k1,k2);
    Norm=Vec1*Vec2;
    double dt=Direction.Angle(Vec1,Norm);
    double dCos=cos(dt);
    double dSin=sin(dt);
    return k1*dCos*dCos+k2*dSin*dSin;
    }

int surface::UContinuity() const
    { return std::numeric_limits<int>::max(); }

int surface::VContinuity() const
    { return std::numeric_limits<int>::max(); }

void surface::SnapToDomain(SGM::Point2D &uv) const
    {
    if(m_bClosedU)
        {
        while(uv.m_u<m_Domain.m_UDomain.m_dMin)
            {
            uv.m_u+=m_Domain.m_UDomain.Length();
            }
        while(m_Domain.m_UDomain.m_dMax<uv.m_u)
            {
            uv.m_u-=m_Domain.m_UDomain.Length();
            }
        }
    else
        {
        if(uv.m_u<m_Domain.m_UDomain.m_dMin)
            {
            uv.m_u=m_Domain.m_UDomain.m_dMin;
            }
        if(m_Domain.m_UDomain.m_dMax<uv.m_u)
            {
            uv.m_u=m_Domain.m_UDomain.m_dMax;
            }
        }
    if(m_bClosedV)
        {
        while(uv.m_v<m_Domain.m_VDomain.m_dMin)
            {
            uv.m_v+=m_Domain.m_VDomain.Length();
            }
        while(m_Domain.m_VDomain.m_dMax<uv.m_v)
            {
            uv.m_v-=m_Domain.m_VDomain.Length();
            }
        }
    else
        {
        if(uv.m_v<m_Domain.m_VDomain.m_dMin)
            {
            uv.m_v=m_Domain.m_VDomain.m_dMin;
            }
        if(m_Domain.m_VDomain.m_dMax<uv.m_v)
            {
            uv.m_v=m_Domain.m_VDomain.m_dMax;
            }
        }
    }

SGM::UnitVector2D surface::FindSurfaceDirection(SGM::Point2D        &uv,
                                                SGM::Vector3D const &Vec) const
    {
    SGM::Vector3D Du,Dv;
    Evaluate(uv,nullptr,&Du,&Dv);
    SGM::UnitVector3D UDu=Du,VDv=Dv;
    double du=UDu%Vec;
    double dv=VDv%Vec;
    return {du,dv};
    }


} // end namespace
