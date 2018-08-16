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

namespace SGMInternal
{
surface::surface(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::SurfaceType),m_SurfaceType(nType)
    {
    m_bClosedU=false;
    m_bClosedV=false;
    m_bSingularLowU=false;
    m_bSingularHighU=false;
    m_bSingularLowV=false;
    m_bSingularHighV=false;
    }

surface *surface::Clone(SGM::Result &) const
    {
    return nullptr;
    }

void surface::FindAllChildren(std::set<entity *, EntityCompare> &) const
    {
    // do nothing, derived classes can override
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
        else
            m_sFixedFaces.insert(pFace);
        }
    m_sFaces=m_sFixedFaces;

    std::set<attribute *,EntityCompare> m_sFixedAttributes;
    for(auto pAttribute : m_sAttributes)
        {
        auto MapValue=mEntityMap.find(pAttribute);
        if(MapValue!=mEntityMap.end())
            m_sFixedAttributes.insert((attribute *)MapValue->second);
        else
            m_sFixedAttributes.insert(pAttribute);
        }
    m_sAttributes=m_sFixedAttributes;

    std::set<entity *,EntityCompare> m_sFixedOwners;
    for(auto pEntity : m_sOwners)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            m_sFixedOwners.insert((attribute *)MapValue->second);
        else
            m_sFixedOwners.insert(pEntity);
        }
    m_sOwners=m_sFixedOwners;
    }

///////////////////////////////////////////////////////////////////////////////
//
// surface virtual member functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Point2D surface::Inverse(SGM::Point3D const &,
                              SGM::Point3D       *,
                              SGM::Point2D const *) const
    { throw std::logic_error("Derived class of surface should override Inverse()"); }

bool surface::IsSame(surface const *pOther,double dTolerance) const
    { return false; } // Derived classes may override

void surface::Transform(SGM::Transform3D const &)
    { throw std::logic_error("Derived class of surface must override Transform()"); }

curve *surface::UParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override UParamLine()"); }

curve *surface::VParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override VParamLine()"); }

void surface::Evaluate(SGM::Point2D const &uv,
                       SGM::Point3D       *Pos,
                       SGM::Vector3D      *Du,
                       SGM::Vector3D      *Dv,
                       SGM::UnitVector3D  *Norm,
                       SGM::Vector3D      *Duu,
                       SGM::Vector3D      *Duv,
                       SGM::Vector3D      *Dvv) const
    { throw std::logic_error("Derived classes must override Evaluate()"); }

///////////////////////////////////////////////////////////////////////////////
//
// surface other member functions
//
///////////////////////////////////////////////////////////////////////////////

void surface::AddFace(face *pFace)
    { m_sFaces.insert(pFace); }

void surface::RemoveFace(face *pFace)
    { m_sFaces.erase(pFace); }

double AreaIntegrand(SGM::Point2D const &uv,void const *pData)
    {
    surface const *pSurface=(surface const *)pData;
    SGM::Vector3D VecU,VecV;
    pSurface->Evaluate(uv,nullptr,&VecU,&VecV);
    return (VecU*VecV).Magnitude();
    }

double surface::FindAreaOfParametricTriangle(SGM::Result        &,//rResult,
                                             SGM::Point2D const &PosA,
                                             SGM::Point2D const &PosB,
                                             SGM::Point2D const &PosC) const
    {
    return SGM::IntegrateTriangle(SGMInternal::AreaIntegrand,PosA,PosB,PosC,this);
    }

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
        DeltaU=(S%DU)/DU.MagnitudeSquared();
        DeltaV=(S%DV)/DV.MagnitudeSquared();
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
    {
    switch(m_SurfaceType)
        {
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUB=(NUBsurface const*)this;
            return pNUB->UContinuity();
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURB=(NURBsurface const*)this;
            return pNURB->UContinuity();
            }
        default:
            {
            return std::numeric_limits<int>::max();
            }
        }
    }

int surface::VContinuity() const
    {
    switch(m_SurfaceType)
        {
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUB=(NUBsurface const*)this;
            return pNUB->VContinuity();
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURB=(NURBsurface const*)this;
            return pNURB->VContinuity();
            }
        default:
            {
            return std::numeric_limits<int>::max();
            }
        }
    }

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
    return SGM::UnitVector2D(du,dv);
    }


} // end namespace
