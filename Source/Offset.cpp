#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Primitive.h"
#include "Faceter.h"

namespace SGMInternal
{

offset::offset(SGM::Result &rResult, double distance, surface *pSurface) :
        surface(rResult, SGM::OffsetType),
        m_pSurface(nullptr),
        m_dDistance(distance)
    {
    if(pSurface)
        {
        SetSurface(pSurface);
        }
    }

offset::offset(SGM::Result &rResult, offset const &other) :
        surface(rResult, SGM::OffsetType),
        m_pSurface(other.m_pSurface),
        m_dDistance(other.m_dDistance)
    {
    }

offset::~offset()
    {
    if (m_pSurface)
        m_pSurface->RemoveOwner(this);
    }

offset *offset::Clone(SGM::Result &rResult) const
    { return new offset(rResult, *this); }

void offset::WriteSGM(SGM::Result                  &,
                      FILE                         *,
                      SGM::TranslatorOptions const &) const
    { throw std::logic_error("Derived class of surface must override WriteSGM()"); }

surface *offset::GetSurface() const
    { return m_pSurface; }

void offset::SetSurface(surface *pSurface)
    {
    pSurface->AddOwner(this);
    m_pSurface = pSurface;
    this->m_bClosedU = pSurface->ClosedInU();
    this->m_bClosedV = pSurface->ClosedInV();
    this->m_bSingularHighU = pSurface->SingularHighU();
    this->m_bSingularHighV = pSurface->SingularHighV();
    this->m_bSingularLowU = pSurface->SingularLowU();
    this->m_bSingularLowV = pSurface->SingularLowV();
    this->m_bClosedV = pSurface->ClosedInV();
    m_Domain=pSurface->GetDomain();
    }

bool offset::IsSame(surface const *pOther,double /*dTolerance*/) const
    {
    if (pOther->GetSurfaceType() != m_SurfaceType)
        {
        return false;
        }
    throw std::logic_error("Derived class of surface must override IsSame()");
    }

void offset::FindAllChildren(std::set<entity *, EntityCompare> &/*sChildren*/) const
    {
    throw std::logic_error("Derived class of surface must override FindAllChildren()");
    }

void offset::ReplacePointers(std::map<entity *,entity *> const &)
{
//    surface::ReplacePointers(mEntityMap);
//
//    auto MapValue=mEntityMap.find(m_pSurface);
//    if(MapValue!=mEntityMap.end())
//        m_pSurface = dynamic_cast<surface *>(MapValue->second);
//    else
//        throw std::runtime_error("offset ReplacePointers did not find a surface in the map");
}

void offset::Evaluate(SGM::Point2D const &uv,
                      SGM::Point3D       *Pos,
                      SGM::Vector3D      *Du,
                      SGM::Vector3D      *Dv,
                      SGM::UnitVector3D  *Norm,
                      SGM::Vector3D      *Duu,
                      SGM::Vector3D      *Duv,
                      SGM::Vector3D      *Dvv) const
    {
    SGM::Point3D BPos;
    SGM::UnitVector3D BNorm;
    SGM::Vector3D du,dv,duu,duv,dvv;
    m_pSurface->Evaluate(uv,&BPos,&du,&dv,&BNorm,&duu,&duv,&dvv);
            
    if(Pos)
        {
        *Pos=BPos+BNorm*m_dDistance;
        }
    if(Du)
        {
        SGM::Vector3D q=du*dv;
        SGM::Vector3D dq=duu*dv+du*duv;
        double dDot=q%q;
        double w=1/sqrt(dDot);
        double dw=-(dq%q)*w/dDot;
        *Du=du+m_dDistance*(q*dw+dq*w);
        }
    if(Dv)
        {
        SGM::Vector3D q=du*dv;
        SGM::Vector3D dq=duv*dv+du*dvv;
        double dDot=q%q;
        double w=1/sqrt(dDot);
        double dw=-(dq%q)*w/dDot;
        *Dv=dv+m_dDistance*(q*dw+dq*w);
        }
    if(Norm)
        {
        *Norm=BNorm;
        }
    if(Duu)
        {
        double h=SGM_MIN_TOL;
        SGM::Point2D uv0(uv.m_u-2*h,uv.m_v),uv1(uv.m_u-h,uv.m_v),uv2(uv.m_u+h,uv.m_v),uv3(uv.m_u+2*h,uv.m_v);
        SGM::Vector3D P0,P1,P2,P3;
        Evaluate(uv0,nullptr,&P0);
        Evaluate(uv1,nullptr,&P1);
        Evaluate(uv2,nullptr,&P2);
        Evaluate(uv3,nullptr,&P3);
        *Duu=SGM::FirstDerivative<SGM::Point3D,SGM::Vector3D>(SGM::Point3D(P0),SGM::Point3D(P1),SGM::Point3D(P2),SGM::Point3D(P3),h);
        }
    if(Duv)
        {
        double h=SGM_MIN_TOL;
        SGM::Point2D uv0(uv.m_u,uv.m_v-2*h),uv1(uv.m_u,uv.m_v-h),uv2(uv.m_u,uv.m_v+h),uv3(uv.m_u,uv.m_v+2*h);
        SGM::Vector3D P0,P1,P2,P3;
        Evaluate(uv0,nullptr,&P0);
        Evaluate(uv1,nullptr,&P1);
        Evaluate(uv2,nullptr,&P2);
        Evaluate(uv3,nullptr,&P3);
        *Duv=SGM::FirstDerivative<SGM::Point3D,SGM::Vector3D>(SGM::Point3D(P0),SGM::Point3D(P1),SGM::Point3D(P2),SGM::Point3D(P3),h);
        }
    if(Dvv)
        {
        double h=SGM_MIN_TOL;
        SGM::Point2D uv0(uv.m_u,uv.m_v-2*h),uv1(uv.m_u,uv.m_v-h),uv2(uv.m_u,uv.m_v+h),uv3(uv.m_u,uv.m_v+2*h);
        SGM::Vector3D P0,P1,P2,P3;
        Evaluate(uv0,nullptr,nullptr,&P0);
        Evaluate(uv1,nullptr,nullptr,&P1);
        Evaluate(uv2,nullptr,nullptr,&P2);
        Evaluate(uv3,nullptr,nullptr,&P3);
        *Dvv=SGM::FirstDerivative<SGM::Point3D,SGM::Vector3D>(SGM::Point3D(P0),SGM::Point3D(P1),SGM::Point3D(P2),SGM::Point3D(P3),h);
        }
    }

SGM::Point2D offset::Inverse(SGM::Point3D const &Pos,
                             SGM::Point3D       *ClosePos,
                             SGM::Point2D const *pGuess) const
    {
    SGM::Point2D StartUV=m_pSurface->Inverse(Pos,nullptr,pGuess);
    StartUV=NewtonsMethod(StartUV,Pos);
    if(ClosePos)
        {
        Evaluate(StartUV,ClosePos);
        }
    return StartUV;
    }

void offset::Transform(SGM::Result            &,//rResult,
                       SGM::Transform3D const &)
    { throw std::logic_error("Derived class of surface must override Transform()"); }

curve *offset::UParamLine(SGM::Result &rResult, 
                          double       dU) const
    { 
    SGM::Result Nothing(nullptr);
    curve *pCurve=m_pSurface->UParamLine(Nothing,dU);
    std::vector<SGM::Point3D> aInterpolate;
    std::vector<double> aParams;
    FacetOptions Options;
    FacetCurve(pCurve,m_Domain.m_VDomain,Options,aInterpolate,aParams);
    curve *pAnswer=CreateNUBCurve(rResult,aInterpolate,&aParams);
    delete pCurve;
    return pAnswer;
    }

curve *offset::VParamLine(SGM::Result &rResult, 
                          double       dV) const
    { 
    SGM::Result Nothing(nullptr);
    curve *pCurve=m_pSurface->VParamLine(Nothing,dV);
    std::vector<SGM::Point3D> aInterpolate;
    std::vector<double> aParams;
    FacetOptions Options;
    FacetCurve(pCurve,m_Domain.m_UDomain,Options,aInterpolate,aParams);
    curve *pAnswer=CreateNUBCurve(rResult,aInterpolate,&aParams);
    delete pCurve;
    return pAnswer;
    }

} // namespace SGMInternal

