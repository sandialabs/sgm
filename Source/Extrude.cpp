#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{

extrude::extrude(SGM::Result             &rResult,
                 SGM::UnitVector3D const &vAxis,
                 curve                   *pCurve): 
    surface(rResult, SGM::ExtrudeType),
    m_pCurve(nullptr),
    m_Origin(),
    m_vAxis(vAxis)
    {
    m_Domain.m_VDomain=SGM::Interval1D(-SGM_MAX,SGM_MAX);
    if (pCurve)
        SetCurve(pCurve);
    }

extrude::~extrude()
    {
    if (m_pCurve)
        m_pCurve->RemoveOwner(this);
    }

SGM::UnitVector3D const &extrude::GetAxis() const
    {
    return m_vAxis;
    }

curve *extrude::GetCurve() const
    {
    return m_pCurve;
    }

SGM::Point3D const &extrude::GetOrigin() const
    {
    return m_Origin;
    }

bool extrude::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    auto pExtrude2=(extrude const *)pOther;
    if(!SGM::NearEqual(m_Origin, pExtrude2->m_Origin, dTolerance))
        {
        return false;
        }
    if(!SGM::NearEqual(m_vAxis, pExtrude2->m_vAxis, dTolerance))
        {
        return false;
        }
    return m_pCurve->IsSame(pExtrude2->m_pCurve, dTolerance);
    }

extrude::extrude(SGM::Result &rResult, extrude const &other) :
        surface(rResult, other),
        m_pCurve(other.m_pCurve),
        m_Origin(other.m_Origin),
        m_vAxis(other.m_vAxis)
    {
    }

extrude* extrude::Clone(SGM::Result &rResult) const
    { return new extrude(rResult, *this); }

void extrude::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(m_pCurve);
    }

void extrude::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
{
    surface::ReplacePointers(mEntityMap);

    auto MapValue=mEntityMap.find(m_pCurve);
    if(MapValue!=mEntityMap.end())
        m_pCurve = dynamic_cast<curve *>(MapValue->second);
    else
        throw std::runtime_error("extrude ReplacePointers did not find a curve in the map");
}

void extrude::Evaluate(SGM::Point2D const &uv,
                       SGM::Point3D       *Pos,
                       SGM::Vector3D      *Du,
                       SGM::Vector3D      *Dv,
                       SGM::UnitVector3D  *Norm,
                       SGM::Vector3D      *Duu,
                       SGM::Vector3D      *Duv,
                       SGM::Vector3D      *Dvv) const
    {
    SGM::Point3D   CurvePos;
    SGM::Vector3D  DuCurve;
    SGM::Vector3D *pDuCurve = &DuCurve;
    SGM::Vector3D  DuuCurve;
    SGM::Vector3D *pDuuCurve = &DuuCurve;

    if (nullptr == Dv && nullptr == Duv && nullptr == Duu && nullptr == Norm)
        pDuCurve = nullptr;
    if (nullptr == Duu)
        pDuuCurve = nullptr;

    // Evaluate the defining curve.

    m_pCurve->Evaluate(uv.m_u, &CurvePos, pDuCurve, pDuuCurve);

    // Fill in the answers.

    if(Pos)
        {
        *Pos=CurvePos+m_vAxis*uv.m_v;
        }
    if(Du)
        {
        assert(pDuCurve != nullptr);
        *Du=*pDuCurve;
        }
    if(Dv)
        {
        *Dv=m_vAxis;
        }
    if(Norm)
        {
        assert(pDuCurve != nullptr);
        *Norm=(*pDuCurve)*m_vAxis;
        }
    if(Duu)
        {
        assert(pDuuCurve != nullptr);
        *Duu=*pDuuCurve;
        }
    if(Duv)
        {
        *Duv=SGM::Vector3D(0,0,0);
        }
    if(Dvv)
        {
        *Dvv=SGM::Vector3D(0,0,0);
        }
    }

SGM::Point2D extrude::Inverse(SGM::Point3D const &Pos,
                              SGM::Point3D       *ClosePos,
                              SGM::Point2D const *pGuess) const
    {
    SGM::Point2D uv;

    uv.m_v=(Pos-m_Origin)%m_vAxis;
    SGM::Point3D PlanePos=Pos-m_vAxis*uv.m_v;
    if(pGuess)
        {
        uv.m_u=m_pCurve->Inverse(PlanePos,nullptr,&(pGuess->m_u));
        }
    else
        {
        uv.m_u=m_pCurve->Inverse(PlanePos);
        }

    if(ClosePos != nullptr)
        {
        Evaluate(uv,ClosePos);
        }

    return uv;
    }

    
void extrude::Transform(SGM::Result            &,//rResult,
                        SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_vAxis=Trans*m_vAxis;
    }

curve *extrude::UParamLine(SGM::Result &rResult, double dU) const
    { 
    SGM::Point3D Pos;
    m_pCurve->Evaluate(dU,&Pos);
    return new line(rResult,Pos,m_vAxis);
    }

curve *extrude::VParamLine(SGM::Result &rResult, double dV) const
    { 
    SGM::Vector3D Offset=m_vAxis*dV;
    curve *pCurve=(curve *)CopyEntity(rResult,m_pCurve);
    SGM::Transform3D Trans(Offset);
    pCurve->Transform(rResult,Trans);
    return pCurve; 
    }

void extrude::SetCurve(curve *pCurve)
    {
    pCurve->AddOwner(this);
    m_pCurve = pCurve;

    pCurve->Evaluate(pCurve->GetDomain().MidPoint(),&m_Origin);

    this->m_bClosedU = pCurve->GetClosed();
    m_Domain.m_UDomain=pCurve->GetDomain();
    }
}
