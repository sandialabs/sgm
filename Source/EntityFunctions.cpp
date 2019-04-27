#include "SGMEntityFunctions.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Surface.h"
#include "Topology.h"

#include <set>
#include <SGMTransform.h>
#include <sstream>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

void FindChildrenToDeleteAndToKeep(entity                      const *pEntity,
                                   std::set<entity *, EntityCompare> &aToDelete,
                                   std::set<entity *, EntityCompare> &aToKeep)
{
    std::set<entity *,EntityCompare> sAllChildren;
    pEntity->FindAllChildren(sAllChildren);

    // find children shared with another parents
    std::set<entity *,EntityCompare> sSharedChildren;
    for (auto pChild : sAllChildren)
    {
        std::set<entity *, EntityCompare> sParents;
        pChild->GetParents(sParents);

        bool bOtherParents = false;
        for (auto pParent : sParents)
        {
            if ((pParent != pEntity) && (sAllChildren.find(pParent) == sAllChildren.end()))
            {
                bOtherParents = true;
                aToKeep.emplace(pChild);
                break;
            }
        }
        if (!bOtherParents)
        {
            aToDelete.emplace(pChild);
        }
    }
}

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
{
    // if this entity still has parents, don't delete it
    if (!pEntity->IsTopLevel())
    {
		std::stringstream ss;
		ss << typeid(*pEntity).name() << " " << pEntity->GetID() << " is not a top level entity and cannot be deleted";
        rResult.SetResult(SGM::ResultTypeCannotDelete);
		rResult.SetMessage(ss.str());
        return;
    }

    std::set<entity *, EntityCompare> sEntitiesToDelete;
    std::set<entity *, EntityCompare> sEntitiesToKeep;
    FindChildrenToDeleteAndToKeep(pEntity, sEntitiesToDelete, sEntitiesToKeep);
    sEntitiesToDelete.emplace(pEntity);

    // unhook entities to keep from any entities being deleted
    for (auto pToKeep : sEntitiesToKeep)
    {
        pToKeep->RemoveParentsInSet(rResult, sEntitiesToDelete);
    }

    for(auto pDelete : sEntitiesToDelete)
    {
        pDelete->SeverRelations(rResult);
    }
    auto pThing = rResult.GetThing();
    for(auto pDelete : sEntitiesToDelete)
    {
        pThing->DeleteEntity(pDelete);
    }
}

entity *CopyEntity(SGM::Result &rResult,
                   entity      *pEntity)
    {
    std::set<entity *,EntityCompare> sChildren;
    pEntity->FindAllChildren(sChildren);

    entity *pAnswer=pEntity->Clone(rResult);
    std::map<entity *,entity *> mCopyMap;
    mCopyMap[pEntity]=pAnswer;
    for(auto pChild : sChildren)
        {
        mCopyMap[pChild]=pChild->Clone(rResult);
        }

    sChildren.insert(pEntity);
    for(auto pChild : sChildren)
        {
        mCopyMap[pChild]->ReplacePointers(mCopyMap);
        }

    return pAnswer;
    }

void CloneSharedChildren(SGM::Result &rResult,
                         std::set<entity *,EntityCompare> const &sFamily,
                         std::map<entity *,entity *> &mIndependentCopies)
{
    for (auto pEntity : sFamily)
    {
        std::set<entity *, EntityCompare> sParents; 
        pEntity->GetParents(sParents);

        bool bIndependent = true;
        for (auto pParent : sParents)
        {
            if (sFamily.find(pParent) == sFamily.end())
            {
              bIndependent = false;
              break;
            }
        }

        if (bIndependent)
        {
            mIndependentCopies[pEntity]=pEntity;
        }
        else
        {
            mIndependentCopies[pEntity]=pEntity->Clone(rResult);
        }
    }
}

//struct FacePointsVisitor : EntityVisitor
//    {
//    FacePointsVisitor() = delete;
//
//    explicit FacePointsVisitor(SGM::Result &rResult) : EntityVisitor(rResult)
//        {}
//
//    inline void Visit(face &f) override
//        { FindFacePointsData(*pResult, &f); }
//    };


struct TransformVisitor: EntityVisitor
    {
    SGM::Transform3D m_Transform3D;
    
    TransformVisitor() = delete;
    
    explicit TransformVisitor(SGM::Result &rResult,
                              SGM::Transform3D const &transform3D) : 
        EntityVisitor(rResult), m_Transform3D(transform3D)
        {}

    inline void Visit(thing &t)    override { t.TransformBox(*pResult, m_Transform3D); }
    //inline void Visit(assembly &)  override { /* TODO: handle assembly transform */ }
    //inline void Visit(attribute &) override { /* do nothing */ }
    inline void Visit(body &b)     override { b.TransformBox(*pResult, m_Transform3D); }
    inline void Visit(complex &c)  override { c.TransformBox(*pResult, m_Transform3D); c.Transform(m_Transform3D); }

    inline void Visit(edge &e)   override { e.TransformBox(*pResult, m_Transform3D); e.TransformFacets(m_Transform3D); }
    inline void Visit(face &f)   override { f.TransformBox(*pResult, m_Transform3D); f.TransformFacets(m_Transform3D); }
    inline void Visit(vertex &v) override { v.TransformBox(*pResult, m_Transform3D); v.TransformData(m_Transform3D); }
    inline void Visit(volume &v) override { v.TransformBox(*pResult, m_Transform3D); }

    inline void Visit(line &c)        override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(circle &c)      override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(ellipse &c)     override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(hyperbola &c)   override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(parabola &c)    override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(hermite &c)     override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(NUBcurve &c)    override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(NURBcurve &c)   override { c.Transform(*pResult, m_Transform3D); }
    inline void Visit(PointCurve &c)  override { c.Transform(*pResult, m_Transform3D); }
    
    inline void Visit(TorusKnot &s)   override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(plane &s)       override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(cylinder &s)    override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(cone &s)        override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(sphere &s)      override { s.Transform(*pResult, m_Transform3D); } 
    inline void Visit(torus &s)       override { s.Transform(*pResult, m_Transform3D); } 
    inline void Visit(revolve &s)     override { s.Transform(*pResult, m_Transform3D); } 
    inline void Visit(extrude &s)     override { s.Transform(*pResult, m_Transform3D); } 
    inline void Visit(offset &s)      override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(NUBsurface &s)  override { s.Transform(*pResult, m_Transform3D); }
    inline void Visit(NURBsurface &s) override { s.Transform(*pResult, m_Transform3D); }
    };

void TransformEntity(SGM::Result            &rResult,
                     SGM::Transform3D const &transform3D,
                     entity                 *pEntity)
    {
    std::set<entity *,EntityCompare> sEntityToTransform;
    if (pEntity->GetType() == SGM::ThingType)
    {
        pEntity->FindAllChildren(sEntityToTransform);
        sEntityToTransform.emplace(pEntity);
    }
    else
    {
        if (!pEntity->IsTopLevel())
        {
			std::stringstream ss;
			ss << typeid(*pEntity).name() << " " << pEntity->GetID() << " is not a top level entity and cannot be transformed";
			rResult.SetResult(SGM::ResultTypeCannotTransform);
			rResult.SetMessage(ss.str());
			return;
        }

        
        std::set<entity *,EntityCompare> sFamily;
        pEntity->FindAllChildren(sFamily);
        sFamily.insert(pEntity);

        std::map<entity *,entity *> mIndependentCopies;
        CloneSharedChildren(rResult, sFamily, mIndependentCopies);

        for(auto pEntry : mIndependentCopies)
        {
            // this needs to happen whether an entry was cloned or not
            pEntry.second->ReplacePointers(mIndependentCopies);
        }

        for(auto pEntry : mIndependentCopies)
        {
            // make original entity that was cloned independent of anyone being transformed
            entity *pOriginal = pEntry.first;
            entity *pClone = pEntry.second;
            if (pOriginal != pClone)
            {
                pOriginal->RemoveParentsInSet(rResult, sFamily);
            }
        }

        for(auto pEntry : mIndependentCopies)
        {
            entity *pIndependent = pEntry.second;
            sEntityToTransform.emplace(pIndependent);
        }
    }

    TransformVisitor transformVisitor(rResult, transform3D);

    for (auto pEntry : sEntityToTransform)
        {
        pEntry->Accept(transformVisitor);
        }
    }

surface *SimplifySurface(SGM::Result       &rResult,
                         surface           *pSurface,
                         HealOptions const &Options)

    {
    if(pSurface->GetSurfaceType()==SGM::TorusType)
        {
        torus *pTorus=(torus *)pSurface;
        if(Options.m_bRepairApples && pTorus->GetKind()==SGM::TorusKindType::AppleType)
            {
            if(pTorus->m_dMajorRadius/pTorus->m_dMinorRadius<0.01)
                {
                return new sphere(rResult,pTorus->m_Center,pTorus->m_dMajorRadius+pTorus->m_dMinorRadius,&pTorus->m_XAxis,&pTorus->m_YAxis);
                }
            }
        }
    else if(pSurface->GetSurfaceType()==SGM::NURBSurfaceType)
        {
        NURBsurface *pNURB=(NURBsurface *)pSurface;

        // Check to see if this NURB is close to a plane.

        std::vector<SGM::Point3D> aPoints;
        auto aControlPoints=pNURB->GetControlPoints();
        aPoints.reserve(aControlPoints.size()*aControlPoints[0].size());
        for(auto Row : aControlPoints)
            {
            for(SGM::Point4D Pos : Row)
                {
                aPoints.push_back(SGM::Point3D(Pos.m_x,Pos.m_y,Pos.m_z));
                }
            }
        SGM::Point3D Origin;
        SGM::UnitVector3D XVec,YVec,ZVec;
        SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec);
        double dError=0;
        for(SGM::Point3D const &Pos : aPoints)
            {
            double dDist=fabs((Pos-Origin)%ZVec);
            if(dError<dDist)
                {
                dError=dDist;
                }
            }
        SGM::Point2D uv=pSurface->GetDomain().MidPoint();
        if(dError<SGM_FIT)
            {
            SGM::UnitVector3D Norm;
            pSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
            if(Norm%ZVec<0)
                {
                ZVec.Negate();
                }
            return new plane(rResult,Origin,ZVec);
            }
        }
    return nullptr;
    }

void RemoveSliver(SGM::Result &rResult,
                  face        *pFace)
    {
    std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
    if(sEdges.size()==2)
        {
        auto iter=sEdges.begin();
        edge *pEdge0=*iter;
        ++iter;
        edge *pEdge1=*iter;
        
        auto iter0=pEdge0->GetFaces().begin();
        face *pFace0=*iter0;
        if(pFace0==pFace)
            {
            ++iter0;
            pFace0=*iter0;
            }

        auto iter1=pEdge1->GetFaces().begin();
        face *pFace1=*iter1;
        if(pFace1==pFace)
            {
            ++iter1;
            pFace1=*iter1;
            }

        // Keep pEdge0.

        auto nType=pFace1->GetSideType(pEdge1);
        SGM::Vector3D Vec0,Vec1;
        SGM::Point3D Pos=pEdge0->FindMidPoint();
        double t0=pEdge0->GetCurve()->Inverse(Pos);
        pEdge0->GetCurve()->Evaluate(t0,nullptr,&Vec0);
        double t1=pEdge1->GetCurve()->Inverse(Pos);
        pEdge1->GetCurve()->Evaluate(t1,nullptr,&Vec1);
        if(Vec0%Vec1<0)
            {
            if(nType==SGM::FaceOnLeftType)
                {
                nType=SGM::FaceOnRightType;
                }
            else
                {
                nType=SGM::FaceOnLeftType;
                }
            }
        pFace1->AddEdge(rResult,pEdge0,nType);
        curve *pCurve1=pEdge1->GetCurve();
        pEdge1->SeverRelations(rResult);
        rResult.GetThing()->DeleteEntity(pEdge1);
        if(pCurve1->GetEdges().empty())
            {
            rResult.GetThing()->DeleteEntity(pCurve1);
            }
        surface *pSurf=pFace->GetSurface();
        pFace->SeverRelations(rResult);
        rResult.GetThing()->DeleteEntity(pFace);
        if(pSurf->GetFaces().empty())
            {
            rResult.GetThing()->DeleteEntity(pSurf);
            }
        }
    }

void Heal(SGM::Result           &rResult,
          std::vector<entity *> &aEntities,
          HealOptions     const &Options)
    {
    if(Options.m_bSimplifySurfaces)
        {
        for(auto pEntity : aEntities)
            {
            std::set<surface *,EntityCompare> sSurfaces;
            FindSurfaces(rResult,pEntity,sSurfaces);
            for(auto pSurface : sSurfaces)
                {
                if(surface *pSimplify=SimplifySurface(rResult,pSurface,Options))
                    {
                    if(pSimplify->GetSurfaceType()==SGM::PlaneType)
                        { 
                        std::cout << " Simplified from NURB to Plane " << pSurface->GetID() << "\n";
                        }
                    else if(pSimplify->GetSurfaceType()==SGM::SphereType)
                        {
                        std::cout << " Simplified from Torus to Sphere " << pSurface->GetID() << "\n";
                        }
                    std::set<face *,EntityCompare> sFaces=pSurface->GetFaces();
                    for(auto *pFace : sFaces)
                        {
                        pFace->ClearFacets(rResult);
                        for(edge *pEdge : pFace->GetEdges())
                            {
                            pFace->ClearUVBoundary(pEdge);
                            }
                        pFace->SetSurface(rResult,pSimplify);
                        }
                    rResult.GetThing()->DeleteEntity(pSurface);
                    }
                }
            }
        }
    if(Options.m_bReparamNURBS)
        {
        std::vector<size_t> aScales;
        aScales.assign(6,0);
        for(auto pEntity : aEntities)
            {
            std::set<surface *,EntityCompare> sSurfaces;
            FindSurfaces(rResult,pEntity,sSurfaces);
            for(auto pSurface : sSurfaces)
                {
                if(pSurface->GetSurfaceType()==SGM::NUBSurfaceType)
                    {
                    NUBsurface *pNUB=(NUBsurface *)pSurface;
                    double dScale=pNUB->ReParam(rResult);
                    if(1000000<dScale)
                        {
                        ++aScales[5];
                        }
                    else if(100000<dScale)
                        {
                        ++aScales[4];
                        }
                    else if(10000<dScale)
                        {
                        ++aScales[3];
                        }
                    else if(1000<dScale)
                        {
                        ++aScales[2];
                        }
                    else if(100<dScale)
                        {
                        ++aScales[1];
                        }
                    else if(10<dScale)
                        {
                        ++aScales[0];
                        }
                    }
                else if(pSurface->GetSurfaceType()==SGM::NURBSurfaceType)
                    {
                    NURBsurface *pNURB=(NURBsurface *)pSurface;
                    double dScale=pNURB->ReParam(rResult);
                    if(1000000<dScale)
                        {
                        ++aScales[5];
                        }
                    else if(100000<dScale)
                        {
                        ++aScales[4];
                        }
                    else if(10000<dScale)
                        {
                        ++aScales[3];
                        }
                    else if(1000<dScale)
                        {
                        ++aScales[2];
                        }
                    else if(100<dScale)
                        {
                        ++aScales[1];
                        }
                    else if(10<dScale)
                        {
                        ++aScales[0];
                        }
                    }
                }
            }
        if(aScales[5])
            {
            std::cout << " Scaled NURB by more than 1,000,000 " << aScales[5] << "\n";
            }
        if(aScales[4])
            {
            std::cout << " Scaled NURB by more than 100,000 " << aScales[4] << "\n";
            }
        if(aScales[3])
            {
            std::cout << " Scaled NURB by more than 10,000 " << aScales[3] << "\n";
            }
        if(aScales[2])
            {
            std::cout << " Scaled NURB by more than 1,000 " << aScales[2] << "\n";
            }
        if(aScales[1])
            {
            std::cout << " Scaled NURB by more than 100 " << aScales[1] << "\n";
            }
        if(aScales[0])
            {
            std::cout << " Scaled NURB by more than 10 " << aScales[0] << "\n";
            }
        }
    if(Options.m_bRemoveSlivers)
        {
        for(auto pEntity : aEntities)
            {
            std::set<face *,EntityCompare> sFaces;
            FindFaces(rResult,pEntity,sFaces);
            for(auto pFace : sFaces)
                {
                if(pFace->IsSliver())
                    {
                    std::cout << " Removed Sliver " << pFace->GetID() << "\n";
                    RemoveSliver(rResult,pFace);
                    }
                }
            }
        }
    if(Options.m_bSnapVertices)
        {
        for(auto pEntity : aEntities)
            {
            std::set<vertex *,EntityCompare> sVertices;
            FindVertices(rResult,pEntity,sVertices);
            for(auto pVertex : sVertices)
                {
                pVertex->Snap(rResult);
                }
            }
        }
    }

} // End of SGMInternal namespace