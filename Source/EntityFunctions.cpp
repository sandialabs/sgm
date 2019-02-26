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

/*
void Heal(SGM::Result           &rResult,
          std::vector<entity *> &aEntities,
          HealOptions     const &Options)
    {
    if(Options.m_bRemoveZeroFacetFaces)
        {
        size_t nEntities=aEntities.size();
        size_t Index1;
        for(Index1=0;Index1<nEntities;++Index1)
            {
            entity *pEntity=aEntities[Index1];
            std::set<face *,EntityCompare> sFaces;
            FindFaces(rResult,pEntity,sFaces);
            std::set<face *,EntityCompare>::iterator FaceIter=sFaces.begin();
            while(FaceIter!=sFaces.end())
                {
                face *pFace=*FaceIter;
                if(pFace->GetTriangles(rResult).empty())
                    {
                    RemoveFace(rResult,pFace);
                    }
                ++FaceIter;
                }
            }
        }
    }
*/
} // End of SGMInternal namespace