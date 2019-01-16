#include "SGMEntityFunctions.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Surface.h"
#include "Topology.h"

#include <set>
#include <SGMTransform.h>
//#include <cstdio>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

void UnhookEntity(SGM::Result &rResult,
                  entity      *pEntity,
                  std::set<entity *,EntityCompare> &sUnhooked)
{
    std::set<entity *,EntityCompare> sAllChildren;
    pEntity->FindAllChildren(sAllChildren);

    // find children shared with another parents
    std::set<entity *,EntityCompare> sSharedChildren;
    for (auto pChild : sAllChildren)
    {
        std::set<entity *, EntityCompare> sParents;
        pChild->GetParents(sParents);

        for (auto pParent : sParents)
        {
            if ((pParent != pEntity) && (sAllChildren.find(pParent) == sAllChildren.end()))
            {
                sSharedChildren.emplace(pChild);
                break;
            }
        }
    }

    // make sure we include all children of a shared child
    for (auto pShared : sSharedChildren)
    {
        std::set<entity *,EntityCompare> sGrandChildren;
        pShared->FindAllChildren(sGrandChildren);

        for (auto pChild : sGrandChildren)
        {
            sSharedChildren.emplace(pChild);
        }
    }

    // make list of non-shared children to unhook
    // set difference sUnhooked = sAllChildren - sSharedChildren
    std::set_difference(sAllChildren.begin(), sAllChildren.end(), sSharedChildren.begin(), sSharedChildren.end(), 
                        std::inserter(sUnhooked, sUnhooked.begin()), EntityCompare());

    // unhook shared children from entities to unhook
    sUnhooked.emplace(pEntity);
    for (auto pShared : sSharedChildren)
    {
        pShared->RemoveParentsInSet(rResult, sUnhooked);
    }
    pEntity->RemoveParents(rResult);
}

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
    {
    //// if this entity still has parents, don't delete it
    //std::set<entity *, EntityCompare> sParents;
    //pEntity->GetParents(sParents);

    //if (sParents.size())
    //{
    //    char Buffer[1000];
    //    snprintf(Buffer,sizeof(Buffer),"%s %lu cannot be deleted because it has parents.",
    //             typeid(*pEntity).name(), pEntity->GetID());
    //    rResult.SetResult(SGM::ResultTypeDeleteWillCorruptModel);
    //    std::string const sMessage(Buffer);
    //    rResult.SetMessage(sMessage);
    //    return;
    //}

    std::set<entity *, EntityCompare> sUnhooked;
    UnhookEntity(rResult, pEntity, sUnhooked);

    auto pThing = rResult.GetThing();
    for(auto pDelete : sUnhooked)
        {
        pDelete->SeverRelations(rResult);
        }
    for(auto pDelete : sUnhooked)
        {
        pThing->DeleteEntity(pDelete);
        }

    //auto pThing = rResult.GetThing();
    //std::set<entity *,EntityCompare> sChildren;
    //pEntity->FindAllChildren(sChildren);
    //std::vector<entity *> aDelete;
    //aDelete.push_back(pEntity);

    //// Remove pEntity from its parent and remove all non-shaired childern.
    //switch(pEntity->GetType())
    //    {
    //    case SGM::FaceType:
    //        {
    //        face *pFace=(face *)pEntity;
    //        for (auto pChild : sChildren) 
    //            {
    //            switch(pChild->GetType())
    //                {
    //                case SGM::EdgeType:
    //                    {
    //                    edge *pEdge=(edge *)pChild;
    //                    if(pEdge->GetFaces().size()==1)
    //                        {
    //                        aDelete.push_back(pChild);
    //                        }
    //                    break;
    //                    }
    //                case SGM::VertexType:
    //                    {
    //                    vertex *pVertex=(vertex *)pChild;
    //                    std::set<face *,EntityCompare> sFaces;
    //                    FindFaces(rResult,pVertex,sFaces);
    //                    if(sFaces.size()==1)
    //                        {
    //                        aDelete.push_back(pChild);
    //                        }
    //                    break;
    //                    }
    //                case SGM::CurveType:
    //                    {
    //                    curve *pCurve=(curve *)pChild;
    //                    std::set<face *,EntityCompare> sFaces;
    //                    FindFaces(rResult,pCurve,sFaces);
    //                    if(sFaces.size()==1)
    //                        {
    //                        aDelete.push_back(pChild);
    //                        }
    //                    break;
    //                    }
    //                case SGM::SurfaceType:
    //                    {
    //                    surface *pSurface=(surface *)pChild;
    //                    std::set<face *,EntityCompare> sFaces;
    //                    FindFaces(rResult,pSurface,sFaces);
    //                    if(sFaces.size()==1)
    //                        {
    //                        aDelete.push_back(pChild);
    //                        }
    //                    break;
    //                    }
    //                default:
    //                    {
    //                    aDelete.push_back(pChild);
    //                    }
    //                }
    //            }

    //        // Check to see if the faces sides need to be changed.

    //        if(pFace->GetSides()==1)
    //            {
    //            std::set<face *,EntityCompare> sFaces;
    //            FindFacesOfCell(rResult,pFace,sFaces);
    //            for(auto pCellFace : sFaces)
    //                {
    //                pCellFace->SetSides(pCellFace->GetSides()+1);
    //                }
    //            }
    //        break;
    //        }
    //    default:
    //        {
    //        for (auto pChild : sChildren) 
    //            {
    //            pChild->SeverRelations(rResult);
    //            aDelete.push_back(pChild);
    //            }
    //        break;
    //        }
    //    }

    //for(auto pDelete : aDelete)
    //    {
    //    pDelete->SeverRelations(rResult);
    //    pThing->DeleteEntity(pDelete);
    //    }
    }

entity *CopyEntity(SGM::Result &rResult,
                   entity      *pEntity)
    {
    std::set<entity *,EntityCompare> aChildren;
    pEntity->FindAllChildren(aChildren);
    entity *pAnswer=pEntity->Clone(rResult);
    std::map<entity *,entity *> mCopyMap;
    mCopyMap[pEntity]=pAnswer;
    for(auto pChild : aChildren)
        {
        mCopyMap[pChild]=pChild->Clone(rResult);
        }
    aChildren.insert(pEntity);
    for(auto pChild : aChildren)
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