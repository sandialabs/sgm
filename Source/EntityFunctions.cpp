#include "SGMEntityFunctions.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Surface.h"
#include "Topology.h"

#include <set>
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
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"%s %lu is not a top level entity and cannot be deleted",
                 typeid(*pEntity).name(), pEntity->GetID());
        rResult.SetResult(SGM::ResultTypeCannotDelete);
        std::string const sMessage(Buffer);
        rResult.SetMessage(sMessage);
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

    for (auto pEntry : sEntityToTransform)
        {
        switch(pEntry->GetType())
            {
            case SGM::FaceType:
                {
                auto pFace=(face *)pEntry;
                pFace->TransformBox(rResult, transform3D);
                pFace->TransformFacets(transform3D);
                break;
                }
            case SGM::EdgeType:
                {
                auto pEdge=(edge *)pEntry;
                pEdge->TransformBox(rResult, transform3D);
                pEdge->TransformFacets(transform3D);
                break;
                }
            case SGM::VertexType:
                {
                auto pVertex=(vertex *)pEntry;
                pVertex->TransformBox(rResult, transform3D);
                pVertex->TransformData(transform3D);
                break;
                }
            case SGM::SurfaceType:
                {
                auto pSurface=(surface *)pEntry;
                pSurface->Transform(rResult,transform3D);
                break;
                }
            case SGM::CurveType:
                {
                auto pCurve=(curve *)pEntry;
                pCurve->Transform(rResult,transform3D);
                break;
                }
            case SGM::ComplexType:
                {
                auto pComplex=(complex *)pEntry;
                pComplex->TransformBox(rResult, transform3D);
                pComplex->Transform(transform3D);
                break;
                }
            case SGM::BodyType:
                {
                auto pBody = (body *)pEntry;
                pBody->TransformBox(rResult, transform3D);
                break;
                }
            case SGM::VolumeType:
                {
                auto pVolume= (volume *)pEntry;
                pVolume->TransformBox(rResult, transform3D);
                break;
                }
            case SGM::ThingType:
                {
                auto pThing = (thing *)pEntry;
                pThing->TransformBox(rResult, transform3D);
                break;
                }
            default:
                {
                throw std::logic_error("Unhandled entity type in TransformEntity");
                break;
                }
            }
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