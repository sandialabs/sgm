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

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
    {
    // if this entity still has parents, don't delete it
    std::set<entity *, EntityCompare> sParents;
    pEntity->GetParents(sParents);

    if (!sParents.empty())
    {
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"%s %lu cannot be deleted because it has parents.",
                 typeid(*pEntity).name(), pEntity->GetID());
        rResult.SetResult(SGM::ResultTypeDeleteWillCorruptModel);
        std::string const sMessage(Buffer);
        rResult.SetMessage(sMessage);
        return;
    }

    auto pThing = rResult.GetThing();
    std::set<entity *,EntityCompare> sChildren;
    pEntity->FindAllChildren(sChildren);
    std::vector<entity *> aDelete;
    aDelete.push_back(pEntity);

    // Remove pEntity from its parent and remove all non-shaired childern.
    switch(pEntity->GetType())
        {
        case SGM::FaceType:
            {
            auto pFace=(face *)pEntity;
            for (auto pChild : sChildren) 
                {
                switch(pChild->GetType())
                    {
                    case SGM::EdgeType:
                        {
                        auto pEdge=(edge *)pChild;
                        if(pEdge->GetFaces().size()==1)
                            {
                            aDelete.push_back(pChild);
                            }
                        break;
                        }
                    case SGM::VertexType:
                        {
                        auto *pVertex=(vertex *)pChild;
                        std::set<face *,EntityCompare> sFaces;
                        FindFaces(rResult,pVertex,sFaces);
                        if(sFaces.size()==1)
                            {
                            aDelete.push_back(pChild);
                            }
                        break;
                        }
                    case SGM::CurveType:
                        {
                        auto *pCurve=(curve *)pChild;
                        std::set<face *,EntityCompare> sFaces;
                        FindFaces(rResult,pCurve,sFaces);
                        if(sFaces.size()==1)
                            {
                            aDelete.push_back(pChild);
                            }
                        break;
                        }
                    case SGM::SurfaceType:
                        {
                        auto *pSurface=(surface *)pChild;
                        std::set<face *,EntityCompare> sFaces;
                        FindFaces(rResult,pSurface,sFaces);
                        if(sFaces.size()==1)
                            {
                            aDelete.push_back(pChild);
                            }
                        break;
                        }
                    default:
                        {
                        aDelete.push_back(pChild);
                        }
                    }
                }

            // Check to see if the faces sides need to be changed.

            if(pFace->GetSides()==1)
                {
                std::set<face *,EntityCompare> sFaces;
                FindFacesOfCell(rResult,pFace,sFaces);
                for(auto pCellFace : sFaces)
                    {
                    pCellFace->SetSides(pCellFace->GetSides()+1);
                    }
                }
            break;
            }
        default:
            {
            for (auto pChild : sChildren) 
                {
                pChild->SeverRelations(rResult);
                aDelete.push_back(pChild);
                }
            break;
            }
        }

    for(auto pDelete : aDelete)
        {
        pDelete->SeverRelations(rResult);
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

void MakeClonesForIndependence(SGM::Result &rResult,
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
        MakeClonesForIndependence(rResult, sFamily, mIndependentCopies);

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
                pOriginal->RemoveParentsInSet(sFamily);
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