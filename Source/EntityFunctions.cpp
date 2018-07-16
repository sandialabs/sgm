#include "SGMEntityFunctions.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Surface.h"
#include "Topology.h"

#include <set>

namespace SGMInternal
{

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
    {
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
            face *pFace=(face *)pEntity;
            for (auto pChild : sChildren) 
                {
                switch(pChild->GetType())
                    {
                    case SGM::EdgeType:
                        {
                        edge *pEdge=(edge *)pChild;
                        if(pEdge->GetFaces().size()==1)
                            {
                            aDelete.push_back(pChild);
                            }
                        break;
                        }
                    case SGM::VertexType:
                        {
                        vertex *pVertex=(vertex *)pChild;
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
                        curve *pCurve=(curve *)pChild;
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
                        surface *pSurface=(surface *)pChild;
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
    std::set<entity *,EntityCompare> aChildern;
    pEntity->FindAllChildren(aChildern);
    entity *pAnswer=pEntity->MakeCopy(rResult);
    std::map<entity *,entity *> mCopyMap;
    mCopyMap[pEntity]=pAnswer;
    for(auto pChild : aChildern)
        {
        mCopyMap[pChild]=pChild->MakeCopy(rResult);
        }
    aChildern.insert(pEntity);
    for(auto pChild : aChildern)
        {
        mCopyMap[pChild]->ReplacePointers(mCopyMap);
        }
    return pAnswer;
    }

void TransformEntity(SGM::Result            &,//rResult,
                     SGM::Transform3D const &Trans,
                     entity                 *pEntity)
    {
    std::set<entity *,EntityCompare> sChildren;
    pEntity->FindAllChildren(sChildren);
    sChildren.insert(pEntity);
    std::set<entity *,EntityCompare>::iterator iter=sChildren.begin();
    while(iter!=sChildren.end())
        {
        entity *pChildEntity=*iter;
        pChildEntity->TransformBox(Trans);
        switch(pChildEntity->GetType())
            {
            case SGM::FaceType:
                {
                face *pFace=(face *)pChildEntity;
                pFace->TransformFacets(Trans);
                break;
                }
            case SGM::EdgeType:
                {
                edge *pEdge=(edge *)pChildEntity;
                pEdge->TransformFacets(Trans);
                break;
                }
            case SGM::VertexType:
                {
                vertex *pVertex=(vertex *)pChildEntity;
                pVertex->TransformData(Trans);
                break;
                }
            case SGM::SurfaceType:
                {
                surface *pSurface=(surface *)pChildEntity;
                pSurface->Transform(Trans);
                break;
                }
            case SGM::CurveType:
                {
                curve *pCurve=(curve *)pChildEntity;
                pCurve->Transform(Trans);
                break;
                }
            case SGM::ComplexType:
                {
                complex *pComplex=(complex *)pChildEntity;
                pComplex->Transform(Trans);
                break;
                }
            default:
                break;
            }
        ++iter;
        }
    }

SGM::Interval3D const &GetBoundingBox(SGM::Result  &rResult,
                                      entity const *pEntity)
    {
    return pEntity->GetBox(rResult);
    }

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

} // End of SGMInternal namespace