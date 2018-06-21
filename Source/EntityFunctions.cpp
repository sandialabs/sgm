#include "SGMEntityFunctions.h"

#include "EntityClasses.h"
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
    for (auto pChild : sChildren) // access by value, the type of pChild is entity*
        pThing->DeleteEntity(pChild);
    pThing->DeleteEntity(pEntity);
    }

entity *CopyEntity(SGM::Result &,//rResult,
                   entity      *)//pEntity)
    {
    return nullptr;
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
          std::vector<entity *> &aEntities)
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

} // End of SGMInternal namespace