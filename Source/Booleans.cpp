
#include "SGMVector.h"
#include "SGMTransform.h"
#include "SGMModify.h"
#include "SGMGraph.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include "Intersectors.h"
#include "Modify.h"
#include "Primitive.h"
#include "Topology.h"
#include "Faceter.h"

#include <limits>
#include <algorithm>
#include <map>

namespace SGMInternal
{

void ReduceToVolumes(SGM::Result                      &rResult,
                     body                             *pBody,
                     std::set<volume *,EntityCompare> &sVolumes)
    {
    FindVolumes(rResult,pBody,sVolumes);
    pBody->SeverRelations(rResult);
    rResult.GetThing()->DeleteEntity(pBody);
    }

vertex *ImprintPointOnEdge(SGM::Result        &rResult,
                           SGM::Point3D const &Pos,
                           edge               *pEdge)
    {
    // Create the new vertex.

    std::set<face *,EntityCompare> sFaces=pEdge->GetFaces();
    double t=pEdge->GetCurve()->Inverse(Pos);
    auto pAnswer=new vertex(rResult, Pos);
    auto pEnd=pEdge->GetEnd();

    // Split open edges at the new vertex and 
    // closed edge with a vertex at the new vertex

    if(pEnd==nullptr)
        {
        pEdge->SetStart(pAnswer);
        pEdge->SetEnd(pAnswer);

        SGM::Interval1D Domain(t,pEdge->GetCurve()->GetDomain().Length()+t);
        pEdge->SetDomain(rResult,Domain);
        }
    else
        {
        auto pNewEdge=new edge(rResult);
        pNewEdge->SetCurve(pEdge->GetCurve());
    
        SGM::Interval1D Domain=pEdge->GetDomain();
        pNewEdge->SetEnd(pEnd);
        pNewEdge->SetStart(pAnswer);
        pEdge->SetEnd(pAnswer);
        if(pEdge->GetStart()==pEnd)
            {
            pEdge->GetStart()->AddEdge(pEdge);
            }

        SGM::Interval1D Domain1(Domain.m_dMin,t),Domain2(t,Domain.m_dMax);
        pEdge->SetDomain(rResult,Domain1);
        pNewEdge->SetDomain(rResult,Domain2);

        // Add the pNewEdge to the pface.

        for(face *pFace : sFaces)
            {
            SGM::EdgeSideType nType=pFace->GetSideType(pEdge);
            pFace->AddEdge(rResult,pNewEdge,nType);
            }
        }

    return pAnswer;
    }

vertex *ImprintPoint(SGM::Result        &rResult,
                     SGM::Point3D const &Pos,
                     topology           *pTopology)
    {
    vertex *pAnswer=nullptr;
    if(pTopology->GetType()==SGM::EntityType::EdgeType)
        {
        auto pEdge=(edge *)pTopology;
        pAnswer=ImprintPointOnEdge(rResult,Pos,pEdge);
        }
    return pAnswer;
    }

void TrimCurveWithFaces(SGM::Result               &rResult,
                        curve                     *pCurve,
                        face                const *pFace0,
                        face                const *pFace1,
                        std::vector<edge *>       &aEdges,
                        std::vector<SGM::Point3D> &aPoints,
                        std::map<double,entity *> &mHitMap0,
                        std::map<double,entity *> &mHitMap1,
                        edge                const *pLimitEdge)
    {
    // Find the intersection(s) of given curve and each of the edges of the pFace.
    // TODO: Note the tolerances need to be considered and that the param space
    // of the face should be taken into acount to find crossing in 2D instead of 3D.

    std::vector<SGM::Point3D> aHits;
    double dTolerance=SGM_MIN_TOL;
    double dTolSquared=dTolerance*dTolerance;
    std::set<edge *,EntityCompare> const &sEdges0=pFace0->GetEdges();
    for(edge *pEdge : sEdges0)
        {
        std::vector<SGM::Point3D> aIntersectionPoints;
        std::vector<SGM::IntersectionType> aTypes;
        IntersectCurves(rResult,pCurve,pEdge->GetCurve(),aIntersectionPoints,aTypes,dTolerance);
        for(SGM::Point3D Pos : aIntersectionPoints)
            {
            if(pLimitEdge && !pLimitEdge->PointInEdge(Pos, dTolerance))
                {
                continue;
                }
            aHits.push_back(Pos);
            double dParam=pCurve->Inverse(Pos);
            if(pEdge->GetStart() && Pos.DistanceSquared(pEdge->GetStart()->GetPoint())<dTolSquared)
                {
                mHitMap0[dParam]=pEdge->GetStart();
                }
            else if(pEdge->GetEnd() && Pos.DistanceSquared(pEdge->GetEnd()->GetPoint())<dTolSquared)
                {
                mHitMap0[dParam]=pEdge->GetEnd();
                }
            else
                {
                mHitMap0[dParam]=pEdge;
                }
            }
        }
    if(pFace1)
        {
        std::set<edge *,EntityCompare> const &sEdges1=pFace1->GetEdges();
        for(edge *pEdge : sEdges1)
            {
            std::vector<SGM::Point3D> aIntersectionPoints;
            std::vector<SGM::IntersectionType> aTypes;
            IntersectCurves(rResult,pCurve,pEdge->GetCurve(),aIntersectionPoints,aTypes,dTolerance);
            for(SGM::Point3D Pos : aIntersectionPoints)
                {
                aHits.push_back(Pos);
                double dParam=pCurve->Inverse(Pos);
                if(pEdge->GetStart() && Pos.DistanceSquared(pEdge->GetStart()->GetPoint())<dTolSquared)
                    {
                    mHitMap1[dParam]=pEdge->GetStart();
                    }
                else if(pEdge->GetEnd() && Pos.DistanceSquared(pEdge->GetEnd()->GetPoint())<dTolSquared)
                    {
                    mHitMap1[dParam]=pEdge->GetEnd();
                    }
                else
                    {
                    mHitMap1[dParam]=pEdge;
                    }
                }
            }
        }

    // Add the end points of the curve and remove duplicates.

    SGM::Interval3D Box=pFace0->GetBox(rResult);
    if(pFace1)
        {
        Box&=pFace1->GetBox(rResult);
        }
    SGM::Interval1D const &Domain=pCurve->GetDomain();
    SGM::Point3D StartPos,EndPos;
    pCurve->Evaluate(Domain.m_dMin,&StartPos);
    pCurve->Evaluate(Domain.m_dMax,&EndPos);
    aHits.push_back(StartPos);
    aHits.push_back(EndPos);
    SGM::RemoveDuplicates3D(aHits,dTolerance,&Box);

    // Find the parameters for each intersection.

    std::vector<std::pair<double,SGM::Point3D> > aHitParams;
    size_t nHits=aHits.size();
    aHitParams.reserve(nHits);
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D const &Pos=aHits[Index1];
        aHitParams.emplace_back(pCurve->Inverse(Pos),Pos);
        }
    std::sort(aHitParams.begin(),aHitParams.end());

    // Check each segment to see if it is inside or outside of the face(s).

    surface const *pSurface0=pFace0->GetSurface();
    surface const *pSurface1=pFace1 ? pFace1->GetSurface() : nullptr;
    std::vector<bool> aIsolated;
    aIsolated.assign(nHits,true);
    for(Index1=1;Index1<nHits;++Index1)
        {
        double dParam0=aHitParams[Index1-1].first;
        double dParam1=aHitParams[Index1].first;
        double dMidParam=(dParam0+dParam1)*0.5;
        SGM::Point3D Pos;
        pCurve->Evaluate(dMidParam,&Pos);
        SGM::Point2D uv0=pSurface0->Inverse(Pos);
        if(pFace0->PointInFace(rResult,uv0))
            {
            if(pFace1==nullptr)
                {
                SGM::Interval1D ParamDomain(dParam0,dParam1);
                aEdges.push_back(CreateEdge(rResult,pCurve,&ParamDomain));
                aIsolated[Index1-1]=false;
                aIsolated[Index1]=false;
                }
            else
                {
                SGM::Point2D uv1=pSurface1->Inverse(Pos);
                if(pFace1->PointInFace(rResult,uv1))
                    {
                    SGM::Interval1D ParamDomain(dParam0,dParam1);
                    aEdges.push_back(CreateEdge(rResult,pCurve,&ParamDomain));
                    aIsolated[Index1-1]=false;
                    aIsolated[Index1]=false;
                    }
                }
            }
        }

    // Check for closed curves inside the face.

    if(mHitMap0.empty() && aHits.size()==1)
        {
        aIsolated[0]=false;
        aEdges.push_back(CreateEdge(rResult,pCurve,nullptr));
        }

    // Check for isolated points.

    for(Index1=0;Index1<nHits;++Index1)
        {
        if(aIsolated[Index1])
            {
            SGM::Point3D Pos=aHitParams[Index1].second;
            SGM::Point2D uv0=pSurface0->Inverse(Pos);
            if(pFace0->PointInFace(rResult,uv0))
                {
                if(pFace1==nullptr)
                    {
                    aPoints.push_back(Pos);
                    }
                else
                    {
                    SGM::Point2D uv1=pSurface1->Inverse(Pos);
                    if(pFace1->PointInFace(rResult,uv1))
                        {
                        aPoints.push_back(Pos);
                        }
                    }
                }
            }
        }
    }

void MergeVertices(SGM::Result &rResult,
                  vertex      *pKeepVertex,
                  vertex      *pDeleteVertex)
    {
    std::set<edge *,EntityCompare> sEdges=pDeleteVertex->GetEdges();
    for(edge *pEdge : sEdges)
        {
        if(pEdge->GetStart()==pDeleteVertex)
            {
            pEdge->SetStart(pKeepVertex);
            }
        if(pEdge->GetEnd()==pDeleteVertex)
            {
            pEdge->SetEnd(pKeepVertex);
            }
        }
    rResult.GetThing()->DeleteEntity(pDeleteVertex);
    }

void MergeVertexSet(SGM::Result &rResult,
                    std::set<vertex *, EntityCompare> &sVertices)
{
    // Note that this only works on single volume wires.
    // Also note that this is a n^2 algorithum which could be n*ln(n).

    std::set<vertex *> sDeleted;
    for(vertex *pVertex:sVertices)
        {
        for(vertex *pTest:sVertices)
            {
            if( pVertex!=pTest && 
                sDeleted.find(pTest)==sDeleted.end() &&
                sDeleted.find(pVertex)==sDeleted.end() &&
                SGM::NearEqual(pVertex->GetPoint(),pTest->GetPoint(),SGM_MIN_TOL))
                {
                MergeVertices(rResult,pVertex,pTest);
                sDeleted.insert(pTest);
                }
            }
        }
}


void ImprintPeninsula(SGM::Result &rResult,
                      edge        *pEdge,
                      face        *pFace,
                      entity      *pStartEntity,
                      entity      *pEndEntity)
    {
    if(pStartEntity)
        {
        if(pStartEntity->GetType()==SGM::EdgeType)
            {
            pStartEntity=ImprintPointOnEdge(rResult,pEdge->GetStart()->GetPoint(),(edge *)pStartEntity);
            }
        MergeVertices(rResult,(vertex *)pStartEntity,pEdge->GetStart());
        }
    else 
        {
        if(pEndEntity->GetType()==SGM::EdgeType)
            {
            pEndEntity=ImprintPointOnEdge(rResult,pEdge->GetEnd()->GetPoint(),(edge *)pEndEntity);
            }
        MergeVertices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());
        }
    pFace->AddEdge(rResult,pEdge,SGM::EdgeSideType::FaceOnBothSidesType);
    }


void ImprintBridge(SGM::Result &rResult,
                   edge        *pEdge,
                   face        *pFace,
                   entity      *pStartEntity,
                   entity      *pEndEntity)
    {
    if(pStartEntity->GetType()==SGM::EdgeType)
        {
        pStartEntity=ImprintPointOnEdge(rResult,pEdge->GetStart()->GetPoint(),(edge *)pStartEntity);
        }
    MergeVertices(rResult,(vertex *)pStartEntity,pEdge->GetStart());

    if(pEndEntity->GetType()==SGM::EdgeType)
        {
        pEndEntity=ImprintPointOnEdge(rResult,pEdge->GetEnd()->GetPoint(),(edge *)pEndEntity);
        }
    MergeVertices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());

    pFace->AddEdge(rResult,pEdge,SGM::EdgeSideType::FaceOnBothSidesType);
    }

size_t FindLoop(entity                                  *pStartEntity,
                std::vector<std::vector<edge *> > const &aaLoops)
    {
    vertex *pVertex=pStartEntity->GetType()==SGM::VertexType ? (vertex *)pStartEntity : nullptr;
    edge *pEdge=pStartEntity->GetType()==SGM::EdgeType ? (edge *)pStartEntity : nullptr;
    size_t nAnswer=0;
    size_t nnLoops=aaLoops.size();
    size_t Index1,Index2;
    for(Index1=0;Index1<nnLoops;++Index1)
        {
        std::vector<edge *> const &aLoops=aaLoops[Index1];
        size_t nLoops=aLoops.size();
        for(Index2=0;Index2<nLoops;++Index2)
            {
            edge *pTestEdge=aLoops[Index2];
            if(pEdge && pEdge==pTestEdge)
                {
                return Index1;
                }
            if(pVertex && (pTestEdge->GetStart()==pVertex || pTestEdge->GetEnd()==pVertex))
                {
                return Index1;
                }
            }
        }
    return nAnswer;
    }

void SplitLoop(std::vector<edge *>            const &aLoop,
               std::vector<SGM::EdgeSideType> const &aSides,
               entity                               *pStartEntity,
               entity                               *pEndEntity,
               edge                                 *pNewEdge,
               std::vector<edge *>                  &aLoopA,
               std::vector<edge *>                  &aLoopB,
               std::vector<SGM::EdgeSideType>       &aSidesA,
               std::vector<SGM::EdgeSideType>       &aSidesB)
    {
    // Run around until pStartEntity is found.

    size_t nLoop=aLoop.size();
    size_t Index1,nStart=0,nEnd=0;
    for(Index1=0;Index1<nLoop;++Index1)
        {
        edge *pEdge=aLoop[Index1];
        vertex *pEnd=pEdge->GetEnd();
        if(aSides[Index1]==SGM::FaceOnRightType)
            {
            pEnd=pEdge->GetStart();
            }
        if(pEnd==pStartEntity)
            {
            nStart=Index1+1;
            break;
            }
        }

    // Keep going until pEndEntity is found.

    size_t nMaxEdge=nLoop*2;
    for(Index1=nStart;Index1<nMaxEdge;++Index1)
        {
        edge *pEdge=aLoop[Index1%nLoop];
        vertex *pEnd=pEdge->GetEnd();
        if(aSides[Index1%nLoop]==SGM::FaceOnRightType)
            {
            pEnd=pEdge->GetStart();
            }
        if(pEnd==pEndEntity)
            {
            nEnd=Index1+1;
            break;
            }
        }

    // Set up the two loops.

    for(Index1=nStart;Index1<nEnd;++Index1)
        {
        edge *pEdge=aLoop[Index1%nLoop];
        SGM::EdgeSideType nType=aSides[Index1%nLoop];
        aLoopA.push_back(pEdge);
        aSidesA.push_back(nType);
        }
    size_t nStart2=nEnd;
    size_t nEnd2=nStart+nLoop;
    for(Index1=nStart2;Index1<nEnd2;++Index1)
        {
        edge *pEdge=aLoop[Index1%nLoop];
        SGM::EdgeSideType nType=aSides[Index1%nLoop];
        aLoopB.push_back(pEdge);
        aSidesB.push_back(nType);
        }

    // Add the new edge to the two loops.

    aLoopA.push_back(pNewEdge);
    aSidesA.push_back(SGM::FaceOnLeftType);
    aLoopB.push_back(pNewEdge);
    aSidesB.push_back(SGM::FaceOnRightType);
    }

face *ImprintSplitter(SGM::Result &rResult,
                      edge        *pEdge,
                      face        *pFace,
                      entity      *pStartEntity,
                      entity      *pEndEntity)
    {
    // Imprint the end points.

    if(pStartEntity->GetType()==SGM::EdgeType)
        {
        pStartEntity=ImprintPointOnEdge(rResult,pEdge->GetStart()->GetPoint(),(edge *)pStartEntity);
        }
    if(pEndEntity->GetType()==SGM::EdgeType)
        {
        pEndEntity=ImprintPointOnEdge(rResult,pEdge->GetEnd()->GetPoint(),(edge *)pEndEntity);
        }
    MergeVertices(rResult,(vertex *)pStartEntity,pEdge->GetStart());
    MergeVertices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());

    // Split the loop into two loops.

    std::vector<std::vector<edge *> > aaLoops;
    std::vector<std::vector<SGM::EdgeSideType> > aaEdgeSideTypes;
    pFace->FindLoops(rResult,aaLoops,aaEdgeSideTypes);

    size_t nLoop=FindLoop(pStartEntity,aaLoops);
    std::vector<edge *> aLoopA,aLoopB;
    std::vector<SGM::EdgeSideType> aEdgeSideTypesA,aEdgeSideTypesB;
    SplitLoop(aaLoops[nLoop],aaEdgeSideTypes[nLoop],pStartEntity,pEndEntity,
              pEdge,aLoopA,aLoopB,aEdgeSideTypesA,aEdgeSideTypesB);
    pFace->AddEdge(rResult,pEdge,SGM::FaceOnBothSidesType);

    // Make a new face for the second part of the split loop.

    auto pNewFace=new face(rResult);
    pNewFace->SetSurface(pFace->GetSurface());
    pNewFace->SetSides(pFace->GetSides());
    pFace->GetVolume()->AddFace(pNewFace);
    size_t nLoopB=aLoopB.size();
    size_t Index1;
    for(Index1=0;Index1<nLoopB;++Index1)
        {
        edge *pLoopEdge=aLoopA[Index1];
        SGM::EdgeSideType nType=aEdgeSideTypesB[Index1];
        if(pEdge==pLoopEdge)
            {
            pFace->AddEdge(rResult,pLoopEdge,nType==SGM::FaceOnLeftType ? SGM::FaceOnRightType : SGM::FaceOnLeftType);
            }
        else
            {
            pFace->RemoveEdge(rResult,pLoopEdge);
            }
        pNewFace->AddEdge(rResult,pLoopEdge,nType);
        }
        
    // Move the other loops to the face of the loop that they are inside.

    return pNewFace;
    }

void ImprintLowersGenus(SGM::Result &rResult,
                        edge        *pEdge,
                        face        *pFace)
    {
    pFace->AddEdge(rResult,pEdge,SGM::FaceOnBothSidesType);
    }

face *ImprintAtoll(SGM::Result &rResult,
                   edge        *pEdge,
                   face        *pFace)
    {
    // Test to see if pEdge is clockwise in the face.
    // Create a new face and add pEdge to both pFace and pNewFace.

    auto *pNewFace=new face(rResult);
    pNewFace->SetSurface(pFace->GetSurface());
    pNewFace->SetSides(pFace->GetSides());
    pNewFace->AddEdge(rResult,pEdge,SGM::FaceOnLeftType);
    pFace->GetVolume()->AddFace(pNewFace);
    std::vector<unsigned int> aAdjacencies;
    std::vector<std::vector<unsigned int> > aaPolygons;
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<entity *> aEntities;
    FacetFaceLoops(rResult,pNewFace,aPoints2D,aPoints3D,aaPolygons,pEdge);
    double dArea=0.0;
    for(std::vector<unsigned int> const &aPolygon : aaPolygons)
        {
        dArea+=SGM::PolygonArea(PointsFromPolygon(aPoints2D,aPolygon));
        }
    if(dArea<0)
        {
        pNewFace->RemoveEdge(rResult,pEdge);
        pNewFace->AddEdge(rResult,pEdge,SGM::FaceOnRightType);
        }
    pFace->AddEdge(rResult,pEdge,pNewFace->GetSideType(pEdge)==SGM::FaceOnRightType ? SGM::FaceOnLeftType : SGM::FaceOnRightType);
    
    return pNewFace;
    }

bool LowersGenus(SGM::Result &rResult,
                 edge        *pEdge,
                 face        *pFace)
    {
    surface *pSurface=pFace->GetSurface();
    if(pSurface->ClosedInU() && pSurface->ClosedInV())
        {
        std::vector<SGM::Point3D> const &aPoints=pEdge->GetFacets(rResult);
        int nUWinds,nVWinds;
        FindWindingNumbers(pSurface,aPoints,nUWinds,nVWinds);
        if(nUWinds || nVWinds)
            {
            return true;
            }
        }
    return false;
    }

bool AreEdgesCoincident(edge const *pEdge1,
                        edge const *pEdge2)
    {
    curve const *pCurve1=pEdge1->GetCurve();
    curve const *pCurve2=pEdge2->GetCurve();
    if(pCurve1->IsSame(pCurve2,SGM_MIN_TOL))
        {
        SGM::Point3D const &Start1=pEdge1->GetStart()->GetPoint();
        SGM::Point3D const &End1=pEdge1->GetEnd()->GetPoint();
        SGM::Point3D const &Start2=pEdge2->GetStart()->GetPoint();
        SGM::Point3D const &End2=pEdge2->GetEnd()->GetPoint();
        if(Start1.Distance(Start2)<SGM_ZERO && End1.Distance(End2)<SGM_ZERO)
            {
            return true;
            }
        if(Start1.Distance(End2)<SGM_ZERO && End1.Distance(Start2)<SGM_ZERO)
            {
            return true;
            }
        }
    return false;
    }

edge *AreCoincident(edge        *pEdge,
                    face        *pFace)
    {
    for(auto pTestEdge : pFace->GetEdges())
        {
        if(AreEdgesCoincident(pEdge,pTestEdge))
            {
            return pTestEdge;
            }
        }
    return nullptr;
    }

void MergeEdges(SGM::Result &rResult,
                edge        *pKeepEdge,
                edge        *pDeleteEdge)
    {
    vertex *pStartKeep=pKeepEdge->GetStart();
    vertex *pEndKeep=pKeepEdge->GetEnd();
    vertex *pStartDelete=pDeleteEdge->GetStart();
    vertex *pEndDelete=pDeleteEdge->GetEnd();
    bool bFlip=false;
    if(pStartKeep->GetPoint().DistanceSquared(pStartDelete->GetPoint())<SGM_ZERO)
        {
        MergeVertices(rResult,pStartKeep,pStartDelete);
        MergeVertices(rResult,pEndKeep,pEndDelete);
        }
    else
        {
        bFlip=true;
        MergeVertices(rResult,pStartKeep,pEndDelete);
        MergeVertices(rResult,pEndKeep,pStartDelete);
        }
    std::set<face *,EntityCompare> sFaces=pDeleteEdge->GetFaces();
    for(face *pFace : sFaces)
        {
        SGM::EdgeSideType nType=pFace->GetSideType(pDeleteEdge);
        pFace->RemoveEdge(rResult,pDeleteEdge);
        if(bFlip)
            {
            if(nType==SGM::EdgeSideType::FaceOnLeftType)
                {
                nType=SGM::EdgeSideType::FaceOnRightType;
                }
            else if(nType==SGM::EdgeSideType::FaceOnRightType)
                {
                nType=SGM::EdgeSideType::FaceOnLeftType;
                }
            pFace->AddEdge(rResult,pKeepEdge,nType);
            }
        else
            {
            pFace->AddEdge(rResult,pKeepEdge,nType);
            }
        }
    curve *pCurve=pDeleteEdge->GetCurve();
    pDeleteEdge->SeverRelations(rResult);
    rResult.GetThing()->DeleteEntity(pCurve);
    rResult.GetThing()->DeleteEntity(pDeleteEdge);
    }

std::vector<face *> ImprintEdgeOnFace(SGM::Result &rResult,
                                      edge        *pEdge,
                                      face        *pFace,
                                      entity      *pStartEntity,
                                      entity      *pEndEntity)
    {
    std::vector<face *> aFaces;
    aFaces.push_back(pFace);
    
    // Figure out which of the following seven cases the edge is on the face.
    //
    // Peninsula (Starts on a loop and ends in the interior of the face.)
    // Splitter (Starts and ends on the same loop and is contractible in the face.)
    // Bridge (Starts and ends on different loops of the face.)
    // Island (Starts and ends in the interior of the face and is either closed or degenerate.)
    // Atoll (Is closed, non-degenerate and starts in the interior of the face.)
    // Non-Contractible (The edge is not contractible in the face.)
    // Coincident (The edge is coincident with an existing edge of the face.)


    if(edge *pConincentEdge=AreCoincident(pEdge,pFace))
        {
        MergeEdges(rResult,pEdge,pConincentEdge);
        }
    else if(pStartEntity && pEndEntity)
        {
        // Bridge, Splitter

        std::vector<std::vector<edge *> > aaLoops;
        std::vector<std::vector<SGM::EdgeSideType> > aaEdgeSideTypes;
        pFace->FindLoops(rResult,aaLoops,aaEdgeSideTypes);
        size_t nStartLoop=FindLoop(pStartEntity,aaLoops);
        size_t nEndLoop=FindLoop(pEndEntity,aaLoops);
        if(nStartLoop==nEndLoop)
            {
            // Splitter

            aFaces.push_back(ImprintSplitter(rResult,pEdge,pFace,pStartEntity,pEndEntity));
            }
        else
            {
            // Bridge

            ImprintBridge(rResult,pEdge,pFace,pStartEntity,pEndEntity);
            }
        }
    else if(pStartEntity || pEndEntity)
        {
        // Peninsula
        
        ImprintPeninsula(rResult,pEdge,pFace,pStartEntity,pEndEntity);
        }
    else if(LowersGenus(rResult,pEdge,pFace))
        {
        // Lowers Genus

        ImprintLowersGenus(rResult,pEdge,pFace);
        }
    else if(pEdge->IsClosed() && !pEdge->IsDegenerate())
        {
        // Atoll

        aFaces.push_back(ImprintAtoll(rResult,pEdge,pFace));
        }
    else
        {
        // Island

        pFace->AddEdge(rResult,pEdge,SGM::EdgeSideType::FaceOnBothSidesType);
        }

    return aFaces;
    }

void FindStartAndEndEnts(edge                const *pEdge,
                         std::map<double,entity *> &mHitMap1,
                         std::map<double,entity *> &mHitMap2,
                         entity                    *&pStartEnt,
                         entity                    *&pEndEnt)
    {
    pStartEnt=nullptr;
    pEndEnt=nullptr;
    SGM::Interval1D const &Domain=pEdge->GetDomain();
    double dStart=Domain.m_dMin;
    double dEnd=Domain.m_dMax;
    double dMinDist=std::numeric_limits<double>::max();
    for(auto iter : mHitMap1)
        {
        double dDist=fabs(dStart-iter.first);
        if(dDist<dMinDist)
            {
            dMinDist=dDist;
            pStartEnt=iter.second;
            }
        }
    for(auto iter : mHitMap2)
        {
        double dDist=fabs(dStart-iter.first);
        if(dDist<dMinDist)
            {
            pStartEnt=nullptr;
            break;
            }
        }
    dMinDist=std::numeric_limits<double>::max();
    for(auto iter : mHitMap1)
        {
        double dDist=fabs(dEnd-iter.first);
        if(dDist<dMinDist)
            {
            dMinDist=dDist;
            pEndEnt=iter.second;
            }
        }
    for(auto iter : mHitMap2)
        {
        double dDist=fabs(dEnd-iter.first);
        if(dDist<dMinDist)
            {
            pEndEnt=nullptr;
            break;
            }
        }
    }

bool ImprintFaces(SGM::Result &rResult,
                  face        *pFace1,
                  face        *pFace2)
    {
    // Find the new edges.

    double dTolerance=SGM_MIN_TOL;
    std::vector<curve *> aCurves;
    surface const *pSurface1=pFace1->GetSurface();
    surface const *pSurface2=pFace2->GetSurface();
    size_t nCurves=IntersectSurfaces(rResult,pSurface1,pSurface2,aCurves,dTolerance);
    std::vector<edge *> aEdges;
    std::vector<SGM::Point3D> aPoints;
    std::map<double,entity *> mHitMap1,mHitMap2;
    size_t Index1;
    for(Index1=0;Index1<nCurves;++Index1)
        {
        curve *pCurve=aCurves[Index1];
        TrimCurveWithFaces(rResult,pCurve,pFace1,pFace2,aEdges,aPoints,mHitMap1,mHitMap2);
        }

    // Imprint the new edges.

    for(edge *pEdge : aEdges)
        {
        entity *pStartEnt1,*pEndEnt1,*pStartEnt2,*pEndEnt2;
        FindStartAndEndEnts(pEdge,mHitMap1,mHitMap2,pStartEnt1,pEndEnt1);
        FindStartAndEndEnts(pEdge,mHitMap2,mHitMap1,pStartEnt2,pEndEnt2);
        std::vector<face *> aFaces1=ImprintEdgeOnFace(rResult,pEdge,pFace1,pStartEnt1,pEndEnt1);
        std::vector<face *> aFaces2=ImprintEdgeOnFace(rResult,pEdge,pFace2,pStartEnt2,pEndEnt2);
        }
    return !(aEdges.empty());
    }

void MergeAndMoveVolumes(SGM::Result            &rResult,
                         body                   *pKeepBody,
                         std::set<size_t> const &sVolumeIDs)
    {
    // First find the volume to keep.

    std::set<volume *,EntityCompare> sVolumes=pKeepBody->GetVolumes();
    volume *pKeepVolume=nullptr;
    for(size_t VolumeID : sVolumeIDs)
        {
        auto pVolume=(volume *)rResult.GetThing()->FindEntity(VolumeID);
        if(pKeepVolume==nullptr && sVolumes.find(pVolume)!=sVolumes.end())
            {
            pKeepVolume=pVolume;
            break;
            }
        }

    // Move all faces and edges of each non-keep volume over to the keep volume.
    
    for(size_t VolumeID : sVolumeIDs)
        {
        auto pVolume=(volume *)rResult.GetThing()->FindEntity(VolumeID);
        if(pVolume!=pKeepVolume)
            {
            if(pKeepVolume==nullptr)
                {
                pVolume->SetBody(pKeepBody);
                }
            else
                {
                std::set<face *,EntityCompare> sFaces=pVolume->GetFaces();
                for(face *pFace : sFaces)
                    {
                    pVolume->RemoveFace(pFace);
                    pKeepVolume->AddFace(pFace);
                    }
                std::set<edge *,EntityCompare> sEdges=pVolume->GetEdges();
                for(edge *pEdge : sEdges)
                    {
                    pVolume->RemoveEdge(pEdge);
                    pKeepVolume->AddEdge(pEdge);
                    }
                body *pBody=pVolume->GetBody();
                pBody->RemoveVolume(pVolume);
                rResult.GetThing()->DeleteEntity(pVolume);
                }
            }
        }
    }

std::vector<face *> ImprintEdgeOnFace(SGM::Result &rResult,
                                      edge        *pEdge,
                                      face        *pFace)
    {
    std::vector<face *> aAnswer;

    std::vector<edge *> aEdges;
    std::vector<SGM::Point3D> aPoints;
    std::map<double,entity *> mHitMap1,mHitMap2;
    curve *pCurve=pEdge->GetCurve();
    TrimCurveWithFaces(rResult,pCurve,pFace,nullptr,aEdges,aPoints,mHitMap1,mHitMap2,pEdge);
    pCurve->RemoveEdge(pEdge);
    rResult.GetThing()->DeleteEntity(pEdge);

    for(edge *pSubEdge : aEdges)
        {
        entity *pStartEnt1,*pEndEnt1;
        FindStartAndEndEnts(pSubEdge,mHitMap1,mHitMap2,pStartEnt1,pEndEnt1);
        std::vector<face *> aFaces=ImprintEdgeOnFace(rResult,pSubEdge,pFace,pStartEnt1,pEndEnt1);
        aAnswer.insert(aAnswer.end(),aFaces.begin(),aFaces.end());
        }

    return aAnswer;
    }

void UniteBodies(SGM::Result &rResult,
                 body        *pKeepBody,
                 body        *pDeleteBody)
    {
    // To start with this is using an n^2 loop.  However, this should be replaced with a tree.

    std::set<std::pair<size_t,size_t> > sMergedVolumes;
    std::set<face *,EntityCompare> sFaces0,sFaces1;
    FindFaces(rResult,pKeepBody,sFaces0);
    FindFaces(rResult,pDeleteBody,sFaces1);
    for(face *pFace0 : sFaces0)
        {
        for(face *pFace1 : sFaces1)
            {
            if(ImprintFaces(rResult,pFace0,pFace1))
                {
                sMergedVolumes.insert({pFace0->GetVolume()->GetID(),pFace1->GetVolume()->GetID()});
                }
            }
        }

    // Merge volumes and bodies.

    std::set<volume *,EntityCompare> const &sVolumes0=pKeepBody->GetVolumes();
    std::set<volume *,EntityCompare> const &sVolumes1=pDeleteBody->GetVolumes();
    std::set<size_t> sGraphVertices;
    std::set<SGM::GraphEdge> sGraphEdges;
    for(volume *pVolume : sVolumes0)
        {
        sGraphVertices.insert(pVolume->GetID());
        }
    for(volume *pVolume : sVolumes1)
        {
        sGraphVertices.insert(pVolume->GetID());
        }
    size_t nGraphEdgeCount=0;
    for(auto iter : sMergedVolumes)
        {
        sGraphEdges.insert(SGM::GraphEdge(iter.first,iter.second,nGraphEdgeCount));
        ++nGraphEdgeCount;
        }
    SGM::Graph graph(sGraphVertices,sGraphEdges);
    std::vector<SGM::Graph> aComps;
    graph.FindComponents(aComps);
    for(SGM::Graph const &comp : aComps)
        {
        std::set<size_t> const &sVerts=comp.GetVertices();
        MergeAndMoveVolumes(rResult,pKeepBody,sVerts);
        }
    rResult.GetThing()->DeleteEntity(pDeleteBody);

    // Orient the sheet body faces to be consistant with each other.

    OrientBody(rResult,pKeepBody);
    }

void FindWindingNumbers(surface                   const *pSurface,
                        std::vector<SGM::Point3D> const &aPolygon3D,
                        int                             &nUWinds,
                        int                             &nVWinds)
    {
    // Find the uv values of the points on the surface.

    std::vector<SGM::Point2D> aPolygon2D;
    size_t nPolygon=aPolygon3D.size();
    aPolygon2D.reserve(nPolygon);
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        SGM::Point2D uv=pSurface->Inverse(aPolygon3D[Index1]);
        aPolygon2D.push_back(uv);
        }

    // Find the U winding number.

    nUWinds=0;
    if(pSurface->ClosedInU())
        {
        // Find an unused U Value.

        SGM::Interval1D UDomain=pSurface->GetDomain().m_UDomain;
        double dMid=UDomain.MidPoint();
        double dDist=UDomain.Length()/4.0;
        std::vector<double> aUs;
        aUs.reserve(nPolygon);
        for(auto uv : aPolygon2D)
            {
            aUs.push_back(uv.m_u);
            }
        std::sort(aUs.begin(),aUs.end());
        double dU=SGM_MAX;
        for(Index1=1;Index1<nPolygon;++Index1)
            {
            if(SGM_MIN_TOL<aUs[Index1]-aUs[Index1-1] && fabs(aUs[Index1]-dMid)<dDist)
                {
                double dTest=(aUs[Index1]+aUs[Index1-1])*0.5;
                if( fabs(dTest-dMid)<fabs(dU-dMid) )
                    {
                    dU=dTest;
                    }
                }
            }
    
        // Find the crossing of dU.

        for(Index1=1;Index1<=nPolygon;++Index1)
            {
            SGM::Point2D const &uv1=aPolygon2D[Index1%nPolygon];
            if(fabs(uv1.m_u-dMid)<dDist)
                {
                SGM::Point2D const &uv0=aPolygon2D[Index1-1];
                if(uv0.m_u<dU && dU<uv1.m_u)
                    {
                    ++nUWinds;
                    }
                else if(uv1.m_u<dU && dU<uv0.m_u)
                    {
                    --nUWinds;
                    }
                }
            }
        }

    // Find the V winding number.

    nVWinds=0;
    if(pSurface->ClosedInV())
        {
        // Find an unused V value.

        SGM::Interval1D VDomain=pSurface->GetDomain().m_VDomain;
        double dMid=VDomain.MidPoint();
        double dDist=VDomain.Length()/4.0;
        std::vector<double> aVs;
        aVs.reserve(nPolygon);
        for(auto uv : aPolygon2D)
            {
            aVs.push_back(uv.m_v);
            }
        std::sort(aVs.begin(),aVs.end());
        double dV=SGM_MAX;
        for(Index1=1;Index1<nPolygon;++Index1)
            {
            if(SGM_MIN_TOL<aVs[Index1]-aVs[Index1-1] && fabs(aVs[Index1]-dMid)<dDist)
                {
                double dTest=(aVs[Index1]+aVs[Index1-1])*0.5;
                if( fabs(dTest-dMid)<fabs(dV-dMid) )
                    {
                    dV=dTest;
                    }
                }
            }

        // Find the crossings of dV.

        for(Index1=1;Index1<=nPolygon;++Index1)
            {
            SGM::Point2D const &uv1=aPolygon2D[Index1%nPolygon];
            if(fabs(uv1.m_v-dMid)<dDist)
                {
                SGM::Point2D const &uv0=aPolygon2D[Index1-1];
                if(uv0.m_v<dV && dV<uv1.m_v)
                    {
                    ++nVWinds;
                    }
                else if(uv1.m_v<dV && dV<uv0.m_v)
                    {
                    --nVWinds;
                    }
                }
            }
        }
    }

bool ConsistentFaces(SGM::Result &rResult,
                     face  const *pFace1,
                     face  const *pFace2)
    {
    std::vector<edge *> aEdges;
    FindCommonEdgesFromFaces(rResult,pFace1,pFace2,aEdges);
    edge *pEdge=aEdges[0];
    std::set<face *,EntityCompare> const &sFaces=pEdge->GetFaces();
    if(sFaces.size()==2)
        {
        if(pFace1->GetSideType(pEdge)==pFace2->GetSideType(pEdge))
            {
            return false;
            }
        return true;
        }
    return false;
    }

void OrientFaces(SGM::Result                    &rResult,
                 std::set<face *,EntityCompare> &sfaces)
    {
    std::set<face *,EntityCompare> sUsed;
    face *pFace=*sfaces.begin();
    sfaces.erase(pFace);
    sUsed.insert(pFace);
    while(sfaces.size())
        {
        face *pTestFace=*sfaces.begin();
        std::set<face *,EntityCompare> sAdjacentFaces;
        FindAdjacentFaces(rResult,pTestFace,sAdjacentFaces);
        for(auto pAdjacentFace : sAdjacentFaces)
            {
            if(sUsed.find(pAdjacentFace)!=sUsed.end())
                {
                if(ConsistentFaces(rResult,pTestFace,pAdjacentFace)==false)
                    {
                    pFace->Negate();
                    }
                sUsed.insert(pTestFace);
                sfaces.erase(pTestFace);
                break;
                }
            }
        }
    }

void OrientBody(SGM::Result &rResult,
                body        *pBody)
    {
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pBody,sFaces);
    if(!sFaces.empty())
        {
        std::set<SGM::Face> sDouble;
        for(auto pFace : sFaces)
            {
            if(pFace->GetSides()==2)
                {
                sDouble.insert(SGM::Face(pFace->GetID()));
                }
            }
        SGM::Graph graph(rResult,sDouble,true);
        std::vector<SGM::Graph> aComponents;
        graph.FindComponents(aComponents);
        for(auto comp : aComponents)
            {
            std::set<size_t> const &sVerts=comp.GetVertices();
            std::set<face *,EntityCompare> sfaces;
            for(auto FaceID : sVerts)
                {
                sfaces.insert((face *)rResult.GetThing()->FindEntity(FaceID));
                }
            OrientFaces(rResult,sfaces);
            }
        }
    }

} // End of SGMInternal namespace.

