#include "SGMVector.h"
#include "SGMTransform.h"
#include "SGMModify.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include "Intersectors.h"
#include "Modify.h"
#include "Primitive.h"
#include "Topology.h"
#include "Graph.h"
#include "Faceter.h"

#include <limits>
#include <algorithm>
#include <map>

namespace SGMInternal
{

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
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        IntersectCurves(rResult,pCurve,pEdge->GetCurve(),aPoints,aTypes,nullptr,pEdge,dTolerance);
        for(SGM::Point3D Pos : aPoints)
            {
            if(pLimitEdge && pLimitEdge->PointInEdge(Pos,dTolerance)==false)
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
            std::vector<SGM::Point3D> aPoints;
            std::vector<SGM::IntersectionType> aTypes;
            IntersectCurves(rResult,pCurve,pEdge->GetCurve(),aPoints,aTypes,nullptr,pEdge,dTolerance);
            for(SGM::Point3D Pos : aPoints)
                {
                if(pLimitEdge && pLimitEdge->PointInEdge(Pos,dTolerance)==false)
                    {
                    continue;
                    }
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
        aHitParams.push_back({pCurve->Inverse(Pos),Pos});
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
                SGM::Interval1D Domain(dParam0,dParam1);
                aEdges.push_back(CreateEdge(rResult,pCurve,&Domain));
                aIsolated[Index1-1]=false;
                aIsolated[Index1]=false;
                }
            else
                {
                SGM::Point2D uv1=pSurface1->Inverse(Pos);
                if(pFace1->PointInFace(rResult,uv1))
                    {
                    SGM::Interval1D Domain(dParam0,dParam1);
                    aEdges.push_back(CreateEdge(rResult,pCurve,&Domain));
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

vertex *ImprintPointOnEdge(SGM::Result        &rResult,
                           SGM::Point3D const &Pos,
                           edge               *pEdge)
    {
    // Create the new vertex.

    std::set<face *,EntityCompare> sFaces=pEdge->GetFaces();
    double t=pEdge->GetCurve()->Inverse(Pos);
    vertex *pAnswer=new vertex(rResult,Pos);
    vertex *pEnd=pEdge->GetEnd();

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
        edge *pNewEdge=new edge(rResult);
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

void MergeVerices(SGM::Result &rResult,
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
        MergeVerices(rResult,(vertex *)pStartEntity,pEdge->GetStart());
        }
    else 
        {
        if(pEndEntity->GetType()==SGM::EdgeType)
            {
            pEndEntity=ImprintPointOnEdge(rResult,pEdge->GetEnd()->GetPoint(),(edge *)pEndEntity);
            }
        MergeVerices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());
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
    MergeVerices(rResult,(vertex *)pStartEntity,pEdge->GetStart());

    if(pEndEntity->GetType()==SGM::EdgeType)
        {
        pEndEntity=ImprintPointOnEdge(rResult,pEdge->GetEnd()->GetPoint(),(edge *)pEndEntity);
        }
    MergeVerices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());

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
    MergeVerices(rResult,(vertex *)pStartEntity,pEdge->GetStart());
    MergeVerices(rResult,(vertex *)pEndEntity,pEdge->GetEnd());

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

    face *pNewFace=new face(rResult);
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

face *ImprintAtoll(SGM::Result &rResult,
                   edge        *pEdge,
                   face        *pFace)
    {
    // Test to see if pEdge is clockwise in the face.
    // Create a new face and add pEdge to both pFace and pNewFace.

    face *pNewFace=new face(rResult);
    pNewFace->SetSurface(pFace->GetSurface());
    pNewFace->SetSides(pFace->GetSides());
    pNewFace->AddEdge(rResult,pEdge,SGM::FaceOnLeftType);
    pFace->GetVolume()->AddFace(pNewFace);
    std::vector<unsigned int> aAdjacencies;
    std::vector<std::vector<unsigned int> > aaPolygons;
    FacetOptions Options;
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<entity *> aEntities;
    FacetFaceLoops(rResult,pNewFace,Options,aPoints2D,aPoints3D,aaPolygons,pEdge);
    double dArea=0.0;
    for(std::vector<unsigned int> const &aPolygon : aaPolygons)
        {
        dArea+=SGM::PolygonArea(PointFormPolygon(aPoints2D,aPolygon));
        }
    if(dArea<0)
        {
        pNewFace->RemoveEdge(rResult,pEdge);
        pNewFace->AddEdge(rResult,pEdge,SGM::FaceOnRightType);
        }
    pFace->AddEdge(rResult,pEdge,pNewFace->GetSideType(pEdge)==SGM::FaceOnRightType ? SGM::FaceOnLeftType : SGM::FaceOnRightType);
    
    return pNewFace;
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

    if(pStartEntity && pEndEntity)
        {
        // Bridge, Splitter, or Non-Contractible

        std::vector<std::vector<edge *> > aaLoops;
        std::vector<std::vector<SGM::EdgeSideType> > aaEdgeSideTypes;
        pFace->FindLoops(rResult,aaLoops,aaEdgeSideTypes);
        size_t nStartLoop=FindLoop(pStartEntity,aaLoops);
        size_t nEndLoop=FindLoop(pEndEntity,aaLoops);
        if(nStartLoop==nEndLoop)
            {
            // Splitter, or Non-Contractible

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
    else if(pEdge->IsClosed() && pEdge->IsDegenerate()==false)
        {
        // Atoll or Non-Contractible

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
    size_t nCurves=IntersectSurfaces(rResult,pSurface1,pSurface2,aCurves,pFace1,pFace2,dTolerance);
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
        volume *pVolume=(volume *)rResult.GetThing()->FindEntity(VolumeID);
        if(pKeepVolume==nullptr && sVolumes.find(pVolume)!=sVolumes.end())
            {
            pKeepVolume=pVolume;
            break;
            }
        }

    // Move all faces and edges of each non-keep volume over to the keep volume.
    
    for(size_t VolumeID : sVolumeIDs)
        {
        volume *pVolume=(volume *)rResult.GetThing()->FindEntity(VolumeID);
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
    std::set<GraphEdge> sGraphEdges;
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
        sGraphEdges.insert(GraphEdge(iter.first,iter.second,nGraphEdgeCount));
        ++nGraphEdgeCount;
        }
    Graph graph(sGraphVertices,sGraphEdges);
    std::vector<Graph> aComps;
    graph.FindComponents(aComps);
    for(Graph const &comp : aComps)
        {
        std::set<size_t> const &sVerts=comp.GetVertices();
        MergeAndMoveVolumes(rResult,pKeepBody,sVerts);
        }
    rResult.GetThing()->DeleteEntity(pDeleteBody);
    }

} // End of SGMInternal namespace.

