#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Faceter.h"
#include "Graph.h"
#include <list>
#include <cmath>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////
//
//  Facet Functions
//
///////////////////////////////////////////////////////////////////////////////

class FacetNode
    {
    public:

    FacetNode(double              dParam,
              SGM::Point3D const &Pos):m_dParam(dParam),m_Pos(Pos) {}

    double       m_dParam;
    SGM::Point3D m_Pos;
    };

void FacetCurve(curve               const *pCurve,
                SGM::Interval1D     const &Domain,
                FacetOptions        const &Options,
                std::vector<SGM::Point3D> &aPoints3D,
                std::vector<double>       *aParams)
    {
    SGM::EntityType nType=pCurve->GetCurveType();
    switch(nType)
        {
        case SGM::LineType:
            {
            SGM::Point3D Start,End;
            line const *pLine=(line const *)pCurve;
            pLine->Evaluate(Domain.m_dMin,&Start);
            pLine->Evaluate(Domain.m_dMax,&End);
            aPoints3D.reserve(2);
            aPoints3D.push_back(Start);
            aPoints3D.push_back(End);
            if(aParams)
                {
                aParams->reserve(2);
                aParams->push_back(Domain.m_dMin);
                aParams->push_back(Domain.m_dMax);
                }
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)pCurve;
            double dRadius=pCircle->GetRadius();
            double dDomainLength=Domain.Length();
            double dAngle=Options.m_dEdgeAngleTol;
            if(Options.m_dMaxLength)
                {
                double dArcLength=dDomainLength*dRadius;
                double dAngleFraction=dArcLength/Options.m_dMaxLength;
                dAngle=std::min(dAngle,dDomainLength*dAngleFraction);
                }
            if(Options.m_dCordHight)
                {
                // CordAngle = 2*acos(1-CordHight/Radius)
                dAngle=std::min(dAngle,2.0*acos(1.0-Options.m_dCordHight/dRadius));
                }
            size_t nPoints=std::min((size_t)(1.0000001+dDomainLength/dAngle),Options.m_nMaxFacets);
            if(nPoints<3)
                {
                nPoints=3;
                }
            aPoints3D.reserve(nPoints);
            if(aParams)
                {
                aParams->reserve(nPoints);
                }
            size_t Index1;
            for(Index1=0;Index1<nPoints;++Index1)
                {
                double dFraction=(Index1/(nPoints-1.0));
                double dt=Domain.MidPoint(dFraction);
                if(aParams)
                    {
                    aParams->push_back(dt);
                    }
                SGM::Point3D Pos;
                pCircle->Evaluate(dt,&Pos);
                aPoints3D.push_back(Pos);
                }
            break;
            }
        default:
            {
            std::list<FacetNode> lNodes;
            SGM::Point3D Pos0,Pos1;
            double d0=Domain.m_dMin;
            double d1=Domain.m_dMax;
            pCurve->Evaluate(d0,&Pos0);
            pCurve->Evaluate(d1,&Pos1);
            lNodes.push_back(FacetNode(d0,Pos0));
            lNodes.push_back(FacetNode(d1,Pos1));
            double dAngle=SGM_PI-Options.m_dFreeEdgeAngleTol;
            double dCos=cos(dAngle);
            bool bRefine=true;
            while(bRefine)
                {
                bRefine=false;
                std::list<FacetNode>::iterator LastIter=lNodes.begin();
                std::list<FacetNode>::iterator iter=lNodes.begin();
                ++iter;
                while(iter!=lNodes.end())
                    {
                    double d0=LastIter->m_dParam;
                    double d1=iter->m_dParam;

                    // Check to see if the segment from d0 to d1 needs more facets.

                    double da=d0*0.65433+d1*0.34567;
                    double db=d0*0.5+d1*0.5;
                    double dc=d0*0.34567+d1*0.65433;

                    SGM::Point3D Pos0=LastIter->m_Pos;
                    SGM::Point3D Pos1=iter->m_Pos;
                    SGM::Point3D PosA,PosB,PosC;
                    pCurve->Evaluate(da,&PosA);
                    pCurve->Evaluate(db,&PosB);
                    pCurve->Evaluate(dc,&PosC);

                    double a1=SGM::UnitVector3D(Pos0-PosA)%SGM::UnitVector3D(Pos1-PosA);
                    double a2=SGM::UnitVector3D(Pos0-PosB)%SGM::UnitVector3D(Pos1-PosB);
                    double a3=SGM::UnitVector3D(Pos0-PosC)%SGM::UnitVector3D(Pos1-PosC);
                    if(dCos<a1 || dCos<a2 || dCos<a3)
                        {
                        bRefine=true;
                        lNodes.insert(iter,FacetNode(db,PosB));
                        iter=LastIter;
                        }
                    else
                        {
                        ++LastIter;
                        }
                    ++iter;
                    }
                }
            size_t nNodes=lNodes.size();
            aPoints3D.reserve(nNodes);
            if(aParams)
                {
                aParams->reserve(nNodes);
                }
            std::list<FacetNode>::iterator iter2=lNodes.begin();
            while(iter2!=lNodes.end())
                {
                aPoints3D.push_back(iter2->m_Pos);
                if(aParams)
                    {
                    aParams->push_back(iter2->m_dParam);
                    }
                ++iter2;
                }
            }
        }
    }

void FacetEdge(edge                const *pEdge,
               FacetOptions        const &Options,
               std::vector<SGM::Point3D> &aPoints3D)
    {
    curve const *pCurve=pEdge->GetCurve();
    SGM::Interval1D const &Domain=pEdge->GetDomain();
    FacetCurve(pCurve,Domain,Options,aPoints3D);
    }

size_t FindUCrosses(SGM::Interval1D           const &UDomain,
                    std::vector<SGM::Point2D> const &aPoints2D,
                    std::vector<size_t>       const &aPolygon,
                    std::vector<double>             &aSplitValues,
                    std::vector<size_t>             &aSplitIndex)
    {
    double dMaxGap=UDomain.Length()*0.5;
    double dMinU=UDomain.m_dMin;
    double dMaxU=UDomain.m_dMax;
    size_t nPoints=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &UV0=aPoints2D[aPolygon[Index1]];
        SGM::Point2D const &UV1=aPoints2D[aPolygon[(Index1+1)%nPoints]];
        double dUGap=fabs(UV0.m_u-UV1.m_u);
        if(dMaxGap<dUGap)
            {
            double d0=std::min(UV0.m_u-dMinU,dMaxU-UV0.m_u);
            double d1=std::min(UV1.m_u-dMinU,dMaxU-UV1.m_u);
            if(d0<d1)
                {
                aSplitValues.push_back(UV0.m_v);
                }
            else
                {
                aSplitValues.push_back(UV1.m_v);
                }
            aSplitIndex.push_back(Index1);
            }
        }
    return aSplitValues.size();
    }

class SplitData
    {
    public:

        SplitData() {}

        SplitData(double dParam,size_t nPolygon,size_t nSpan):
            m_dParam(dParam),m_nPolygon(nPolygon),m_nSpan(nSpan) {}

        bool operator<(SplitData const &Other) const {return m_dParam<Other.m_dParam;}

        double m_dParam;
        size_t m_nPolygon;
        size_t m_nSpan;
    };

class MergeData
    {
    public:

        MergeData() {}

        SplitData m_Split1;
        SplitData m_Split2;
    };

GraphEdge FindGraphEdge(SGM::Interval1D           const &Domain,
                        std::vector<size_t>       const &aSplitMap,
                        size_t                           nPoints,
                        std::vector<SGM::Point2D> const &aPoints2D,
                        size_t                           nPos0,
                        size_t                           nPos1,
                        size_t                          &nCount)
    {
    GraphEdge GE(nPos0,nPos1,nCount);
    ++nCount;
    if(nPos0<nPoints && nPos1<nPoints)
        {
        }
    else if(nPos0<nPoints)
        {
        double u0=aPoints2D[nPos0].m_u;
        double u1A=aPoints2D[nPos1].m_u;
        double u1B=aPoints2D[aSplitMap[nPos1-nPoints]].m_u;
        if(u0<Domain.MidPoint())
            {
            GE.m_nEnd=u1A<u1B ? nPos1 : aSplitMap[nPos1-nPoints];
            }
        else
            {
            GE.m_nEnd=u1A<u1B ? aSplitMap[nPos1-nPoints] : nPos1;
            }
        }
    else if(nPos1<nPoints)
        {
        double u1=aPoints2D[nPos1].m_u;
        double u0A=aPoints2D[nPos0].m_u;
        double u0B=aPoints2D[aSplitMap[nPos0-nPoints]].m_u;
        if(u1<Domain.MidPoint())
            {
            GE.m_nStart=u0A<u0B ? nPos0 : aSplitMap[nPos0-nPoints];
            }
        else
            {
            GE.m_nStart=u0A<u0B ? aSplitMap[nPos0-nPoints] : nPos0;
            }
        }
    else
        {
        throw;
        }
    return GE;
    }

void FindLowGraphEdge(SGM::Interval1D           const &Domain,
                      std::vector<size_t>       const &aSplitMap,
                      size_t                           nPoints,
                      std::vector<SGM::Point2D> const &aPoints2D,
                      size_t                           nPos0A,
                      size_t                           nPos1A,
                      size_t                          &nCount,
                      std::set<GraphEdge>             &sGraphEdges)
    {
    size_t nLow0,nLow1,nHigh0,nHigh1;
    if(aPoints2D[nPos0A].m_u<Domain.MidPoint())
        {
        nLow0=nPos0A;
        nHigh0=aSplitMap[nPos0A-nPoints];
        }
    else
        {
        nLow0=aSplitMap[nPos0A-nPoints];
        nHigh0=nPos0A;
        }
    if(aPoints2D[nPos1A].m_u<Domain.MidPoint())
        {
        nLow1=nPos1A;
        nHigh1=aSplitMap[nPos1A-nPoints];
        }
    else
        {
        nLow1=aSplitMap[nPos1A-nPoints];
        nHigh1=nPos1A;
        }
    size_t nA,nB,nC,nD;
    if(aPoints2D[nLow0].m_v<aPoints2D[nLow1].m_v)
        {
        nA=nLow1;
        nB=nLow0;
        }
    else
        {
        nA=nLow0;
        nB=nLow1;
        }
    if(aPoints2D[nHigh0].m_v<aPoints2D[nHigh1].m_v)
        {
        nC=nHigh0;
        nD=nHigh1;
        }
    else
        {
        nC=nHigh1;
        nD=nHigh0;
        }

    sGraphEdges.insert(GraphEdge(nA,nB,nCount));
    ++nCount;
    sGraphEdges.insert(GraphEdge(nC,nD,nCount));
    ++nCount;
    }

void MergePolygons(face                              const *pFace,
                   std::vector<MergeData>            const &aMerge,
                   std::vector<SGM::Point2D>               &aPoints2D,
                   std::vector<SGM::Point3D>               &aPoints3D,
                   std::vector<entity *>                   &aEntities,
                   std::vector<std::vector<size_t> >       &aaPolygons)
    {
    SGM::Interval1D const &UDomain=pFace->GetSurface()->GetDomain().m_UDomain;
    double dMidDomain=UDomain.MidPoint();

    // Add the span points.

    size_t nPoints=aPoints2D.size();
    std::vector<size_t> aSplitMap;
    size_t nMerge=aMerge.size();
    size_t nSplits=nMerge*2;
    aSplitMap.reserve(nSplits);
    size_t Index1;
    std::vector<SplitData> aSplits;
    aSplits.reserve(nSplits);
    std::vector<size_t> aSplitVertices;
    aSplitVertices.reserve(nSplits);
    std::set<size_t> sUsedPolygons;
    for(Index1=0;Index1<nMerge;++Index1)
        {
        aSplits.push_back(aMerge[Index1].m_Split1);
        aSplits.push_back(aMerge[Index1].m_Split2);
        sUsedPolygons.insert(aMerge[Index1].m_Split1.m_nPolygon);
        sUsedPolygons.insert(aMerge[Index1].m_Split2.m_nPolygon);
        }
    for(Index1=0;Index1<nSplits;++Index1)
        {
        size_t nPoly=aSplits[Index1].m_nPolygon;
        size_t nSpan=aSplits[Index1].m_nSpan;
        std::vector<size_t> &aPoly=aaPolygons[nPoly];
        size_t nPolyPoints=aPoly.size();
        size_t nPos0=aPoly[nSpan];
        size_t nPos1=aPoly[(nSpan+1)%nPolyPoints];
        double u0=aPoints2D[nPos0].m_u;
        double u1=aPoints2D[nPos1].m_u;
        double v0=aPoints2D[nPos0].m_v;
        double v1=aPoints2D[nPos1].m_v;
        SGM::Point3D const &Pos0=aPoints3D[nPos0];
        SGM::Point3D const &Pos1=aPoints3D[nPos1];
        entity *pEnt0=aEntities[nPos0];
        entity *pEnt1=aEntities[nPos1];
        if(UDomain.OnBoundary(u0))
            {
            if(dMidDomain<u0)
                {
                u0=UDomain.m_dMin;
                }
            else
                {
                u0=UDomain.m_dMax;
                }
            aSplitMap.push_back(nPos0);
            nPos0=aPoints2D.size();
            aSplitVertices.push_back(nPos0);
            aPoly[nSpan]=nPos0;
            aPoints2D.push_back(SGM::Point2D(u0,v0));
            aPoints3D.push_back(Pos0);
            aEntities.push_back(pEnt0);
            }
        else if(UDomain.OnBoundary(u1))
            {
            if(dMidDomain<u1)
                {
                u1=UDomain.m_dMin;
                }
            else
                {
                u1=UDomain.m_dMax;
                }
            aSplitMap.push_back(nPos1);
            nPos1=aPoints2D.size();
            aSplitVertices.push_back(nPos1);
            aPoly[(nSpan+1)%nPolyPoints]=nPos1;
            aPoints2D.push_back(SGM::Point2D(u1,v1));
            aPoints3D.push_back(Pos1);
            aEntities.push_back(pEnt1);
            }
        else
            {
            throw;
            }
        }

    // Create a segment graph.

    size_t nCount=0;
    std::set<GraphEdge> sFacetEdges;
    std::set<size_t> sFacetPoints;
    std::set<size_t>::iterator PolyIter=sUsedPolygons.begin();
    while(PolyIter!=sUsedPolygons.end())
        {
        std::vector<size_t> const &aPoly=aaPolygons[*PolyIter];
        size_t nPoly=aPoly.size();
        for(Index1=0;Index1<nPoly;++Index1)
            {
            size_t nPos0=aPoly[Index1];
            size_t nPos1=aPoly[(Index1+1)%nPoly];
            sFacetPoints.insert(nPos0);
            sFacetEdges.insert(FindGraphEdge(UDomain,aSplitMap,nPoints,aPoints2D,nPos0,nPos1,nCount));
            }
        ++PolyIter;
        }
    for(Index1=0;Index1<nSplits;Index1+=2)
        {
        FindLowGraphEdge(UDomain,aSplitMap,nPoints,aPoints2D,aSplitVertices[Index1],aSplitVertices[Index1+1],nCount,sFacetEdges);
        }
    Graph graph(sFacetPoints,sFacetEdges);
    std::vector<Graph> aComps;
    size_t nComps=graph.FindComponents(aComps);

    // Build the new polygons.

    std::vector<std::vector<size_t> > aaNewPolygons;
    for(Index1=0;Index1<nComps;++Index1)
        {
        Graph const &comp=aComps[Index1];
        if(comp.IsCycle())
            {
            std::vector<size_t> aPolygon;
            comp.OrderVertices(aPolygon);
            aaNewPolygons.push_back(aPolygon);
            }
        else
            {
            throw;
            }
        }
    aaPolygons=aaNewPolygons;
    }

void CreateWholeSurfaceLoop(SGM::Result                       &rResult,
                            face                        const *pFace,
                            FacetOptions                const &Options,
                            std::vector<SGM::Point2D>         &aPoints2D,
                            std::vector<SGM::Point3D>         &aPoints3D,
                            std::vector<entity *>             &aEntities,
                            std::vector<std::vector<size_t> > &aaPolygons,
                            std::vector<size_t>               &aTriangles)
    {
    surface const *pSurface=pFace->GetSurface();
    if(pSurface->ClosedInU())
        {
        if(pSurface->ClosedInV())
            {
            // The torus case.

//            SGM::Interval2D const &Domain=pSurface->GetDomain();
//            double dMinU=Domain.m_UDomain.m_dMin;
//            double dMaxU=Domain.m_UDomain.m_dMax;
//            double dMinV=Domain.m_VDomain.m_dMin;
//            double dMaxV=Domain.m_VDomain.m_dMax;
//
//            curve *pUSeam=pSurface->UParamLine(rResult,dMinU);
//            curve *pVSeam=pSurface->VParamLine(rResult,dMinV);
//
//            dMaxU;
//            dMaxV;
//
//            pUSeam->Remove(rResult);
//            pVSeam->Remove(rResult);
            }
        else if(pSurface->SingularLowV() && pSurface->SingularHighV())
            {
            // The sphere case

            SGM::Interval2D const &Domain=pSurface->GetDomain();
            double dMinU=Domain.m_UDomain.m_dMin;
            double dMaxU=Domain.m_UDomain.m_dMax;
            double dMinV=Domain.m_VDomain.m_dMin;
            double dMaxV=Domain.m_VDomain.m_dMax;

            // Find the seam points.

            curve *pSeam=pSurface->UParamLine(rResult,dMinU);
            std::vector<SGM::Point3D> aTemp3D,aSeamPoints3D;
            std::vector<double> aSeamVs;
            FacetCurve(pSeam,Domain.m_VDomain,Options,aTemp3D);
            pSeam->Remove(rResult);
            
            // Trim off the two singularities.

            size_t nTemp3D=aTemp3D.size();
            aSeamPoints3D.reserve(nTemp3D-2);
            size_t Index1;
            for(Index1=1;Index1<nTemp3D-1;++Index1)
                {
                aSeamPoints3D.push_back(aTemp3D[Index1]);
                }
            size_t nSeamPoints3D=aSeamPoints3D.size();
            SGM::Point3D LowPos=aSeamPoints3D.front();
            SGM::Point3D HighPos=aSeamPoints3D.back();
            dMinV=pSurface->Inverse(LowPos).m_v;
            dMaxV=pSurface->Inverse(HighPos).m_v;

            aSeamVs.reserve(nSeamPoints3D);
            for(Index1=0;Index1<nSeamPoints3D;++Index1)
                {
                aSeamVs.push_back(pSeam->Inverse(aSeamPoints3D[Index1]));
                }
            std::vector<size_t> aPolygon;
            aPolygon.reserve(nSeamPoints3D*2-2);
            aPoints2D.reserve(nSeamPoints3D*2-2);
            aPoints3D.reserve(nSeamPoints3D*2-2);
            aEntities.assign(nSeamPoints3D*2-2,(entity *)pFace);
            size_t nCount=0;

            // South pole.

            size_t nSouthPoleStart=aPoints3D.size();
            size_t nPolePoints=7;
            for(Index1=0;Index1<nPolePoints;++Index1)
                {
                double dFraction=Index1/(nPolePoints-1.0);
                double dU=Domain.m_UDomain.MidPoint(dFraction);
                SGM::Point3D Pos;
                SGM::Point2D uv(dU,dMinV);
                pSurface->Evaluate(uv,&Pos);
                aPoints3D.push_back(Pos);
                aPoints2D.push_back(uv);
                aPolygon.push_back(nCount);
                ++nCount;
                }
            
            // High U seam

            for(Index1=1;Index1<nSeamPoints3D-1;++Index1)
                {
                aPoints3D.push_back(aSeamPoints3D[Index1]);
                aPoints2D.push_back(SGM::Point2D(dMaxU,aSeamVs[Index1]));
                aPolygon.push_back(nCount);
                ++nCount;
                }

            // North pole.

            size_t nNorthPoleStart=aPoints3D.size();
            for(Index1=0;Index1<nPolePoints;++Index1)
                {
                double dFraction=(nPolePoints-1-Index1)/(nPolePoints-1.0);
                double dU=Domain.m_UDomain.MidPoint(dFraction);
                SGM::Point3D Pos;
                SGM::Point2D uv(dU,dMaxV);
                pSurface->Evaluate(uv,&Pos);
                aPoints3D.push_back(Pos);
                aPoints2D.push_back(uv);
                aPolygon.push_back(nCount);
                ++nCount;
                }

            // Low U seam

            for(Index1=nSeamPoints3D-2;0<Index1;--Index1)
                {
                aPoints3D.push_back(aSeamPoints3D[Index1]);
                aPoints2D.push_back(SGM::Point2D(dMinU,aSeamVs[Index1]));
                aPolygon.push_back(nCount);
                ++nCount;
                }

            aaPolygons.push_back(aPolygon);
            
            // Add in the polar triangles.

            SGM::Point3D NorthPole=aTemp3D.back();
            SGM::Point3D SouthPole=aTemp3D.front();
            SGM::Point2D NorthUV(Domain.m_UDomain.MidPoint(),Domain.m_VDomain.m_dMax);
            SGM::Point2D SouthUV(Domain.m_UDomain.MidPoint(),Domain.m_VDomain.m_dMin);
            size_t nNorth=aPoints3D.size();
            aPoints3D.push_back(NorthPole);
            aPoints2D.push_back(NorthUV);
            size_t nSouth=aPoints3D.size();
            aPoints3D.push_back(SouthPole);
            aPoints2D.push_back(SouthUV);
            for(Index1=nNorthPoleStart+1;Index1<nNorthPoleStart+nPolePoints;++Index1)
                {
                aTriangles.push_back(nNorth);
                aTriangles.push_back(Index1-1);
                aTriangles.push_back(Index1);
                }
            for(Index1=nSouthPoleStart+1;Index1<nSouthPoleStart+nPolePoints;++Index1)
                {
                aTriangles.push_back(nSouth);
                aTriangles.push_back(Index1-1);
                aTriangles.push_back(Index1);
                }
            }
        }
    }

size_t FacetFaceLoops(SGM::Result                       &rResult,
                      face                        const *pFace,
                      FacetOptions                const &Options,
                      std::vector<SGM::Point2D>         &aPoints2D,
                      std::vector<SGM::Point3D>         &aPoints3D,
                      std::vector<entity *>             &aEntities,
                      std::vector<std::vector<size_t> > &aaPolygons,
                      std::vector<size_t>               &aTriangles)
    {
    // First find and use all the vertices.

    surface const *pSurface=pFace->GetSurface();
    std::set<vertex *> sVertices;
    FindVertices(rResult,pFace,sVertices);
    std::set<vertex *>::iterator VertexIter=sVertices.begin();
    std::map<vertex *,size_t> mVertexMap;
    size_t nCount=0;
    while(VertexIter!=sVertices.end())
        {
        vertex *pVertex=*VertexIter;
        SGM::Point3D const &Pos=pVertex->GetPoint();
        aPoints3D.push_back(Pos);
        SGM::Point2D uv=pSurface->Inverse(Pos);
        aPoints2D.push_back(uv);
        mVertexMap[pVertex]=nCount;
        aEntities.push_back(pVertex);
        ++nCount;
        ++VertexIter;
        }

    // Then find all the edges.

    std::vector<std::vector<edge *> > aaLoops;
    std::vector<std::vector<SGM::EdgeSideType> > aaEdgeSideType;
    size_t nLoops=pFace->FindLoops(rResult,aaLoops,aaEdgeSideType);
    bool bFlip=pFace->GetFlipped();
    size_t Index1,Index2,Index3;
    for(Index1=0;Index1<nLoops;++Index1)
        {
        std::vector<edge *> const &aLoop=aaLoops[Index1];
        std::vector<SGM::EdgeSideType> const &aEdgeSideType=aaEdgeSideType[Index1];
        size_t nEdges=aLoop.size();
        std::vector<size_t> aPolygon;
        for(Index2=0;Index2<nEdges;++Index2)
            {
            edge *pEdge=aLoop[Index2];
            vertex *pStart=pEdge->GetStart();
            vertex *pEnd=pEdge->GetEnd();
            std::vector<SGM::Point3D> aPoints;
            FacetEdge(pEdge,Options,aPoints);
            if(bFlip)
                {
                if(aEdgeSideType[Index2]==SGM::EdgeSideType::FaceOnLeftType)
                    {
                    std::reverse(aPoints.begin(),aPoints.end());
                    std::swap(pStart,pEnd);
                    }
                }
            else
                {
                if(aEdgeSideType[Index2]==SGM::EdgeSideType::FaceOnRightType)
                    {
                    std::reverse(aPoints.begin(),aPoints.end());
                    std::swap(pStart,pEnd);
                    }
                }
            if(pStart)
                {
                aPolygon.push_back(mVertexMap[pStart]);
                }
            size_t nVertices=0;
            if(pStart)
                {
                ++nVertices;
                }
            if(pEnd)
                {
                ++nVertices;
                }
            size_t nPoints=aPoints.size()-1;
            if(nVertices<nPoints)
                {
                for(Index3=nVertices ? 1 : 0;Index3<nPoints;++Index3)
                    {
                    aPolygon.push_back(aPoints2D.size());
                    SGM::Point3D const &Pos=aPoints[Index3];
                    SGM::Point2D uv=pSurface->Inverse(Pos);
                    aPoints2D.push_back(uv);
                    aPoints3D.push_back(Pos);
                    aEntities.push_back(pEdge);
                    }
                }
            }
        aaPolygons.push_back(aPolygon);
        }

    // Connect non-simple loops to make one outer loop.

    if(pSurface->ClosedInU())
        {
        if(nLoops==0)
            {
            // In this case we need a polygon that goes around the whole domain.

            CreateWholeSurfaceLoop(rResult,pFace,Options,aPoints2D,aPoints3D,aEntities,aaPolygons,aTriangles);
            }
        else
            {
            // Find loops the cross the seam.

            SGM::Interval1D const &UDomain=pSurface->GetDomain().m_UDomain;
            std::vector<SplitData> aSplits;
            for(Index1=0;Index1<nLoops;++Index1)
                {
                std::vector<double> aSplitValues;
                std::vector<size_t> aSplitIndex;
                std::vector<size_t> const &aPolygon=aaPolygons[Index1];
                size_t nSplitValues=FindUCrosses(UDomain,aPoints2D,aPolygon,aSplitValues,aSplitIndex);
                for(Index2=0;Index2<nSplitValues;++Index2)
                    {
                    aSplits.push_back(SplitData(aSplitValues[Index2],Index1,aSplitIndex[Index2]));
                    }
                }
            std::sort(aSplits.begin(),aSplits.end());

            // Merge the old loops together.

            std::vector<MergeData> aMerge;
            size_t nSplits=aSplits.size();
            aMerge.reserve(nSplits/2);
            for(Index1=1;Index1<nSplits;Index1+=2)
                {
                MergeData MD;
                MD.m_Split1=aSplits[Index1-1];
                MD.m_Split2=aSplits[Index1];
                aMerge.push_back(MD);
                }
            if(aMerge.size())
                {
                MergePolygons(pFace,aMerge,aPoints2D,aPoints3D,aEntities,aaPolygons);
                }
            }
        }

    return nLoops;
    }

class EdgeValue
    {
    public:

        EdgeValue(double dValue,size_t nTriangle,size_t nEdge):
            m_dDot(dValue),m_nTriangle(nTriangle),m_nEdge(nEdge) {}

        bool operator<(EdgeValue const &Other) const 
            {
            if(m_dDot<Other.m_dDot)
                {
                return true;
                }
            else if(m_dDot==Other.m_dDot)
                {
                if(m_nTriangle<Other.m_nTriangle)
                    {
                    return true;
                    }
                else if(m_nTriangle==Other.m_nTriangle)
                    {
                    if(m_nEdge<Other.m_nEdge)
                        {
                        return true;
                        }
                    }
                }
            return false;
            }

        double m_dDot;
        size_t m_nTriangle;
        size_t m_nEdge;
    };

void FixBackPointer(size_t               nA,
                    size_t               nStart,
                    size_t               nT,
                    std::vector<size_t> &aTriangles,
                    std::vector<size_t> &aAdjacences)
    {
    if(nA!=SIZE_MAX)
        {
        if(nStart==aTriangles[nA])
            {
            aAdjacences[nA]=nT;
            }
        else if(nStart==aTriangles[nA+1])
            {
            aAdjacences[nA+1]=nT;
            }
        else
            {
            aAdjacences[nA+2]=nT;
            }
        }
    }

void FixEdgeData(size_t                                     nTri1,
                 size_t                                     nTri2,
                 std::vector<SGM::UnitVector3D>      const &aNormals,
                 std::vector<size_t>                 const &aTriangles,
                 std::vector<size_t>                 const &aAdjacences,
                 std::set<EdgeValue>                       &sEdgeData,
                 std::map<std::pair<size_t,size_t>,double> &mEdgeData)
    {
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri1,0)],nTri1,0));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri1,1)],nTri1,1));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri1,2)],nTri1,2));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri2,0)],nTri2,0));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri2,1)],nTri2,1));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nTri2,2)],nTri2,2));

    SGM::UnitVector3D const &NormA2=aNormals[aTriangles[nTri1]];
    SGM::UnitVector3D const &NormB2=aNormals[aTriangles[nTri1+1]];
    SGM::UnitVector3D const &NormC2=aNormals[aTriangles[nTri1+2]];
    SGM::UnitVector3D const &NormD2=aNormals[aTriangles[nTri2]];
    SGM::UnitVector3D const &NormE2=aNormals[aTriangles[nTri2+1]];
    SGM::UnitVector3D const &NormF2=aNormals[aTriangles[nTri2+2]];

    if(aTriangles[nTri1]<aTriangles[nTri1+1] && aAdjacences[nTri1]!=SIZE_MAX)
        {
        double dVal=NormA2%NormB2;
        sEdgeData.insert(EdgeValue(dVal,nTri1,0));
        mEdgeData[std::pair<size_t,size_t>(nTri1,0)]=dVal;
        }
    if(aTriangles[nTri1+1]<aTriangles[nTri1+2] && aAdjacences[nTri1+1]!=SIZE_MAX)
        {
        double dVal=NormB2%NormC2;
        sEdgeData.insert(EdgeValue(dVal,nTri1,1));
        mEdgeData[std::pair<size_t,size_t>(nTri1,1)]=dVal;
        }
    if(aTriangles[nTri1+2]<aTriangles[nTri1] && aAdjacences[nTri1+2]!=SIZE_MAX)
        {
        double dVal=NormC2%NormA2;
        sEdgeData.insert(EdgeValue(dVal,nTri1,2));
        mEdgeData[std::pair<size_t,size_t>(nTri1,2)]=dVal;
        }
    if(aTriangles[nTri2]<aTriangles[nTri2+1] && aAdjacences[nTri2]!=SIZE_MAX)
        {
        double dVal=NormD2%NormE2;
        sEdgeData.insert(EdgeValue(dVal,nTri2,0));
        mEdgeData[std::pair<size_t,size_t>(nTri2,0)]=dVal;
        }
    if(aTriangles[nTri2+1]<aTriangles[nTri2+2] && aAdjacences[nTri2+1]!=SIZE_MAX)
        {
        double dVal=NormE2%NormF2;
        sEdgeData.insert(EdgeValue(dVal,nTri2,1));
        mEdgeData[std::pair<size_t,size_t>(nTri2,1)]=dVal;
        }
    if(aTriangles[nTri2+2]<aTriangles[nTri2] && aAdjacences[nTri2+2]!=SIZE_MAX)
        {
        double dVal=NormF2%NormD2;
        sEdgeData.insert(EdgeValue(dVal,nTri2,2));
        mEdgeData[std::pair<size_t,size_t>(nTri2,2)]=dVal;
        }
    }

void IncrementalDelaunay(std::set<size_t>                          &sSearchTris,
                         std::vector<SGM::Point2D>           const &aPoints2D,
                         std::vector<SGM::UnitVector3D>      const &aNormals,
                         std::vector<size_t>                       &aTriangles,
                         std::vector<size_t>                       &aAdjacences,
                         std::set<EdgeValue>                       &sEdgeData,
                         std::map<std::pair<size_t,size_t>,double> &mEdgeData)
    {
    while(sSearchTris.empty()==false)
        {
        size_t nT=*(sSearchTris.begin());
        size_t nT0=aAdjacences[nT];
        size_t nT1=aAdjacences[nT+1];
        size_t nT2=aAdjacences[nT+2];
        sSearchTris.erase(nT);
        if(nT0!=SIZE_MAX)
            {
            if(FlipTriangles(aPoints2D,aTriangles,aAdjacences,nT,0))
                {
                sSearchTris.insert(nT);
                sSearchTris.insert(nT0);
                nT1=aAdjacences[nT+1];
                nT2=aAdjacences[nT+2];
                FixEdgeData(nT,nT0,aNormals,aTriangles,aAdjacences,sEdgeData,mEdgeData);
                }
            }
        if(nT1!=SIZE_MAX)
            {
            if(FlipTriangles(aPoints2D,aTriangles,aAdjacences,nT,1))
                {
                sSearchTris.insert(nT);
                sSearchTris.insert(nT1);
                nT2=aAdjacences[nT+2];
                FixEdgeData(nT,nT1,aNormals,aTriangles,aAdjacences,sEdgeData,mEdgeData);
                }
            }
        if(nT2!=SIZE_MAX)
            {
            if(FlipTriangles(aPoints2D,aTriangles,aAdjacences,nT,2))
                {
                sSearchTris.insert(nT);
                sSearchTris.insert(nT2);
                FixEdgeData(nT,nT2,aNormals,aTriangles,aAdjacences,sEdgeData,mEdgeData);
                }
            }
        }
    }

void SplitEdge(EdgeValue                           const &EV,
               face                                const *pFace,
               std::vector<SGM::Point2D>                 &aPoints2D,
               std::vector<SGM::Point3D>                 &aPoints3D,
               std::vector<SGM::UnitVector3D>            &aNormals,
               std::vector<size_t>                       &aTriangles,
               std::vector<size_t>                       &aAdjacences,
               std::set<EdgeValue>                       &sEdgeData,
               std::map<std::pair<size_t,size_t>,double> &mEdgeData)
    {
    // Add the new point

    surface const *pSurface=pFace->GetSurface();
    size_t nT0=EV.m_nTriangle;
    size_t nEdge=EV.m_nEdge;
    size_t a=aTriangles[nT0+nEdge];
    size_t b=aTriangles[nT0+(nEdge+1)%3];
    size_t c=aTriangles[nT0+(nEdge+2)%3];
    SGM::Point2D const &uv0=aPoints2D[a];
    SGM::Point2D const &uv1=aPoints2D[b];
    SGM::Point2D Miduv=SGM::MidPoint(uv0,uv1);
    SGM::Point3D Pos;
    SGM::UnitVector3D Norm;
    pSurface->Evaluate(Miduv,&Pos,NULL,NULL,&Norm);
    size_t m=aPoints2D.size();
    aPoints2D.push_back(Miduv);
    aPoints3D.push_back(Pos);
    aNormals.push_back(Norm);

    // Find the adjacent triangle information
    
    size_t nT1=aAdjacences[nT0+nEdge];
    size_t n0=aTriangles[nT1];
    size_t n1=aTriangles[nT1+1];
    size_t n2=aTriangles[nT1+2];
    size_t d,nA2,nA3;
    if(n0!=a && n0!=b)
        {
        d=n0;
        nA2=aAdjacences[nT1+2];
        nA3=aAdjacences[nT1];
        }
    else if(n1!=a && n1!=b)
        {
        d=n1;
        nA2=aAdjacences[nT1];
        nA3=aAdjacences[nT1+1];
        }
    else
        {
        d=n2;
        nA2=aAdjacences[nT1+1];
        nA3=aAdjacences[nT1+2];
        }

    // Add the new triangles.

    aTriangles[nT0]=a;
    aTriangles[nT0+1]=m;
    aTriangles[nT0+2]=c;

    aTriangles[nT1]=m;
    aTriangles[nT1+1]=b;
    aTriangles[nT1+2]=c;

    size_t nT2=aTriangles.size();
    aTriangles.push_back(m);
    aTriangles.push_back(a);
    aTriangles.push_back(d);

    size_t nT3=aTriangles.size();
    aTriangles.push_back(b);
    aTriangles.push_back(m);
    aTriangles.push_back(d);

    // Fix adjacencies

    size_t nA0=aAdjacences[nT0+(nEdge+1)%3];
    size_t nA1=aAdjacences[nT0+(nEdge+2)%3];

    aAdjacences[nT0]=nT2;
    aAdjacences[nT0+1]=nT1;
    aAdjacences[nT0+2]=nA1;

    aAdjacences[nT1]=nT3;
    aAdjacences[nT1+1]=nA0;
    aAdjacences[nT1+2]=nT0;

    aAdjacences.push_back(nT0);
    aAdjacences.push_back(nA2);
    aAdjacences.push_back(nT3);

    aAdjacences.push_back(nT1);
    aAdjacences.push_back(nT2);
    aAdjacences.push_back(nA3);

    FixBackPointer(nA0,c,nT1,aTriangles,aAdjacences);
    FixBackPointer(nA1,a,nT0,aTriangles,aAdjacences);
    FixBackPointer(nA2,d,nT2,aTriangles,aAdjacences);
    FixBackPointer(nA3,b,nT3,aTriangles,aAdjacences);

    // Fix edge data.

    SGM::UnitVector3D const &NormA=aNormals[a];
    SGM::UnitVector3D const &NormB=aNormals[b];
    SGM::UnitVector3D const &NormC=aNormals[c];
    SGM::UnitVector3D const &NormD=aNormals[d];
    SGM::UnitVector3D const &NormM=aNormals[m];
    
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT0,nEdge)],nT0,nEdge));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT0,(nEdge+1)%3)],nT0,(nEdge+1)%3));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT0,(nEdge+2)%3)],nT0,(nEdge+2)%3));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT1,0)],nT1,0));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT1,1)],nT1,1));
    sEdgeData.erase(EdgeValue(mEdgeData[std::pair<size_t,size_t>(nT1,2)],nT1,2));

    if(aAdjacences[nT0+0]!=SIZE_MAX)
        {
        double dVal=NormA%NormM;
        sEdgeData.insert(EdgeValue(dVal,nT0,0));
        mEdgeData[std::pair<size_t,size_t>(nT0,0)]=dVal;
        }
    if(aAdjacences[nT3+0]!=SIZE_MAX)
        {
        double dVal=NormB%NormM;
        sEdgeData.insert(EdgeValue(dVal,nT3,0));
        mEdgeData[std::pair<size_t,size_t>(nT3,0)]=dVal;
        }
    if(aAdjacences[nT1+2]!=SIZE_MAX)
        {
        double dVal=NormC%NormM;
        sEdgeData.insert(EdgeValue(dVal,nT1,2));
        mEdgeData[std::pair<size_t,size_t>(nT1,2)]=dVal;
        }
    if(aAdjacences[nT2+2]!=SIZE_MAX)
        {
        double dVal=NormD%NormM;
        sEdgeData.insert(EdgeValue(dVal,nT2,2));
        mEdgeData[std::pair<size_t,size_t>(nT2,2)]=dVal;
        }

    if(c<a && aAdjacences[nT0+2]!=SIZE_MAX)
        {
        double dVal=NormC%NormA;
        sEdgeData.insert(EdgeValue(dVal,nT0,2));
        mEdgeData[std::pair<size_t,size_t>(nT0,2)]=dVal;
        }
    if(b<c && aAdjacences[nT1+1]!=SIZE_MAX)
        {
        double dVal=NormB%NormC;
        sEdgeData.insert(EdgeValue(dVal,nT1,1));
        mEdgeData[std::pair<size_t,size_t>(nT1,1)]=dVal;
        }
    if(a<d  && aAdjacences[nT2+1]!=SIZE_MAX)
        {
        double dVal=NormA%NormD;
        sEdgeData.insert(EdgeValue(dVal,nT2,1));
        mEdgeData[std::pair<size_t,size_t>(nT2,1)]=dVal;
        }
    if(d<b && aAdjacences[nT3+2]!=SIZE_MAX)
        {
        double dVal=NormD%NormB;
        sEdgeData.insert(EdgeValue(dVal,nT3,2));
        mEdgeData[std::pair<size_t,size_t>(nT3,2)]=dVal;
        }

    std::set<size_t> sSearchTris;
    sSearchTris.insert(nT0);
    sSearchTris.insert(nT1);
    sSearchTris.insert(nT2);
    sSearchTris.insert(nT3);
    IncrementalDelaunay(sSearchTris,aPoints2D,aNormals,aTriangles,aAdjacences,sEdgeData,mEdgeData);
    }            

void RefineTriangles(face                     const *pFace,
                     FacetOptions             const &Options,
                     std::vector<SGM::Point2D>      &aPoints2D,
                     std::vector<SGM::Point3D>      &aPoints3D,
                     std::vector<SGM::UnitVector3D> &aNormals,
                     std::vector<size_t>            &aTriangles,
                     std::vector<size_t>            &aAdjacences)
    {
    std::set<EdgeValue> sEdgeData;
    std::map<std::pair<size_t,size_t>,double> mEdgeData;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::UnitVector3D const &NormA=aNormals[a];
        SGM::UnitVector3D const &NormB=aNormals[b];
        SGM::UnitVector3D const &NormC=aNormals[c];
        if(a<b && aAdjacences[Index1]!=SIZE_MAX)
            {
            double dVal=NormA%NormB;
            sEdgeData.insert(EdgeValue(NormA%NormB,Index1,0));
            mEdgeData[std::pair<size_t,size_t>(Index1,0)]=dVal;
            }
        if(b<c && aAdjacences[Index1+1]!=SIZE_MAX)
            {
            double dVal=NormB%NormC;
            sEdgeData.insert(EdgeValue(dVal,Index1,1));
            mEdgeData[std::pair<size_t,size_t>(Index1,1)]=dVal;
            }
        if(c<a && aAdjacences[Index1+2]!=SIZE_MAX)
            {
            double dVal=NormC%NormA;
            sEdgeData.insert(EdgeValue(dVal,Index1,2));
            mEdgeData[std::pair<size_t,size_t>(Index1,2)]=dVal;
            }
        }
    double dDotTol=std::cos(Options.m_dFaceAngleTol);
    dDotTol=0.93969262078590838405410927732473;
    bool bSplit=true;
    size_t nCount=0;
    while(bSplit && sEdgeData.size())
        {
        EdgeValue EV=*(sEdgeData.begin());
        if(EV.m_dDot<dDotTol)
            {
            SplitEdge(EV,pFace,aPoints2D,aPoints3D,aNormals,aTriangles,aAdjacences,sEdgeData,mEdgeData);
            bSplit=true;
            }
        else
            {
            bSplit=false;
            }
        ++nCount;
        }
    }

bool FlipIt(std::vector<SGM::Point3D>      const &aPoints3D,
            size_t                                a,
            size_t                                b,
            size_t                                c,
            size_t                                d)
    {
    SGM::Point3D const &A=aPoints3D[a];
    SGM::Point3D const &B=aPoints3D[b];
    SGM::Point3D const &C=aPoints3D[c];
    SGM::Point3D const &D=aPoints3D[d];

    // CD has to be shorter than AB and 
    // The angle at C with A and B and the
    // angle at D with A and B should be obtuse.

    double CD=C.DistanceSquared(D);
    double AB=A.DistanceSquared(B);
    if(AB<CD)
        {
        return false;
        }
    if(0<(A-C)%(B-C) || 0<(A-D)%(B-D))
        {
        return false;
        }
    return true;
    }

bool Flip3D(std::vector<SGM::Point3D>      const &aPoints3D,
            std::vector<size_t>                  &aTriangles,
            size_t                                T0,
            size_t                                nEdge,
            size_t                                T1)
    {
    size_t a=aTriangles[T0];
    size_t b=aTriangles[T0+1];
    size_t c=aTriangles[T0+2];
    size_t d=aTriangles[T1];
    size_t e=aTriangles[T1+1];
    size_t f=aTriangles[T1+2];
    if(nEdge==0)
        {
        if(d!=a && d!=b)
            {
            if(FlipIt(aPoints3D,a,b,c,d))
                {
                return true;
                }
            }
        else if(e!=a && e!=b)
            {
            if(FlipIt(aPoints3D,a,b,c,e))
                {
                return true;
                }
            }
        else
            {
            if(FlipIt(aPoints3D,a,b,c,f))
                {
                return true;
                }
            }
        }
    else if(nEdge==1)
        {
        if(d!=c && d!=b)
            {
            if(FlipIt(aPoints3D,c,b,a,d))
                {
                return true;
                }
            }
        else if(e!=c && e!=b)
            {
            if(FlipIt(aPoints3D,c,b,a,e))
                {
                return true;
                }
            }
        else
            {
            if(FlipIt(aPoints3D,c,b,a,f))
                {
                return true;
                }
            }
        }
    else
        {
        if(d!=a && d!=c)
            {
            if(FlipIt(aPoints3D,c,a,b,d))
                {
                return true;
                }
            }
        else if(e!=a && e!=c)
            {
            if(FlipIt(aPoints3D,c,a,b,e))
                {
                return true;
                }
            }
        else
            {
            if(FlipIt(aPoints3D,c,a,b,f))
                {
                return true;
                }
            }
        }
    return false;
    }

void FlipTriangles(size_t               nTri,
                   size_t               nEdge,
                   std::vector<size_t> &aTriangles,
                   std::vector<size_t> &aAdjacences)
    {
    size_t a=aTriangles[nTri];
    size_t b=aTriangles[nTri+1];
    size_t c=aTriangles[nTri+2];
    size_t nT=aAdjacences[nTri+nEdge];
    size_t d=aTriangles[nT];
    size_t e=aTriangles[nT+1];
    size_t f=aTriangles[nT+2];
    size_t g,nTA,nTB;
    if(d!=a && d!=b && d!=c)
        {
        g=d;
        nTA=aAdjacences[nT+2];
        nTB=aAdjacences[nT];
        }
    else if(e!=a && e!=b && e!=c)
        {
        g=e;
        nTA=aAdjacences[nT];
        nTB=aAdjacences[nT+1];
        }
    else
        {
        g=f;
        nTA=aAdjacences[nT+1];
        nTB=aAdjacences[nT+2];
        }
    
    size_t nT0=aAdjacences[nTri];
    size_t nT1=aAdjacences[nTri+1];
    size_t nT2=aAdjacences[nTri+2];
    if(nEdge==0)
        {
        aTriangles[nTri]=g;
        aTriangles[nTri+1]=c;
        aTriangles[nTri+2]=a;
        aTriangles[nT]=g;
        aTriangles[nT+1]=b;
        aTriangles[nT+2]=c;

        aAdjacences[nTri]=nT;
        aAdjacences[nTri+1]=nT2;
        aAdjacences[nTri+2]=nTA;
        aAdjacences[nT]=nTB;
        aAdjacences[nT+1]=nT1;
        aAdjacences[nT+2]=nTri;
        }
    else if(nEdge==1)
        {
        aTriangles[nTri]=g;
        aTriangles[nTri+1]=a;
        aTriangles[nTri+2]=b;
        aTriangles[nT]=g;
        aTriangles[nT+1]=c;
        aTriangles[nT+2]=a;

        aAdjacences[nTri]=nT;
        aAdjacences[nTri+1]=nT0;
        aAdjacences[nTri+2]=nTA;
        aAdjacences[nT]=nTB;
        aAdjacences[nT+1]=nT2;
        aAdjacences[nT+2]=nTri;
        }
    else
        {
        aTriangles[nTri]=g;
        aTriangles[nTri+1]=a;
        aTriangles[nTri+2]=b;
        aTriangles[nT]=g;
        aTriangles[nT+1]=b;
        aTriangles[nT+2]=c;

        aAdjacences[nTri]=nTB;
        aAdjacences[nTri+1]=nT0;
        aAdjacences[nTri+2]=nT;
        aAdjacences[nT]=nTri;
        aAdjacences[nT+1]=nT1;
        aAdjacences[nT+2]=nTA;
        }
    FixBackPointers(nT,aTriangles,aAdjacences);
    FixBackPointers(nTri,aTriangles,aAdjacences);
    }

void DelaunayFlips3D(std::vector<SGM::Point3D>      const &aPoints3D,
                     std::vector<size_t>                  &aTriangles,
                     std::vector<size_t>                  &aAdjacences)
    {
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t T0=aAdjacences[Index1];
        size_t T1=aAdjacences[Index1+1];
        size_t T2=aAdjacences[Index1+2];
        if(T0!=SIZE_MAX && Flip3D(aPoints3D,aTriangles,Index1,0,T0))
            {
            FlipTriangles(Index1,0,aTriangles,aAdjacences);
            T1=aAdjacences[Index1+1];
            T2=aAdjacences[Index1+2];
            }
        else if(T1!=SIZE_MAX && Flip3D(aPoints3D,aTriangles,Index1,1,T1))
            {
            FlipTriangles(Index1,1,aTriangles,aAdjacences);
            T2=aAdjacences[Index1+2];
            }
        else if(T2!=SIZE_MAX && Flip3D(aPoints3D,aTriangles,Index1,2,T2))
            {
            FlipTriangles(Index1,2,aTriangles,aAdjacences);
            }
        }
    }

void StarSmoothing(face                     const *pFace,
                   std::vector<SGM::Point2D>      &aPoints2D,
                   std::vector<SGM::Point3D>      &aPoints3D,
                   std::vector<SGM::UnitVector3D> &aNormals,
                   std::vector<size_t>            &aTriangles,
                   size_t                          nStiner)
    {
    size_t nPoints=aPoints2D.size();
    if(nStiner<nPoints)
        {
        std::set<size_t> sVertices;
        std::set<GraphEdge> sEdges;
        size_t Index1,Index2;
        for(Index1=0;Index1<nPoints;++Index1)
            {
            sVertices.insert(Index1);
            }
        size_t nTriangles=aTriangles.size();
        size_t nCount=0;
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            size_t a=aTriangles[Index1];
            size_t b=aTriangles[Index1+1];
            size_t c=aTriangles[Index1+2];
            if(a<b)
                {
                sEdges.insert(GraphEdge(a,b,nCount));
                ++nCount;
                }
            if(b<c)
                {
                sEdges.insert(GraphEdge(b,c,nCount));
                ++nCount;
                }
            if(c<a)
                {
                sEdges.insert(GraphEdge(c,a,nCount));
                ++nCount;
                }
            }
        Graph graph(sVertices,sEdges);
        surface const *pSurface=pFace->GetSurface();
        for(Index1=nStiner;Index1<nPoints;++Index1)
            {
            std::vector<size_t> const &aStar=graph.GetStar(Index1);
            size_t nStar=aStar.size();
            double dUTotal=0,dVTotal=0;
            for(Index2=0;Index2<nStar;++Index2)
                {
                SGM::Point2D const &uv=aPoints2D[aStar[Index2]];
                dUTotal+=uv.m_u;
                dVTotal+=uv.m_v;
                }
            SGM::Point2D Center(dUTotal/nStar,dVTotal/nStar);
            aPoints2D[Index1]=Center;
            SGM::Point3D Pos;
            SGM::UnitVector3D Norm;
            pSurface->Evaluate(Center,&Pos,NULL,NULL,&Norm);
            aPoints3D[Index1]=Pos;
            aNormals[Index1]=Norm;
            }
        }
    }

void FacetFace(SGM::Result                    &rResult,
               face                     const *pFace,
               FacetOptions             const &Options,
               std::vector<SGM::Point2D>      &aPoints2D,
               std::vector<SGM::Point3D>      &aPoints3D,
               std::vector<SGM::UnitVector3D> &aNormals,
               std::vector<size_t>            &aTriangles,
               std::vector<entity *>          &aEntities)
    {
    std::vector<std::vector<size_t> > aaPolygons;
    std::vector<size_t> aAdjacences;
    std::vector<size_t> aPoles;
    FacetFaceLoops(rResult,pFace,Options,aPoints2D,aPoints3D,aEntities,aaPolygons,aPoles);
    SGM::TriangulatePolygon(rResult,aPoints2D,aaPolygons,aTriangles,aAdjacences);
    size_t nOldSize=aPoints2D.size();
    surface const *pSurface=pFace->GetSurface();
    size_t Index1;
    for(Index1=0;Index1<nOldSize;++Index1)
        {
        SGM::Point2D const &uv=aPoints2D[Index1];
        SGM::UnitVector3D Norm;
        pSurface->Evaluate(uv,NULL,NULL,NULL,&Norm);
        aNormals.push_back(Norm);
        }
    RefineTriangles(pFace,Options,aPoints2D,aPoints3D,aNormals,aTriangles,aAdjacences);
    DelaunayFlips3D(aPoints3D,aTriangles,aAdjacences);
    size_t nPoles=aPoles.size();
    for(Index1=0;Index1<nPoles;++Index1)
        {
        aTriangles.push_back(aPoles[Index1]);
        }
    /*
    for(Index1=0;Index1<5;++Index1)
        {
        StarSmoothing(pFace,aPoints2D,aPoints3D,aNormals,aTriangles,nOldSize);
        }
    */
    }


