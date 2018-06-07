#include "SGMMathematics.h"
#include "SGMVector.h"

#include "EntityClasses.h"
#include "Faceter.h"
#include "Graph.h"
#include "Topology.h"
#include "Surface.h"
#include "Curve.h"
#include "Query.h"

#include <cfloat>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////
//
//  face methods
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{
face::face(SGM::Result &rResult):topology(rResult,SGM::EntityType::FaceType),
    m_pVolume(nullptr),m_pSurface(nullptr),m_bFlipped(false),m_nSides(1)
    {
    }

volume *face::GetVolume() const 
    {
    return m_pVolume;
    }

std::vector<SGM::Point2D> const &face::GetPoints2D(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles,m_aEntities);
        }
    return m_aPoints2D;
    }

std::vector<SGM::Point3D> const &face::GetPoints3D(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles,m_aEntities);
        }
    return m_aPoints3D;
    }

std::vector<size_t> const &face::GetTriangles(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles,m_aEntities);
        }
    return m_aTriangles;
    }

std::vector<SGM::UnitVector3D> const &face::GetNormals(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles,m_aEntities);
        }
    return m_aNormals;
    }

bool face::PointInFace(SGM::Result        &rResult,
                       SGM::Point2D const &uv,
                       SGM::Point3D       *ClosePos,    // The closest point.
                       entity            **pBoundary,   // The closest sub element.
                       SGM::Point3D       *pPos) const  // Found point on boundary.
    {
    // First check for the closed face case.

    if(m_sEdges.empty())
        {
        return true;
        }

    // Find the closest edge or vertex.

    SGM::Point3D Pos,CPos,TPos;
    entity *pEntity= (entity *)this;
    double dDist=DBL_MAX;
    if(ClosePos)
        {
        Pos=*ClosePos;
        }
    else
        {
        m_pSurface->Evaluate(uv,&Pos);
        }
    std::set<edge *>::iterator iter=m_sEdges.begin();
    while(iter!=m_sEdges.end())
        {
        edge *pEdge=*iter;
        entity *pTempEnt=nullptr;
        FindClosestPointOnEdge(rResult,Pos,pEdge,TPos,pTempEnt);
        double dTempDist=Pos.DistanceSquared(TPos);
        if(dTempDist<dDist)
            {
            dDist=dTempDist;
            pEntity=pTempEnt;
            CPos=TPos;
            }
        ++iter;
        }

    // Find the parameter line to check sided-ness from.

    if(pBoundary)
        {
        *pBoundary=pEntity;
        }
    SGM::Point2D Buv=m_pSurface->Inverse(CPos);
    if(pEntity->GetType()==SGM::EntityType::EdgeType)
        {
        SGM::Vector3D VecU,VecV,Vec;
        m_pSurface->Evaluate(Buv,nullptr,&VecU,&VecV);
        edge *pEdge=(edge *)pEntity;
        curve const *pCurve=pEdge->GetCurve();
        double t=pCurve->Inverse(CPos);
        if(pPos)
            {
            *pPos=CPos;
            }
        pCurve->Evaluate(t,nullptr,&Vec);
        SGM::Vector2D VecUV(Vec%VecU,Vec%VecV);
        SGM::Vector2D TestVec=uv-Buv;
        double dZ=VecUV.m_u*TestVec.m_v-VecUV.m_v*TestVec.m_u;
        std::map<edge *,SGM::EdgeSideType>::const_iterator EdgeTypeIter=m_mFaceType.find(pEdge);
        if(EdgeTypeIter->second==SGM::FaceOnRightType)
            {
            dZ=-dZ;
            }
        if(SGM_ZERO<=dZ)
            {
            return m_bFlipped==true ? false : true;
            }
        else if(-SGM_ZERO>=dZ)
            {
            return m_bFlipped==true ? true : false;
            }
        else
            {
            return true;
            }
        }
    else // The vertex case.
        {
        SGM::Point3D const &VertexPos=((vertex *)pEntity)->GetPoint();
        if(pPos)
            {
            *pPos=VertexPos;
            }
        if(SGM::NearEqual(VertexPos,Pos,SGM_ZERO))
            {
            return true;
            }
        else
            {
            // Have to find inside out side from one of the edges 
            // or the angle of the edges.
            return false;
            }
        return false;
        }
    }

double TriangleArea(surface      const *pSurface,
                    SGM::Point3D const &A,
                    SGM::Point3D const &B,
                    SGM::Point3D const &C,
                    SGM::Point2D const &a,
                    SGM::Point2D const &b,
                    SGM::Point2D const &c)
    {
    SGM::Point3D AAB,AB,ABB,AAC,AC,ACC,BBC,BC,BCC,ABBC,ACBC,ABAC;
    SGM::Point2D aab,ab,abb,aac,ac,acc,bbc,bc,bcc,abbc,acbc,abac;

    aab=SGM::MidPoint(a,b,0.25);
    ab =SGM::MidPoint(a,b,0.5);
    abb=SGM::MidPoint(a,b,0.75);

    pSurface->Evaluate(aab,&AAB);
    pSurface->Evaluate(ab ,&AB );
    pSurface->Evaluate(abb,&ABB);

    aac=SGM::MidPoint(a,c,0.25);
    ac =SGM::MidPoint(a,c,0.5);
    acc=SGM::MidPoint(a,c,0.75);

    pSurface->Evaluate(aac,&AAC);
    pSurface->Evaluate(ac ,&AC );
    pSurface->Evaluate(acc,&ACC);

    bbc=SGM::MidPoint(b,c,0.25);
    bc =SGM::MidPoint(b,c,0.5);
    bcc=SGM::MidPoint(b,c,0.75);

    pSurface->Evaluate(bbc,&BBC);
    pSurface->Evaluate(bc ,&BC );
    pSurface->Evaluate(bcc,&BCC);

    abbc=SGM::MidPoint(ab,bc);
    acbc=SGM::MidPoint(ac,bc);
    abac=SGM::MidPoint(ab,ac);

    pSurface->Evaluate(abbc,&ABBC);
    pSurface->Evaluate(acbc,&ACBC);
    pSurface->Evaluate(abac,&ABAC);

    double dArea=0;
    dArea+=((AAB-A)*(AAC-A)).Magnitude();

    dArea+=((AB-AAB)*(ABAC-AAB)).Magnitude();
    dArea+=((AAB-ABAC)*(AAC-ABAC)).Magnitude();
    dArea+=((ABAC-AAC)*(AC-AAC)).Magnitude();

    dArea+=((ABB-AB)*(ABBC-AB)).Magnitude();
    dArea+=((AB-ABBC)*(ABAC-ABBC)).Magnitude();
    dArea+=((ABBC-ABAC)*(ACBC-ABAC)).Magnitude();
    dArea+=((ABAC-ACBC)*(AC-ACBC)).Magnitude();
    dArea+=((ACBC-AC)*(ACC-AC)).Magnitude();

    dArea+=((ABB-B)*(BBC-B)).Magnitude();
    dArea+=((ABB-BBC)*(ABBC-BBC)).Magnitude();
    dArea+=((BBC-ABBC)*(BC-ABBC)).Magnitude();
    dArea+=((BBC-BC)*(ABBC-BC)).Magnitude();
    dArea+=((BCC-BC)*(ACBC-BC)).Magnitude();
    dArea+=((BCC-ACBC)*(ACC-ACBC)).Magnitude();
    dArea+=((ACC-C)*(BCC-C)).Magnitude();

    return dArea*0.5;
    }

double face::FindArea(SGM::Result &rResult) const
    {
    // Method one. 
    // Using Romberg integration on a reduced set of parametric triangles.

    SGMInternal::FacetOptions Options;
    Options.m_bParametric=true;
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<size_t> aTriangles;
    std::vector<SGM::UnitVector3D> aNormals;
    std::vector<entity *> aEntities;
    SGMInternal::FacetFace(rResult,this,Options,aPoints2D,aPoints3D,aNormals,aTriangles,aEntities);
    size_t nTriangles=aTriangles.size();
    double dMethod1=0;
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        SGM::Point2D const &PosA=aPoints2D[a];
        SGM::Point2D const &PosB=aPoints2D[b];
        SGM::Point2D const &PosC=aPoints2D[c];
        dMethod1+=m_pSurface->FindAreaOfParametricTriangle(rResult,PosA,PosB,PosC);
        }

    // Method two.  
    // Refining the full set of parametric triangles twice and taking their area.

    GetPoints2D(rResult);
    nTriangles=m_aTriangles.size();
    double dMethod2=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=m_aTriangles[Index1];
        size_t b=m_aTriangles[Index1+1];
        size_t c=m_aTriangles[Index1+2];
        SGM::Point2D const &PosA=m_aPoints2D[a];
        SGM::Point2D const &PosB=m_aPoints2D[b];
        SGM::Point2D const &PosC=m_aPoints2D[c];
        SGM::Point3D const &A=m_aPoints3D[a];
        SGM::Point3D const &B=m_aPoints3D[b];
        SGM::Point3D const &C=m_aPoints3D[c];
        dMethod2+=TriangleArea(m_pSurface,A,B,C,PosA,PosB,PosC);
        }

    return std::max(dMethod1,dMethod2);
    }

size_t face::FindLoops(SGM::Result                                  &rResult,
                       std::vector<std::vector<edge *> >            &aaLoops,
                       std::vector<std::vector<SGM::EdgeSideType> > &aaFlipped) const
    {
    thing *pThing=rResult.GetThing();
    Graph graph(rResult,m_sEdges);
    std::vector<Graph> aComponents;
    size_t nLoops=graph.FindComponents(aComponents);
    aaLoops.reserve(nLoops);
    aaFlipped.reserve(nLoops);
    std::vector<std::vector<edge *> > aaTempLoops;
    std::vector<std::vector<SGM::EdgeSideType> > aaTempFlipped;
    aaTempLoops.reserve(nLoops);
    aaTempFlipped.reserve(nLoops);

    size_t Index1;
    for(Index1=0;Index1<nLoops;++Index1)
        {
        Graph const &comp=aComponents[Index1];
        std::set<GraphEdge> const &sGEdges=comp.GetEdges();
        std::set<edge *> sEdges;
        std::set<GraphEdge>::const_iterator GEdgeIter=sGEdges.begin();
        while(GEdgeIter!=sGEdges.end())
            {
            size_t ID=GEdgeIter->m_nID;
            sEdges.insert((edge *)pThing->FindEntity(ID));
            ++GEdgeIter;
            }
        std::vector<edge *> aEdges;
        std::vector<SGM::EdgeSideType> aFlips;
        OrderLoopEdges(rResult,this,sEdges,aEdges,aFlips);
        aaTempLoops.push_back(aEdges);
        aaTempFlipped.push_back(aFlips);
        }

    // Order the loops by start ID of the first edge.

    std::vector<std::pair<size_t,size_t> > aOrder;
    aOrder.reserve(nLoops);
    for(Index1=0;Index1<nLoops;++Index1)
        {
        aOrder.push_back(std::pair<size_t,size_t>(aaTempLoops[Index1][0]->GetID(),Index1));
        }
    std::sort(aaTempLoops.begin(),aaTempLoops.end());
    for(Index1=0;Index1<nLoops;++Index1)
        {
        aaLoops.push_back(aaTempLoops[aOrder[Index1].second]);
        aaFlipped.push_back(aaTempFlipped[aOrder[Index1].second]);
        }

    return nLoops;
    }

void face::AddEdge(edge *pEdge,SGM::EdgeSideType nEdgeType)
    {
    m_sEdges.insert(pEdge);
    pEdge->AddFace(this);
    m_mFaceType[pEdge]=nEdgeType;
    }

void face::SetSurface(surface *pSurface)
    {
    if(m_pSurface)
        {
        m_pSurface->RemoveFace(this);
        }
    m_pSurface=pSurface;
    pSurface->AddFace(this);
    }

SGM::EdgeSideType face::GetEdgeType(edge const *pEdge) const 
    {
    std::map<edge *,SGM::EdgeSideType>::const_iterator iter=m_mFaceType.find((edge *)pEdge);
    return iter->second;
    }

bool face::GetFlipped() const
    {
    return m_bFlipped;
    }
}