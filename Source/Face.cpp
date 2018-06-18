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
    std::set<edge *,EntityCompare>::iterator iter=m_sEdges.begin();
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
        std::map<edge *,SGM::EdgeSideType>::const_iterator EdgeTypeIter=m_mSideType.find(pEdge);
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

double TriangleArea(std::vector<SGM::Point3D> const &aPoints3D,
                    std::vector<size_t>       const &aTriangles)
    {
    double dArea=0.0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        dArea+=((aPoints3D[b]-aPoints3D[a])*(aPoints3D[c]-aPoints3D[a])).Magnitude();
        }
    return dArea*0.5;
    }

void face::ClearFacets(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty()==false)
        {
        m_aPoints3D.clear();
        m_aPoints2D.clear();
        m_aEntities.clear();
        m_aTriangles.clear();
        m_aNormals.clear();
        m_Box.Reset();
        m_mSeamType.clear();
        if(m_pVolume)
            {
            m_pVolume->ClearBox(rResult);
            }
        }
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

    this->GetPoints2D(rResult);
    aPoints2D=m_aPoints2D;
    aPoints3D=m_aPoints3D;
    aTriangles=m_aTriangles;
    aEntities=m_aEntities;

    double dDiff=SGM_MAX;
    double dOldArea=SGM_MAX;
    double dArea2=0;
    double dArea0=TriangleArea(aPoints3D,aTriangles);
    size_t nCount=0;
    while(SGM_MIN_TOL<dDiff && nCount<4)
        {
        SubdivideFacets(rResult,this,aPoints3D,aPoints2D,aTriangles,aEntities);
        double dArea1=TriangleArea(aPoints3D,aTriangles);
        dArea2=(4*dArea1-dArea0)/3;
        dArea0=dArea1;
        dDiff=fabs(dArea2-dOldArea);
        dOldArea=dArea2;
        ++nCount;
        }

    return std::max(dMethod1,dArea2);
    }

double FindLocalVolume(std::vector<SGM::Point3D> const &aPoints,
                       std::vector<size_t>       const &aTriangles)
    {
    double dAnswer=0;
    size_t nTriangles=aTriangles.size();
    size_t Index1;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        SGM::Point3D const &A=aPoints[aTriangles[Index1]];
        SGM::Point3D const &B=aPoints[aTriangles[Index1+1]];
        SGM::Point3D const &C=aPoints[aTriangles[Index1+2]];
        dAnswer+=A.m_x*(B.m_y*C.m_z-B.m_z*C.m_y)+A.m_y*(B.m_z*C.m_x-B.m_x*C.m_z)+A.m_z*(B.m_x*C.m_y-B.m_y*C.m_x);
        }
    return dAnswer;
    }

double face::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    std::vector<SGM::Point2D> aPoints2D=GetPoints2D(rResult);
    std::vector<SGM::Point3D> aPoints3D=m_aPoints3D;
    std::vector<entity *> aEntities=m_aEntities;
    std::vector<size_t> aTriangles=m_aTriangles;
    double dAnswer0=FindLocalVolume(aPoints3D,aTriangles);
    if(bApproximate)
        {
        return dAnswer0;
        }

    double dVolume=dAnswer0;
    size_t Index1;
    for(Index1=0;Index1<2;++Index1)
        {
        SubdivideFacets(rResult,this,aPoints3D,aPoints2D,aTriangles,aEntities);
        double dAnswer1=FindLocalVolume(aPoints3D,aTriangles);
        dVolume=(4*dAnswer1-dAnswer0)/3;
        dAnswer0=dAnswer1;
        }
    return dVolume;
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
        std::set<edge *,EntityCompare> sEdges;
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
        aOrder.emplace_back(std::pair<size_t,size_t>(aaTempLoops[Index1][0]->GetID(),Index1));
        }
    std::sort(aOrder.begin(),aOrder.end());
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
    m_mSideType[pEdge]=nEdgeType;
    }

void face::RemoveEdge(SGM::Result &rResult,
                      edge        *pEdge)
    {
    m_sEdges.erase(pEdge);
    m_mSideType.erase(pEdge);
    m_mSeamType.erase(pEdge);
    pEdge->RemoveFace((face *)this);
    ClearFacets(rResult);
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

SGM::EdgeSideType face::GetSideType(edge const *pEdge) const 
    {
    std::map<edge *,SGM::EdgeSideType>::const_iterator iter=m_mSideType.find((edge *)pEdge);
    return iter->second;
    }

void face::TransformFacets(SGM::Transform3D const &Trans)
    {
    size_t nPoints=m_aPoints3D.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        m_aPoints3D[Index1]*=Trans;
        m_aNormals[Index1]*=Trans;
        }
    }

bool face::GetFlipped() const
    {
    return m_bFlipped;
    }

SGM::EdgeSeamType FindEdgeSeamType(edge const *pEdge,
                                   face const *pFace)
    {
    surface const *pSurface=pFace->GetSurface();
    if(pSurface->ClosedInU() || pSurface->ClosedInV())
        {
        SGM::Point3D MidPos=pEdge->FindMidPoint();
        SGM::Point2D uv=pSurface->Inverse(MidPos);
        SGM::Interval2D const &Domain=pSurface->GetDomain();
        if(Domain.OnBoundary(uv,SGM_MIN_TOL))
            {
            SGM::Point3D MidPosPlus=pEdge->FindMidPoint(0.501);
            SGM::Point2D uvPlus=pSurface->Inverse(MidPosPlus);
            if(Domain.OnBoundary(uvPlus,SGM_MIN_TOL))
                {
                if(pSurface->ClosedInU() && Domain.m_UDomain.OnBoundary(uv.m_u,SGM_MIN_TOL))
                    {
                    if(uv.m_v<uvPlus.m_v)
                        {
                        return SGM::EdgeSeamType::UpperUSeamType;
                        }
                    else
                        {
                        return SGM::EdgeSeamType::LowerUSeamType;
                        }
                    }
                if(pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL))
                    {
                    if(uv.m_u<uvPlus.m_u)
                        {
                        return SGM::EdgeSeamType::LowerVSeamType;
                        }
                    else
                        {
                        return SGM::EdgeSeamType::UpperVSeamType;
                        }
                    }
                }
            }
        }
    return SGM::EdgeSeamType::NotASeamType;
    }

SGM::EdgeSeamType face::GetSeamType(edge const *pEdge) const 
    {
    std::map<edge *,SGM::EdgeSeamType>::const_iterator iter=m_mSeamType.find((edge *)pEdge);
    if(iter==m_mSeamType.end())
        {
        SGM::EdgeSeamType nSeamType=FindEdgeSeamType(pEdge,this);
        m_mSeamType[(edge *)pEdge]=nSeamType;
        return nSeamType;
        }
    return iter->second;
    }

SGM::Point2D face::EvaluateParamSpace(edge         const *pEdge,
                                      SGM::EdgeSideType   nType,
                                      SGM::Point3D const &Pos) const
    {
    SGM::EdgeSeamType nSeamType=GetSeamType(pEdge);
    SGM::Point2D uv=m_pSurface->Inverse(Pos);
    SGM::Interval2D const &Domain=m_pSurface->GetDomain();
    if(Domain.OnBoundary(uv,SGM_MIN_TOL))
        {
        if(nSeamType!=SGM::EdgeSeamType::NotASeamType)
            {
            if(m_bFlipped)
                {
                if(nSeamType==SGM::EdgeSeamType::UpperUSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_u=Domain.m_UDomain.m_dMax;
                        }
                    else
                        {
                        uv.m_u=Domain.m_UDomain.m_dMin;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::UpperVSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_v=Domain.m_VDomain.m_dMax;
                        }
                    else
                        {
                        uv.m_v=Domain.m_VDomain.m_dMin;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::LowerUSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_u=Domain.m_UDomain.m_dMin;
                        }
                    else
                        {
                        uv.m_u=Domain.m_UDomain.m_dMax;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::LowerVSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_v=Domain.m_VDomain.m_dMin;
                        }
                    else
                        {
                        uv.m_v=Domain.m_VDomain.m_dMax;
                        }
                    }
                }
            else
                {
                if(nSeamType==SGM::EdgeSeamType::UpperUSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_u=Domain.m_UDomain.m_dMin;
                        }
                    else
                        {
                        uv.m_u=Domain.m_UDomain.m_dMax;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::UpperVSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_v=Domain.m_VDomain.m_dMin;
                        }
                    else
                        {
                        uv.m_v=Domain.m_VDomain.m_dMax;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::LowerUSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_u=Domain.m_UDomain.m_dMax;
                        }
                    else
                        {
                        uv.m_u=Domain.m_UDomain.m_dMin;
                        }
                    }
                else if(nSeamType==SGM::EdgeSeamType::LowerVSeamType)
                    {
                    if(nType==SGM::EdgeSideType::FaceOnRightType)
                        {
                        uv.m_v=Domain.m_VDomain.m_dMax;
                        }
                    else
                        {
                        uv.m_v=Domain.m_VDomain.m_dMin;
                        }
                    }
                }
            }
        else if((m_pSurface->ClosedInU() && Domain.m_UDomain.OnBoundary(uv.m_u,SGM_MIN_TOL)) ||
                (m_pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL)))
            {
            double t=pEdge->GetCurve()->Inverse(Pos);
            SGM::Interval1D const &EdgeDomain=pEdge->GetDomain();
            pEdge->SnapToDomain(t,SGM_MIN_TOL);
            double dFraction=EdgeDomain.Fraction(t);
            double t2;
            if(0.5<dFraction)
                {
                t2=dFraction-SGM_FIT;
                }
            else
                {
                t2=dFraction+SGM_FIT;
                }
            SGM::Point3D TestPos=pEdge->FindMidPoint(t2);
            SGM::Point2D TestUV=m_pSurface->Inverse(TestPos);
            if(m_pSurface->ClosedInU() && Domain.m_UDomain.OnBoundary(uv.m_u,SGM_MIN_TOL))
                {
                if(TestUV.m_u<Domain.m_UDomain.MidPoint())
                    {
                    uv.m_u=Domain.m_UDomain.m_dMin;
                    }
                else
                    {
                    uv.m_u=Domain.m_UDomain.m_dMax;
                    }
                }
            if(m_pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL))
                {
                if(TestUV.m_v<Domain.m_VDomain.MidPoint())
                    {
                    uv.m_v=Domain.m_VDomain.m_dMin;
                    }
                else
                    {
                    uv.m_v=Domain.m_VDomain.m_dMax;
                    }
                }
            }

        // Deal with the case of a point at (0,0) on a torus.

        if( m_pSurface->ClosedInU() && 
            m_pSurface->ClosedInV() &&
            Domain.OnCorner(uv,SGM_MIN_TOL))
            {
            double t=pEdge->GetCurve()->Inverse(Pos);
            SGM::Interval1D const &EdgeDomain=pEdge->GetDomain();
            pEdge->SnapToDomain(t,SGM_MIN_TOL);
            double dFraction=EdgeDomain.Fraction(t);
            double t2;
            if(0.5<dFraction)
                {
                t2=dFraction-SGM_FIT;
                }
            else
                {
                t2=dFraction+SGM_FIT;
                }
            SGM::Point3D TestPos=pEdge->FindMidPoint(t2);
            SGM::Point2D TestUV=EvaluateParamSpace(pEdge,nType,TestPos);
            uv.m_u=TestUV.m_u<Domain.m_UDomain.MidPoint() ? Domain.m_UDomain.m_dMin : Domain.m_UDomain.m_dMax;
            uv.m_v=TestUV.m_v<Domain.m_VDomain.MidPoint() ? Domain.m_VDomain.m_dMin : Domain.m_VDomain.m_dMax;
            }
        }
    return uv;
    }

} // End SGMInternal namespace