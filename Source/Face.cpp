#include "SGMMathematics.h"
#include "SGMVector.h"
#include "SGMGraph.h"

#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Faceter.h"
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

void face::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(GetSurface());
    for (auto pEdge : GetEdges())
        {
        sChildren.insert(pEdge);
        pEdge->FindAllChildren(sChildren);
        }
    }

SGM::Interval3D const &face::GetBox(SGM::Result &rResult) const
    {
    if (m_Box.IsEmpty())
        {
        switch(GetSurface()->GetSurfaceType())
            {
            case SGM::EntityType::ConeType:
            case SGM::EntityType::CylinderType:
            case SGM::EntityType::PlaneType:
                {
                // Only use the edge boxes.
                auto sEdges = GetEdges();
                StretchBox(rResult,m_Box,sEdges.begin(),sEdges.end());
                break;
                }
            default:
                {
                // Use all the points.
                auto aPoints = GetPoints3D(rResult);
                auto aTriangles = GetTriangles(rResult);
                double dMaxLength = SGM::FindMaxEdgeLength3D(aPoints,aTriangles);
                m_Box = SGM::Interval3D(aPoints);
                m_Box=m_Box.Extend(sqrt(dMaxLength)*FACET_HALF_TANGENT_OF_FACET_FACE_ANGLE);
                }
            }
        }
    return m_Box;
    }

bool face::GetColor(int &nRed,int &nGreen,int &nBlue) const
    {
    if(entity::GetColor(nRed, nGreen, nBlue))
        {
        return true;
        }

    volume* pVolume = GetVolume();
    if (pVolume)
        return pVolume->GetColor(nRed,nGreen,nBlue);
    else
        return entity::GetColor(nRed,nGreen,nBlue);
    }

void face::SeverRelations(SGM::Result &)
    {
    if(GetVolume())
        GetVolume()->RemoveFace(this);
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(edge *pEdge : sEdges)
        pEdge->RemoveFace(this);
    if(GetSurface())
        SetSurface(nullptr);
    RemoveAllOwners();
    }

void face::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.

    std::map<edge *,SGM::EdgeSeamType> m_sFixedSeamType;
    for(auto SeamType : m_mSeamType)
        {
        auto MapValue=mEntityMap.find(SeamType.first);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedSeamType[(edge *)MapValue->second]=SeamType.second;
            }
        else
            {
            m_sFixedSeamType[SeamType.first]=SeamType.second;
            }
        }
    m_mSeamType=m_sFixedSeamType;

    std::map<edge *,SGM::EdgeSideType> m_sFixedSideType;
    for(auto SideType : m_mSideType)
        {
        auto MapValue=mEntityMap.find(SideType.first);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedSideType[(edge *)MapValue->second]=SideType.second;
            }
        else
            {
            m_sFixedSideType[SideType.first]=SideType.second;
            }
        }
    m_mSideType=m_sFixedSideType;

    if(m_pVolume)
        {
        auto MapValue=mEntityMap.find(m_pVolume);
        if(MapValue!=mEntityMap.end())
            {
            m_pVolume=(volume *)MapValue->second;
            }
        }

    if(m_pSurface)
        {
        auto MapValue=mEntityMap.find(m_pSurface);
        if(MapValue!=mEntityMap.end())
            {
            m_pSurface=(surface *)MapValue->second;
            }
        }

    std::set<edge *,EntityCompare> m_sFixedEdges;
    for(auto pEdge : m_sEdges)
        {
        auto MapValue=mEntityMap.find(pEdge);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedEdges.insert((edge *)MapValue->second);
            }
        else
            {
            m_sFixedEdges.insert(pEdge);
            }
        }
    m_sEdges=m_sFixedEdges;
    OwnerAndAttributeReplacePointers(mEntityMap);
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
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
        }
    return m_aPoints2D;
    }

std::vector<SGM::Point3D> const &face::GetPoints3D(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
        }
    return m_aPoints3D;
    }

std::vector<unsigned int> const &face::GetTriangles(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
        }
    return m_aTriangles;
    }

std::vector<SGM::UnitVector3D> const &face::GetNormals(SGM::Result &rResult) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
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
        FindClosestPointOnEdge3D(rResult,Pos,pEdge,TPos,pTempEnt);
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
    if(pEntity->GetType()==SGM::EntityType::EdgeType)
        {
        edge *pEdge=(edge *)pEntity;
        std::map<edge *,SGM::EdgeSideType>::const_iterator EdgeTypeIter=m_mSideType.find(pEdge);
        SGM::Point2D Buv=EvaluateParamSpace(pEdge,EdgeTypeIter->second,CPos);

        SGM::Vector3D VecU,VecV,Vec;
        m_pSurface->Evaluate(Buv,nullptr,&VecU,&VecV);
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
        if(EdgeTypeIter->second==SGM::FaceOnRightType)
            {
            dZ=-dZ;
            }
        if(SGM_ZERO<=dZ)
            {
            return !m_bFlipped;
            }
        else if(-SGM_ZERO>=dZ)
            {
            return m_bFlipped;
            }
        else
            {
            return true;
            }
        }
    else // The vertex case.
        {
        SGM::Point2D Buv=m_pSurface->Inverse(CPos);
        vertex *pVertex=(vertex *)pEntity;
        SGM::Point3D const &VertexPos=pVertex->GetPoint();
        if(pPos)
            {
            *pPos=VertexPos;
            }
        if(SGM::NearEqual(VertexPos, Pos, SGM_ZERO))
            {
            return true;
            }
        else
            {
            // Consider the non-seam edges of the vertex on this face.
            
            std::set<edge *,EntityCompare> const &sVertexEdges=pVertex->GetEdges();
            std::vector<edge *> aTestEdges;
            for(auto *pEdge : sVertexEdges)
                {
                std::set<face *,EntityCompare> const &sEdgeFaces=pEdge->GetFaces();
                if(sEdgeFaces.find((face *)this)!=sEdgeFaces.end())
                    {
                    aTestEdges.push_back(pEdge);
                    }
                }
            if(aTestEdges.size()==1)
                {
                edge *pEdge=aTestEdges[0];
                SGM::EdgeSideType nSideType=GetSideType(pEdge);
                SGM::Vector3D StartDir=pEdge->FindStartVector();
                SGM::Vector3D EndDir=pEdge->FindEndVector();
                EndDir.Negate();
                SGM::Point2D uvC=EvaluateParamSpace(pEdge,nSideType,VertexPos);
                SGM::Point2D uvA=uvC,uvB=uvC;
                if(nSideType==SGM::FaceOnLeftType)
                    {
                    uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                    uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                    }
                else if(nSideType==SGM::FaceOnRightType)
                    {
                    uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                    uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                    }
                else
                    {
                    return true;
                    }

                return InAngle(uvC,uvA,uvB,uv);
                }
            else if(aTestEdges.size()==2)
                {
                edge *pEdge0=aTestEdges[0];
                edge *pEdge1=aTestEdges[1];
                SGM::EdgeSideType nSideType0=GetSideType(pEdge0);
                SGM::EdgeSideType nSideType1=GetSideType(pEdge1);
                SGM::Point2D uvA=EvaluateParamSpace(pEdge0,nSideType0,VertexPos);
                SGM::Point2D uvB=uvA,uvC=uvA;

                if(nSideType0==SGM::FaceOnLeftType)
                    {
                    if(pEdge0->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge0->FindStartVector();
                        uvB=uvA+m_pSurface->FindSurfaceDirection(uvA,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge0->FindEndVector();
                        EndDir.Negate();
                        uvC=uvA+m_pSurface->FindSurfaceDirection(uvA,EndDir);
                        }
                    }
                else if(nSideType0==SGM::FaceOnRightType)
                    {
                    if(pEdge0->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge0->FindStartVector();
                        uvC=uvA+m_pSurface->FindSurfaceDirection(uvA,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge0->FindEndVector();
                        EndDir.Negate();
                        uvB=uvA+m_pSurface->FindSurfaceDirection(uvA,EndDir);
                        }
                    }
                else
                    {
                    return true;
                    }

                if(nSideType1==SGM::FaceOnLeftType)
                    {
                    if(pEdge1->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge1->FindStartVector();
                        uvB=uvA+m_pSurface->FindSurfaceDirection(uvA,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge1->FindEndVector();
                        EndDir.Negate();
                        uvC=uvA+m_pSurface->FindSurfaceDirection(uvA,EndDir);
                        }
                    }
                else if(nSideType1==SGM::FaceOnRightType)
                    {
                    if(pEdge1->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge1->FindStartVector();
                        uvC=uvA+m_pSurface->FindSurfaceDirection(uvA,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge1->FindEndVector();
                        EndDir.Negate();
                        uvB=uvA+m_pSurface->FindSurfaceDirection(uvA,EndDir);
                        }
                    }
                else
                    {
                    throw;  // A case that has not been considered.
                    }
                if (m_bFlipped)
                    {
                    return InAngle(uvA,uvC,uvB,uv);
                    }
                else
                    {
                    return InAngle(uvA,uvB,uvC,uv);
                    }
                }
            else
                {
                throw;  // More code has to be added for the 
                // case of being closest to a vertex with more than 
                // two edges on the given face.
                }
            }
        }
    }

double TriangleArea(std::vector<SGM::Point3D> const &aPoints3D,
                    std::vector<unsigned int> const &aTriangles)
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
    if(!m_aPoints2D.empty())
        {
        m_aPoints3D.clear();
        m_aPoints2D.clear();
        m_aTriangles.clear();
        m_aNormals.clear();
        m_Box.Reset();
        m_mSeamType.clear();
        if(m_pVolume)
            {
            m_pVolume->ResetBox(rResult);
            }
        }
    }

double face::FindArea(SGM::Result &rResult) const
    {
    // Method one. 
    // Using Romberg integration on a reduced set of parametric triangles.

    //SGMInternal::FacetOptions Options;
    //Options.m_bParametric=true;
    //std::vector<SGM::Point2D> aPoints2D;
    //std::vector<SGM::Point3D> aPoints3D;
    //std::vector<unsigned int> aTriangles;
    //std::vector<SGM::UnitVector3D> aNormals;
    //std::vector<entity *> aEntities;
    //SGMInternal::FacetFace(rResult,this,Options,aPoints2D,aPoints3D,aNormals,aTriangles,aEntities);
    //size_t nTriangles=aTriangles.size();
    //double dMethod1=0;
    //size_t Index1;
    //for(Index1=0;Index1<nTriangles;Index1+=3)
    //    {
    //    size_t a=aTriangles[Index1];
    //    size_t b=aTriangles[Index1+1];
    //    size_t c=aTriangles[Index1+2];
    //    SGM::Point2D const &PosA=aPoints2D[a];
    //    SGM::Point2D const &PosB=aPoints2D[b];
    //    SGM::Point2D const &PosC=aPoints2D[c];
    //    dMethod1+=m_pSurface->FindAreaOfParametricTriangle(rResult,PosA,PosB,PosC);
    //    }

    // Method two.  
    // Refining the full set of parametric triangles twice and taking their area.

    this->GetPoints2D(rResult);
    std::vector<SGM::Point2D> aPoints2D=m_aPoints2D;
    std::vector<SGM::Point3D> aPoints3D=m_aPoints3D;
    std::vector<unsigned int> aTriangles=m_aTriangles;

    double dDiff=SGM_MAX;
    double dOldArea=SGM_MAX;
    double dArea2=0;
    double dArea0=TriangleArea(aPoints3D,aTriangles);
    size_t nCount=0;
    while(SGM_MIN_TOL<dDiff && nCount<5)
        {
        SubdivideFacets(this,aPoints3D,aPoints2D,aTriangles);
        double dArea1=TriangleArea(aPoints3D,aTriangles);
        dArea2=(4*dArea1-dArea0)/3;
        dArea0=dArea1;
        dDiff=fabs(dArea2-dOldArea);
        dOldArea=dArea2;
        ++nCount;
        }

    return dArea2;
    //return std::max(dMethod1,dArea2);
    }

double FindLocalVolume(std::vector<SGM::Point3D> const &aPoints,
                       std::vector<unsigned int> const &aTriangles)
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
    std::vector<unsigned int> aTriangles=m_aTriangles;
    double dAnswer0=FindLocalVolume(aPoints3D,aTriangles);
    if(m_bFlipped)
        {
        dAnswer0=-dAnswer0;
        }
    if(bApproximate)
        {
        return dAnswer0;
        }

    double dVolume=dAnswer0;
    size_t Index1;
    for(Index1=0;Index1<2;++Index1)
        {
        SubdivideFacets(this,aPoints3D,aPoints2D,aTriangles);
        double dAnswer1=FindLocalVolume(aPoints3D,aTriangles);
        if(m_bFlipped)
            {
            dAnswer1=-dAnswer1;
            }
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
    std::set<SGM::Edge> sEdges;
    for(auto pEdge : m_sEdges)
        {
        sEdges.insert(SGM::Edge(pEdge->GetID()));
        }
    SGM::Graph graph(rResult,sEdges);
    std::vector<SGM::Graph> aComponents;
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
        SGM::Graph const &comp=aComponents[Index1];
        std::set<SGM::GraphEdge> const &sGEdges=comp.GetEdges();
        std::set<edge *,EntityCompare> sEdges;
        std::set<SGM::GraphEdge>::const_iterator GEdgeIter=sGEdges.begin();
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

void face::AddEdge(SGM::Result       &rResult,
                   edge              *pEdge,
                   SGM::EdgeSideType nEdgeType)
    {
    m_sEdges.insert(pEdge);
    m_mSideType[pEdge]=nEdgeType;
    pEdge->AddFace(this);
    ClearFacets(rResult);
    }

void face::RemoveEdge(SGM::Result &rResult,
                      edge        *pEdge)
    {
    m_sEdges.erase(pEdge);
    m_mSideType.erase(pEdge);
    m_mSeamType.erase(pEdge);
    pEdge->RemoveFace(this);
    ClearFacets(rResult);
    }

void face::SetSurface(surface *pSurface)
    {
    if(m_pSurface)
        {
        m_pSurface->RemoveFace(this);
        }
    m_pSurface=pSurface;
    if(pSurface)
        {
        pSurface->AddFace(this);
        }
    }

SGM::EdgeSideType face::GetSideType(edge const *pEdge) const 
    {
    std::map<edge *,SGM::EdgeSideType>::const_iterator iter=m_mSideType.find((edge *)pEdge);
    if(iter==m_mSideType.end())
        {
        return SGM::EdgeSideType::FaceOnUnknown;
        }
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

bool face::HasBranchedVertex() const
    {
    std::set<vertex *> sVertices;
    for(auto pEdge : m_sEdges)
        {
        if(pEdge->GetStart())
            {
            sVertices.insert(pEdge->GetStart());
            }
        if(pEdge->GetEnd())
            {
            sVertices.insert(pEdge->GetEnd());
            }
        }
    for(auto pVertex : sVertices)
        {
        std::set<edge *,EntityCompare> const &sEdges=pVertex->GetEdges();
        int nCount=0;
        for(auto pVertexEdge : sEdges)
            {
            if(m_sEdges.find(pVertexEdge)!=m_sEdges.end())
                {
                ++nCount;
                if(pVertexEdge->GetStart()==pVertexEdge->GetEnd())
                    {
                    ++nCount;
                    }
                }
            }
        if(2<nCount)
            {
            return true;
            }
        }
    return false;
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
                    if(uv.m_v<uvPlus.m_v && fabs(uv.m_v-uvPlus.m_v)<Domain.m_UDomain.Length()*0.5)
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
                    if(uv.m_u<uvPlus.m_u && fabs(uv.m_u-uvPlus.m_u)<Domain.m_VDomain.Length()*0.5)
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
                                      SGM::Point3D const &Pos,
                                      bool                bFirstCall) const
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
        else
            {
            // Check for singularities.
            if( pEdge->GetStart() && 
                m_pSurface->SingularHighV() && 
                SGM::NearEqual(uv.m_v,Domain.m_VDomain.m_dMax,SGM_MIN_TOL,false))
                {
                SGM::Point3D PosE;
                if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_MIN_TOL))
                    {
                    PosE=pEdge->FindMidPoint(0.01);
                    }
                else 
                    {
                    PosE=pEdge->FindMidPoint(0.99);
                    }
                SGM::Point2D uvE=m_pSurface->Inverse(PosE);
                uv.m_u=uvE.m_u;
                }
            else if( pEdge->GetStart() && 
                     m_pSurface->SingularHighU() && 
                     SGM::NearEqual(uv.m_u,Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                {
                SGM::Point3D PosE;
                if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_MIN_TOL))
                    {
                    PosE=pEdge->FindMidPoint(0.01);
                    }
                else 
                    {
                    PosE=pEdge->FindMidPoint(0.99);
                    }
                SGM::Point2D uvE=m_pSurface->Inverse(PosE);
                uv.m_v=uvE.m_v;
                }
            else if( pEdge->GetStart() && 
                     m_pSurface->SingularLowV() && 
                     SGM::NearEqual(uv.m_v,Domain.m_VDomain.m_dMin,SGM_MIN_TOL,false))
                {
                SGM::Point3D PosE;
                if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_MIN_TOL))
                    {
                    PosE=pEdge->FindMidPoint(0.01);
                    }
                else 
                    {
                    PosE=pEdge->FindMidPoint(0.99);
                    }
                SGM::Point2D uvE=m_pSurface->Inverse(PosE);
                uv.m_u=uvE.m_u;
                }
            else if( pEdge->GetStart() && 
                     m_pSurface->SingularLowU() && 
                     SGM::NearEqual(uv.m_u,Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                {
                SGM::Point3D PosE;
                if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_MIN_TOL))
                    {
                    PosE=pEdge->FindMidPoint(0.01);
                    }
                else 
                    {
                    PosE=pEdge->FindMidPoint(0.99);
                    }
                SGM::Point2D uvE=m_pSurface->Inverse(PosE);
                uv.m_v=uvE.m_v;
                }
            }

        // Deal with the case of a point at (0,0) on a torus.

        if( bFirstCall &&
            m_pSurface->ClosedInU() && 
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
            SGM::Point2D TestUV=EvaluateParamSpace(pEdge,nType,TestPos,false);
            uv.m_u=TestUV.m_u<Domain.m_UDomain.MidPoint() ? Domain.m_UDomain.m_dMin : Domain.m_UDomain.m_dMax;
            uv.m_v=TestUV.m_v<Domain.m_VDomain.MidPoint() ? Domain.m_VDomain.m_dMin : Domain.m_VDomain.m_dMax;
            }
        }
    return uv;
    }

} // End SGMInternal namespace
