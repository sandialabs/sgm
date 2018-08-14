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

face *face::MakeCopy(SGM::Result &rResult) const
    {
    face *pAnswer=new face(rResult);

    pAnswer->m_sEdges=m_sEdges;
    pAnswer->m_mSideType=m_mSideType;
    pAnswer->m_pVolume=m_pVolume;
    pAnswer->m_pSurface=m_pSurface;
    pAnswer->m_bFlipped=m_bFlipped;
    pAnswer->m_nSides=m_nSides;

    pAnswer->m_aPoints3D=m_aPoints3D;
    pAnswer->m_aPoints2D=m_aPoints2D;
    pAnswer->m_aEntities=m_aEntities;
    pAnswer->m_aTriangles=m_aTriangles;
    pAnswer->m_aNormals=m_aNormals;
    pAnswer->m_mSeamType=m_mSeamType;

    pAnswer->m_Box=m_Box;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_sOwners=m_sOwners;
    return pAnswer;
    }

void face::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.

    std::vector<entity *> m_aFixedEntities;
    m_aFixedEntities.reserve(m_aEntities.size());
    for(auto pEntity : m_aFixedEntities)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            {
            m_aFixedEntities.push_back(MapValue->second);
            }
        else
            {
            m_aFixedEntities.push_back(pEntity);
            }
        }
    m_aEntities=m_aFixedEntities;

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

    std::set<attribute *,EntityCompare> m_sFixedAttributes;
    for(auto pAttribute : m_sAttributes)
        {
        auto MapValue=mEntityMap.find(pAttribute);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedAttributes.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedAttributes.insert(pAttribute);
            }
        }
    m_sAttributes=m_sFixedAttributes;

    std::set<entity *,EntityCompare> m_sFixedOwners;
    for(auto pEntity : m_sOwners)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedOwners.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedOwners.insert(pEntity);
            }
        }
    m_sOwners=m_sFixedOwners;
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

std::vector<unsigned int> const &face::GetTriangles(SGM::Result &rResult) const
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
                    throw;  // A case that has not been considered.
                    }

                return InAngle(uvC,uvA,uvB,uv);
                }
            else if(aTestEdges.size()==2)
                {
                edge *pEdge0=aTestEdges[0];
                edge *pEdge1=aTestEdges[1];
                SGM::EdgeSideType nSideType0=GetSideType(pEdge0);
                SGM::EdgeSideType nSideType1=GetSideType(pEdge1);
                SGM::Point2D uvC=EvaluateParamSpace(pEdge0,nSideType0,VertexPos);
                SGM::Point2D uvA=uvC,uvB=uvC;

                if(nSideType0==SGM::FaceOnLeftType)
                    {
                    if(pEdge0->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge0->FindStartVector();
                        uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge0->FindEndVector();
                        EndDir.Negate();
                        uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                        }
                    }
                else if(nSideType0==SGM::FaceOnRightType)
                    {
                    if(pEdge0->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge0->FindStartVector();
                        uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge0->FindEndVector();
                        EndDir.Negate();
                        uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                        }
                    }
                else
                    {
                    throw;  // A case that has not been considered.
                    }

                if(nSideType1==SGM::FaceOnLeftType)
                    {
                    if(pEdge1->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge1->FindStartVector();
                        uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge1->FindEndVector();
                        EndDir.Negate();
                        uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                        }
                    }
                else if(nSideType1==SGM::FaceOnRightType)
                    {
                    if(pEdge1->GetStart()==pVertex)
                        {
                        SGM::Vector3D StartDir=pEdge1->FindStartVector();
                        uvA=uvC+m_pSurface->FindSurfaceDirection(uvC,StartDir);
                        }
                    else
                        {
                        SGM::Vector3D EndDir=pEdge1->FindEndVector();
                        EndDir.Negate();
                        uvB=uvC+m_pSurface->FindSurfaceDirection(uvC,EndDir);
                        }
                    }
                else
                    {
                    throw;  // A case that has not been considered.
                    }
                return InAngle(uvC,uvA,uvB,uv);
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
    std::vector<unsigned int> aTriangles;
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
    std::vector<entity *> aEntities=m_aEntities;
    std::vector<unsigned int> aTriangles=m_aTriangles;
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