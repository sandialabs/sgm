#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Faceter.h"
#include "Topology.h"
#include "Surface.h"
#include "Curve.h"
#include "Query.h"
#include "Mathematics.h"
#include "Signature.h"

#include "SGMGraph.h"
#include "SGMMathematics.h"
#include "SGMTriangle.h"
#include "SGMVector.h"

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
    GetSurface()->FindAllChildren(sChildren);
    for (auto pEdge : GetEdges())
        {
        sChildren.insert(pEdge);
        pEdge->FindAllChildren(sChildren);
        }
    }

void face::GetParents(std::set<entity *, EntityCompare> &sParents) const
    {
    sParents.emplace(m_pVolume);
    entity::GetParents(sParents);
    }

SGM::Interval3D const &face::GetBox(SGM::Result &rResult,bool bContruct) const
    {
    if(m_Box.IsEmpty() && bContruct)
        {
        switch(GetSurface()->GetSurfaceType())
            {
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

void face::RemoveParentsInSet(SGM::Result &rResult,
                              std::set<entity *,EntityCompare>  const &sParents)
{
    if (sParents.find(GetVolume()) != sParents.end())
    {
        GetVolume()->RemoveFace(this);
        SetVolume(nullptr);        
    }
    topology::RemoveParentsInSet(rResult, sParents);
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
        //else
        //    {
        //    m_sFixedSeamType[SeamType.first]=SeamType.second;
        //    }
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
        //else
        //    {
        //    m_sFixedSideType[SideType.first]=SideType.second;
        //    }
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
        //else
        //    {
        //    m_sFixedEdges.insert(pEdge);
        //    }
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

entity *FindClosestEdgeOrVertex(SGM::Result                          &rResult,
                                SGM::Point3D                   const &Pos,
                                std::set<edge *,EntityCompare> const &m_sEdges)
    {
    entity *pAnswer=nullptr;
    for(edge *pEdge : m_sEdges)
        {
        double dTol=std::max(SGM_MIN_TOL,pEdge->GetTolerance());
        if(pEdge->GetBox(rResult).InInterval(Pos,dTol))
            {
            if(pEdge->GetEnd() && SGM::NearEqual(Pos.DistanceSquared(pEdge->GetEnd()->GetPoint()),0,SGM_ZERO,false))
                {
                pAnswer=pEdge->GetEnd();
                break;
                }
            if(pEdge->GetStart() && SGM::NearEqual(Pos.DistanceSquared(pEdge->GetStart()->GetPoint()),0,SGM_ZERO,false))
                {
                pAnswer=pEdge->GetStart();
                break;
                }
            if(pEdge->PointInEdge(Pos,dTol))
                {
                pAnswer=pEdge;
                break;
                }
            }
        }
    return pAnswer;
    }

void face::FindPointEntities(SGM::Result &rResult, std::vector<entity *> &aEntities) const
    {
    if(m_aPoints2D.empty())
        {
        FacetOptions Options;
        FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
        }

    std::vector<unsigned int> aBoundary;
    std::set<unsigned int> sInterior;
    SGM::FindBoundary(m_aTriangles,aBoundary,sInterior);
    aEntities.assign(m_aPoints3D.size(),(entity *)this);

    for(unsigned int Index : aBoundary)
        {
        if(aEntities[Index]==this)
            {
            if(entity *pEnt=FindClosestEdgeOrVertex(rResult,m_aPoints3D[Index],m_sEdges))
                {
                aEntities[Index]=pEnt;
                }
            }
        }
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
    auto pEntity= (entity *)this;
    double dDist=DBL_MAX;
    if(ClosePos)
        {
        Pos=*ClosePos;
        }
    else
        {
        m_pSurface->Evaluate(uv,&Pos);
        }
    for (auto pEdge : m_sEdges)
        {
        entity *pTempEnt=nullptr;
        FindClosestPointOnEdge3D(rResult,Pos,pEdge,TPos,pTempEnt);
        double dTempDist=Pos.DistanceSquared(TPos);
        if(dTempDist<dDist)
            {
            dDist=dTempDist;
            pEntity=pTempEnt;
            CPos=TPos;
            }
        }

    // Find the parameter line to check sided-ness from.

    if(pBoundary)
        {
        *pBoundary=pEntity;
        }
    if(pEntity->GetType()==SGM::EntityType::EdgeType)
        {
        auto pEdge=(edge *)pEntity;
        auto EdgeTypeIter=m_mSideType.find(pEdge);
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
        //SGM::Point2D Buv=m_pSurface->Inverse(CPos);
        auto pVertex=(vertex *)pEntity;
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

bool OnNonLinearEdge(SGM::Point3D const &PosA,
                     SGM::Point3D const &PosB,
                     entity       const *pAEnt,
                     entity       const *pBEnt,
                     SGM::Point3D       &Pos)
    {
    if(pAEnt->GetType()==SGM::EdgeType && pBEnt->GetType()==SGM::EdgeType && pAEnt==pBEnt)
        {
        edge *pEdge=(edge *)pAEnt;
        pEdge->GetCurve()->Inverse(SGM::MidPoint(PosA,PosB),&Pos);
        return true;
        }
    if( pAEnt->GetType()==SGM::VertexType && pBEnt->GetType()==SGM::EdgeType && 
        (((edge *)pBEnt)->GetStart()==pAEnt || ((edge *)pBEnt)->GetEnd()==pAEnt))
        {
        edge *pEdge=(edge *)pBEnt;
        pEdge->GetCurve()->Inverse(SGM::MidPoint(PosA,PosB),&Pos);
        return true;
        }
    if( pAEnt->GetType()==SGM::EdgeType && pBEnt->GetType()==SGM::VertexType && 
        (((edge *)pAEnt)->GetStart()==pBEnt || ((edge *)pAEnt)->GetEnd()==pBEnt))
        {
        edge *pEdge=(edge *)pAEnt;
        pEdge->GetCurve()->Inverse(SGM::MidPoint(PosA,PosB),&Pos);
        return true;
        }
    return false;
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
        SGM::Point3D const &PosA=aPoints3D[a];
        SGM::Point3D const &PosB=aPoints3D[b];
        SGM::Point3D const &PosC=aPoints3D[c];
        dArea+=((PosB-PosA)*(PosC-PosA)).Magnitude();
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

void face::InitializeFacetSubdivision(SGM::Result &rResult,
                                      const size_t MAX_LEVELS,
                                      std::vector<SGM::Point2D> &aPoints2D,
                                      std::vector<SGM::Point3D> &aPoints3D,
                                      std::vector<unsigned int> &aTriangles,
                                      std::vector<entity *> &aEntities) const
    {
    GetPoints2D(rResult);
    size_t nPoints = m_aPoints3D.size();
    size_t nTriangles = m_aTriangles.size();
    for (unsigned iLevel = 0; iLevel < MAX_LEVELS; ++iLevel)
        {
        nPoints=nTriangles+2*nPoints-2;
        nTriangles=nTriangles*4;
        }
    assert(nPoints < std::numeric_limits<unsigned>::max());

    aPoints2D.reserve(nPoints);
    aPoints2D.assign(m_aPoints2D.begin(), m_aPoints2D.end());

    aPoints3D.reserve(nPoints);
    aPoints3D.assign(m_aPoints3D.begin(), m_aPoints3D.end());

    aTriangles.reserve(nTriangles);
    aTriangles.assign(m_aTriangles.begin(), m_aTriangles.end());

    aEntities.reserve(nPoints);
    FindPointEntities(rResult, aEntities);
    }

// Richardson extrapolation to infinite resolution
// Returns true if extrapolation was performed and the extrapolated value.

std::pair<bool,double> RichardsonExtrapolation(const double * aEstimated)
    {
    double aDifference[2];
    double dExtrapolated = 0.0;
    aDifference[0] = aEstimated[0] - aEstimated[1];
    aDifference[1] = aEstimated[1] - aEstimated[2];
    double dRatio = aDifference[0] / aDifference[1];
    bool isSameSign = dRatio > 0;
    if (isSameSign)
        {
        double p = std::log(dRatio) / std::log(2.0);
        double a = aDifference[0] / (std::pow(2.0, p) - std::pow(1.0, p));
        dExtrapolated = aEstimated[2] - a * std::pow(0.5, p);
        //std::cout << "    extrapolated = " << dExtrapolated << std::endl;
        }
    return std::make_pair(isSameSign,dExtrapolated);
    }

double face::FindArea(SGM::Result &rResult) const
    {
    static const size_t MAX_LEVELS = 5;
    double aArea[MAX_LEVELS];
    double aAreaEstimated[MAX_LEVELS];

    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned int> aTriangles;
    std::vector<entity *>     aEntities;
    InitializeFacetSubdivision(rResult, MAX_LEVELS, aPoints2D, aPoints3D, aTriangles, aEntities);

    double dDiff = SGM_MAX;
    aArea[0] = TriangleArea(aPoints3D,aTriangles);
    aAreaEstimated[0] = SGM_MAX;
    size_t nLevel;
    for (nLevel=1; SGM_MIN_TOL<dDiff && nLevel<MAX_LEVELS; ++nLevel)
        {
        SubdivideFacets(this,aPoints3D,aPoints2D,aTriangles,aEntities);
        aArea[nLevel]=TriangleArea(aPoints3D,aTriangles);
        aAreaEstimated[nLevel]=(4*aArea[nLevel]-aArea[nLevel-1])/3;
        if (nLevel>=3 && dDiff<SGM_MIN_TOL) // We have at least 3 levels and getting close to tolerance.
            {
            std::pair<bool, double> Extrapolated = RichardsonExtrapolation(&aAreaEstimated[nLevel - 2]);
            if (Extrapolated.first)
                {
                return Extrapolated.second;
                }
            }
            dDiff=std::abs(aAreaEstimated[nLevel]-aAreaEstimated[nLevel-1]);
        }
    return aAreaEstimated[nLevel-1];
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

inline double NegateValue(bool bFlipped, double dValue)
    {
    return bFlipped ? -dValue : dValue;
    }

double face::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    static const size_t MAX_LEVELS = 5;
    double aVolume[MAX_LEVELS];
    double aVolumeEstimated[MAX_LEVELS];
    GetPoints2D(rResult);

    aVolume[0]=NegateValue(m_bFlipped,FindLocalVolume(m_aPoints3D,m_aTriangles));
    if(bApproximate)
        {
        return aVolume[0];
        }

    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<unsigned int> aTriangles;
    std::vector<entity *>     aEntities;
    InitializeFacetSubdivision(rResult, MAX_LEVELS, aPoints2D, aPoints3D, aTriangles, aEntities);

    double dDiff = SGM_MAX;
    aVolumeEstimated[0] = SGM_MAX;
    size_t nLevel;

    for(nLevel=1; dDiff>SGM_MIN_TOL && nLevel<MAX_LEVELS; ++nLevel)
        {
        SubdivideFacets(this,aPoints3D,aPoints2D,aTriangles,aEntities);
        aVolume[nLevel]=NegateValue(m_bFlipped,FindLocalVolume(aPoints3D,aTriangles));
        aVolumeEstimated[nLevel]=(4*aVolume[nLevel]-aVolume[nLevel-1])/3;
        if (nLevel>=3 && dDiff<1000*SGM_MIN_TOL) // we have at least 3 levels and getting close to tolerance
            {
            std::pair<bool, double> Extrapolated = RichardsonExtrapolation(&aVolumeEstimated[nLevel - 2]);
            if (Extrapolated.first)
                {
                return Extrapolated.second;
                }
            }
        dDiff=std::abs(aVolumeEstimated[nLevel]-aVolumeEstimated[nLevel-1]);
        }
    return aVolumeEstimated[nLevel-1];
    }

SGM::UnitVector3D face::FindNormalOfFace(SGM::Point3D const &Pos) const
    {
    SGM::Point2D uv=m_pSurface->Inverse(Pos);
    SGM::UnitVector3D Norm;
    m_pSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
    if(m_bFlipped)
        {
        Norm.Negate();
        }
    return Norm;
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

    size_t Index1,Index2;
    for(Index1=0;Index1<nLoops;++Index1)
        {
        SGM::Graph const &comp=aComponents[Index1];
        std::set<SGM::GraphEdge> const &sGEdges=comp.GetEdges();
        std::set<edge *,EntityCompare> sLoopEdges;
        auto GEdgeIter=sGEdges.begin();
        while(GEdgeIter!=sGEdges.end())
            {
            size_t ID=GEdgeIter->m_nID;
            sLoopEdges.insert((edge *)pThing->FindEntity(ID));
            ++GEdgeIter;
            }
        std::vector<edge *> aEdges;
        std::vector<SGM::EdgeSideType> aFlips;
        OrderLoopEdges(rResult,this,sLoopEdges,aEdges,aFlips);
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

    // Check for double sided loops of surface that are close in both directions.

    if(m_pSurface->ClosedInU() && m_pSurface->ClosedInV())
        {
        for(Index1=0;Index1<nLoops;++Index1)
            {
            auto aLoop=aaLoops[Index1];
            bool bFound=false;
            for(auto pEdge : aLoop)
                {
                if(GetSideType(pEdge)!=SGM::FaceOnBothSidesType)
                    {
                    bFound=true;
                    break;
                    }
                }
            if(!bFound)
                {
                std::vector<SGM::EdgeSideType> aNewFlipped;
                size_t nLoop=aLoop.size();
                aNewFlipped.reserve(nLoop);
                for(Index2=0;Index2<nLoop;++Index2)
                    {
                    aaFlipped[Index1][Index2]=SGM::EdgeSideType::FaceOnLeftType;
                    aNewFlipped.push_back(SGM::EdgeSideType::FaceOnRightType);
                    }
                aaFlipped.push_back(aNewFlipped);
                aaLoops.push_back(aaLoops[Index1]);
                }
            }
        }

    return aaLoops.size();
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

void face::SetEdgeSideType(SGM::Result       &rResult,
                           edge              *pEdge,
                           SGM::EdgeSideType nEdgeType)
    {
    m_mSideType[pEdge]=nEdgeType;
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
    auto iter=m_mSideType.find((edge *)pEdge);
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

void face::Negate()
    {
    for(edge *pEdge : m_sEdges)
        {
        SGM::EdgeSideType nType=m_mSideType[pEdge];
        if(nType==SGM::FaceOnLeftType)
            {
            m_mSideType[pEdge]=SGM::FaceOnRightType;
            }
        else if(nType==SGM::FaceOnRightType)
            {
            m_mSideType[pEdge]=SGM::FaceOnLeftType;
            }
        }
    m_bFlipped=!m_bFlipped;
    size_t nTriangles=m_aTriangles.size();
    if(nTriangles)
        {
        size_t Index1;
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            std::swap(m_aTriangles[Index1],m_aTriangles[Index1+1]);
            }
        size_t nNormals=m_aNormals.size();
        for(Index1=0;Index1<nNormals;++Index1)
            {
            m_aNormals[Index1].Negate();
            }
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

Signature const & face::GetSignature(SGM::Result &rResult) const
{
    if (!m_Signature.Xsequence.size())
    {
        std::vector<SGM::Point3D> aFacePoints;
        GetSignaturePoints(rResult, aFacePoints);
        m_Signature.Initialize(aFacePoints);
    }
    return m_Signature;
}

void face::GetSignaturePoints(SGM::Result &rResult,
                              std::vector<SGM::Point3D> &aPoints) const
{
    std::set<vertex *, EntityCompare> sVertices;
    FindVertices(rResult, this, sVertices);
    if (sVertices.size() == 0)
        throw std::logic_error("Signature for face without vertices is not implemented");

    for (auto pVert : sVertices)
    {
        aPoints.emplace_back(pVert->GetPoint());
    }

    for (auto pEdge : m_sEdges)
    {
        std::vector<SGM::Point3D> aEdgePoints;
        pEdge->GetSignaturePoints(rResult, aEdgePoints);
        for (auto Pos : aEdgePoints)
        {
            aPoints.emplace_back(Pos);
        }
    }
}

} // End SGMInternal namespace
