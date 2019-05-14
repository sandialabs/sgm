#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Faceter.h"
#include "Topology.h"
#include "Surface.h"
#include "Curve.h"
#include "Query.h"
#include "Mathematics.h"
#include "Signature.h"
#include "Intersectors.h"
#include "Primitive.h"

#include "SGMGraph.h"
#include "SGMMathematics.h"
#include "SGMTriangle.h"
#include "SGMVector.h"
#include "SGMPolygon.h"

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
    sChildren.insert(m_pSurface);
    m_pSurface->FindAllChildren(sChildren);
    for (auto pEdge : m_sEdges)
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
        switch(m_pSurface->GetSurfaceType())
            {
            case SGM::EntityType::CylinderType:
            case SGM::EntityType::PlaneType:
                {
                // Only use the edge boxes.
                StretchBox(rResult,m_Box,m_sEdges.begin(),m_sEdges.end());
                break;
                }
            default:
                {
                // Use all the points.
                if(m_aPoints2D.empty())
                    {
                    FacetOptions Options;
                    FacetFace(rResult,this,Options,m_aPoints2D,m_aPoints3D,m_aNormals,m_aTriangles);
                    }
                double dMaxLength = SGM::FindMaxEdgeLength3D(m_aPoints3D,m_aTriangles);
                m_Box = SGM::Interval3D(m_aPoints3D);
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

    if(m_pVolume)
        {
        return m_pVolume->GetColor(nRed,nGreen,nBlue);
        }
    else
        {
        return entity::GetColor(nRed,nGreen,nBlue);
        }
    }

void face::RemoveParentsInSet(SGM::Result                            &rResult,
                              std::set<entity *,EntityCompare> const &sParents)
{
    if (sParents.find(m_pVolume) != sParents.end())
    {
        m_pVolume->RemoveFace(rResult,this);
        m_pVolume=nullptr;        
    }
    topology::RemoveParentsInSet(rResult, sParents);
}

void face::SeverRelations(SGM::Result &rResult)
    {
    if(m_pVolume)
        {
        m_pVolume->RemoveFace(rResult,this);
        m_pVolume=nullptr;
        }
    std::set<edge *,EntityCompare> sEdges=GetEdges();
    for(edge *pEdge : sEdges)
        {
        pEdge->RemoveFace(this);
        }
    sEdges.clear();
    if(m_pSurface)
        {
        m_pSurface->RemoveFace(this);
        m_pSurface=nullptr;
        }
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
        }
    m_sEdges=m_sFixedEdges;
    m_sVertices.clear();
    OwnerAndAttributeReplacePointers(mEntityMap);
    }

volume *face::GetVolume() const 
    {
    return m_pVolume;
    }

std::set<vertex *,EntityCompare> const &face::GetVertices() const
    {
    if(m_sVertices.empty() && m_sEdges.empty()==false)
        {
        for(auto pEdge : m_sEdges)
            {
            if(vertex *pStart=pEdge->GetStart())
                {
                m_sVertices.insert(pStart);
                m_sVertices.insert(pEdge->GetEnd());
                }
            }
        }
    return m_sVertices;
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

void FixSegmentU(SGM::Interval1D const &UDomain,
                 SGM::Point2D    const &uv1,
                 SGM::Point2D    const &uv2,
                 SGM::Point2D          &FixedUV1,
                 SGM::Point2D          &FixedUV2)
    {
    bool bNotFixed=true;
    if(fabs(UDomain.m_dMin-uv1.m_u)<SGM_MIN_TOL)
        {
        if(UDomain.MidPoint()<uv2.m_u)
            {
            FixedUV1.m_u+=UDomain.Length();
            bNotFixed=false;
            }
        }
    else if(fabs(UDomain.m_dMax-uv1.m_u)<SGM_MIN_TOL)
        {
        if(uv2.m_u<UDomain.MidPoint())
            {
            FixedUV1.m_u-=UDomain.Length();
            bNotFixed=false;
            }
        }
    if(bNotFixed)
        {
        if(fabs(UDomain.m_dMin-uv2.m_u)<SGM_MIN_TOL)
            {
            if(UDomain.MidPoint()<uv1.m_u)
                {
                FixedUV2.m_u+=UDomain.Length();
                }
            }
        else if(fabs(UDomain.m_dMax-uv2.m_u)<SGM_MIN_TOL)
            {
            if(uv1.m_u<UDomain.MidPoint())
                {
                FixedUV2.m_u-=UDomain.Length();
                }
            }
        }
    }

void FixSegmentV(SGM::Interval1D const &VDomain,
                 SGM::Point2D    const &uv1,
                 SGM::Point2D    const &uv2,
                 SGM::Point2D          &FixedUV1,
                 SGM::Point2D          &FixedUV2)
    {
    bool bNotFixed=true;
    if(fabs(VDomain.m_dMin-uv1.m_v)<SGM_MIN_TOL)
        {
        if(VDomain.MidPoint()<uv2.m_v)
            {
            FixedUV1.m_v+=VDomain.Length();
            bNotFixed=false;
            }
        }
    else if(fabs(VDomain.m_dMax-uv1.m_v)<SGM_MIN_TOL)
        {
        if(uv2.m_v<VDomain.MidPoint())
            {
            FixedUV1.m_v-=VDomain.Length();
            bNotFixed=false;
            }
        }
    if(bNotFixed)
        {
        if(fabs(VDomain.m_dMin-uv2.m_v)<SGM_MIN_TOL)
            {
            if(VDomain.MidPoint()<uv1.m_v)
                {
                FixedUV2.m_v+=VDomain.Length();
                }
            }
        else if(fabs(VDomain.m_dMax-uv2.m_v)<SGM_MIN_TOL)
            {
            if(uv1.m_v<VDomain.MidPoint())
                {
                FixedUV2.m_v-=VDomain.Length();
                }
            }
        }
    }

inline void FixSegmentUV(SGM::Interval1D const &UDomain,
                         SGM::Interval1D const &VDomain,
                         SGM::Point2D          &uv1,
                         SGM::Point2D          &uv2,
                         SGM::Point2D          &FixedUV1,
                         SGM::Point2D          &FixedUV2)
    {
    FixSegmentU(UDomain,uv1,uv2,FixedUV1,FixedUV2);
    FixSegmentV(VDomain,uv1,uv2,FixedUV1,FixedUV2);
    uv1=FixedUV1;
    uv2=FixedUV2;
    }

struct
    {
    double operator()(surface      const *pSurface,
                      SGM::Point2D       &uv1,
                      SGM::Point2D       &uv2,
                      SGM::Point2D const &uv) const
        {
        SGM::Interval1D const &UDomain = pSurface->GetDomain().m_UDomain;
        SGM::Interval1D const &VDomain = pSurface->GetDomain().m_VDomain;
        SGM::Point2D FixedUV1=uv1,FixedUV2=uv2;
        FixSegmentUV(UDomain,VDomain,uv1,uv2,FixedUV1,FixedUV2);
        uv1=FixedUV1;
        uv2=FixedUV2;
        return SegmentDistanceSquared(FixedUV1,FixedUV2,uv);
        }
    } FixSegmentSquaredDistanceUV;

struct
    {
    double operator()(surface      const *pSurface,
                      SGM::Point2D       &uv1,
                      SGM::Point2D       &uv2,
                      SGM::Point2D const &uv) const
        {
        SGM::Interval1D const &UDomain = pSurface->GetDomain().m_UDomain;
        SGM::Point2D FixedUV1=uv1,FixedUV2=uv2;
        FixSegmentU(UDomain,uv1,uv2,FixedUV1,FixedUV2);
        uv1=FixedUV1;
        uv2=FixedUV2;
        return SegmentDistanceSquared(FixedUV1,FixedUV2,uv);
        }
    } FixSegmentSquaredDistanceU;

struct
    {
    double operator()(surface      const *pSurface,
                      SGM::Point2D       &uv1,
                      SGM::Point2D       &uv2,
                      SGM::Point2D const &uv) const
        {
        SGM::Interval1D const &VDomain = pSurface->GetDomain().m_VDomain;
        SGM::Point2D FixedUV1=uv1,FixedUV2=uv2;
        FixSegmentV(VDomain,uv1,uv2,FixedUV1,FixedUV2);
        uv1=FixedUV1;
        uv2=FixedUV2;
        return SegmentDistanceSquared(FixedUV1,FixedUV2,uv);
        }
    } FixSegmentSquaredDistanceV;

struct
    {
    double operator()(surface      const *,
                      SGM::Point2D       &uv1,
                      SGM::Point2D       &uv2,
                      SGM::Point2D const &uv) const
        {
        return SegmentDistanceSquared(uv1,uv2,uv);
        }
    } SegmentSquaredDistance;

template <class SegmentSquaredDistanceOp>
void MinimumFaceEdgeSegmentDistanceSquared(SGM::Result               &rResult,
                                           const face                *pFace,
                                           const SGM::Point2D        &uv,
                                           SegmentSquaredDistanceOp   squaredDistanceOp,
                                           edge                     **ppCloseEdge,
                                           double                    &dMinDist,
                                           SGM::Point2D              &OutUV1,
                                           SGM::Point2D              &OutUV2)
    {
    surface *pSurface=pFace->GetSurface();
    dMinDist=std::numeric_limits<double>::max();
    auto const &sEdges=pFace->GetEdges();
    for (edge *pEdge : sEdges)
        {
        std::vector<SGM::Point2D> const &aUVBoundary = pFace->GetUVBoundary(rResult, pEdge);
        size_t nUVBoundary = aUVBoundary.size();
        size_t Index1;
        for (Index1 = 1; Index1 < nUVBoundary; ++Index1)
            {
            SGM::Point2D uv1=aUVBoundary[Index1-1];
            SGM::Point2D uv2=aUVBoundary[Index1];
            double dDist = squaredDistanceOp(pSurface,uv1,uv2,uv);
            if (dDist < dMinDist)
                {
                dMinDist = dDist;
                OutUV1 = uv1;
                OutUV2 = uv2;
                *ppCloseEdge = pEdge;
                }
            }
        if (pEdge->IsClosed())
            {
            SGM::Point2D uv1=aUVBoundary[nUVBoundary-1];
            SGM::Point2D uv2=aUVBoundary[0];
            double dDist = squaredDistanceOp(pSurface,uv1,uv2,uv);
            if (dDist < dMinDist)
                {
                dMinDist = dDist;
                OutUV1 = uv1;
                OutUV2 = uv2;
                *ppCloseEdge = pEdge;
                }
            }
        }
    }

void FindClosestBoundary(SGM::Result        &rResult,
                         face         const *pFace,
                         SGM::Point2D const &uv,
                         SGM::Segment2D     &CloseSeg,
                         edge               **ppCloseEdge,
                         vertex             **ppCloseVertex,
                         SGM::Point2D       &CloseUV)
    {
    surface *pSurface=pFace->GetSurface();
    double dMinDist=std::numeric_limits<double>::max();
    SGM::Point2D Start,End;

    bool bClosedInU = pSurface->ClosedInU();
    bool bClosedInV = pSurface->ClosedInV();

    if (bClosedInU)
        {
        if (bClosedInV)
            {
            MinimumFaceEdgeSegmentDistanceSquared(rResult,pFace,uv,FixSegmentSquaredDistanceUV,
                                                  ppCloseEdge,dMinDist,Start,End);
            }
        else
            {
            MinimumFaceEdgeSegmentDistanceSquared(rResult,pFace,uv,FixSegmentSquaredDistanceU,
                                                  ppCloseEdge,dMinDist,Start,End);
            }
        }
    else if (bClosedInV)
        {
        MinimumFaceEdgeSegmentDistanceSquared(rResult,pFace,uv,FixSegmentSquaredDistanceV,
                                              ppCloseEdge,dMinDist,Start,End);
        }
    else
        {
        MinimumFaceEdgeSegmentDistanceSquared(rResult,pFace,uv,SegmentSquaredDistance,
                                              ppCloseEdge,dMinDist,Start,End);
        }

    // Fnd the closest point and segment.
    SGM::Interval2D const &SurfDomain=pSurface->GetDomain();
    if( SurfDomain.m_UDomain.Length()*0.5<fabs(End.m_u-Start.m_u) ||
        SurfDomain.m_VDomain.Length()*0.5<fabs(End.m_v-Start.m_v))
        {
        if(SurfDomain.m_UDomain.Length()*0.5<fabs(End.m_u-Start.m_u))
            {
            if(SurfDomain.m_UDomain.OnBoundary(Start.m_u,SGM_MIN_TOL))
                {
                if(Start.m_u<SurfDomain.m_UDomain.MidPoint())
                    {
                    Start.m_u+=SurfDomain.m_UDomain.Length();
                    }
                else
                    {
                    Start.m_u-=SurfDomain.m_UDomain.Length();
                    }
                }
            else if(SurfDomain.m_UDomain.OnBoundary(End.m_u,SGM_MIN_TOL))
                {
                if(End.m_u<SurfDomain.m_UDomain.MidPoint())
                    {
                    End.m_u+=SurfDomain.m_UDomain.Length();
                    }
                else
                    {
                    End.m_u-=SurfDomain.m_UDomain.Length();
                    }
                }
            }
        if(SurfDomain.m_VDomain.Length()*0.5<fabs(End.m_v-Start.m_v))
            {
            if(SurfDomain.m_VDomain.OnBoundary(Start.m_v,SGM_MIN_TOL))
                {
                if(Start.m_v<SurfDomain.m_VDomain.MidPoint())
                    {
                    Start.m_v+=SurfDomain.m_VDomain.Length();
                    }
                else
                    {
                    Start.m_v-=SurfDomain.m_VDomain.Length();
                    }
                }
            else if(SurfDomain.m_VDomain.OnBoundary(End.m_v,SGM_MIN_TOL))
                {
                if(End.m_v<SurfDomain.m_VDomain.MidPoint())
                    {
                    End.m_v+=SurfDomain.m_VDomain.Length();
                    }
                else
                    {
                    End.m_v-=SurfDomain.m_VDomain.Length();
                    }
                }
            }
        }
    SegmentClosestPoint(Start,End,uv,CloseUV);
    CloseSeg={Start,End};

    *ppCloseVertex=nullptr;
    vertex *pStart=(*ppCloseEdge)->GetStart();
    if(pStart)
        {
        SGM::Point3D Pos;
        pSurface->Evaluate(CloseUV,&Pos);
        SGM::Point3D StartPos=pStart->GetPoint();
        double dStartDist=Pos.Distance(StartPos);
        if(dStartDist<pStart->GetTolerance()+SGM_MIN_TOL)
            {
            *ppCloseVertex=pStart;
            }
        vertex *pEnd=(*ppCloseEdge)->GetEnd();
        SGM::Point3D EndPos=pEnd->GetPoint();
        double dEndDist=Pos.Distance(EndPos);
        if(dEndDist<pEnd->GetTolerance()+SGM_MIN_TOL)
            {
            *ppCloseVertex=pEnd;
            }

        // Check to see if we are near a cusp and some other edges is closer.

        if(pFace->GetSurface()->ClosedInU()==false && pFace->GetSurface()->ClosedInV()==false)
            {
            double t=(*ppCloseEdge)->GetCurve()->Inverse(Pos);
            double dFraction=(*ppCloseEdge)->GetDomain().Fraction(t);
            edge *pTestEdge=nullptr;
            if(dFraction<0.1)
                {
                std::vector<edge *> aEdges;
                FindEdgesOnFaceAtVertex(rResult,pStart,pFace,aEdges);
                if(aEdges.size()==2)
                    {
                    if(aEdges[0]!=(*ppCloseEdge))
                        {
                        pTestEdge=aEdges[0];
                        }
                    else
                        {
                        pTestEdge=aEdges[1];
                        }
                    }
                }
            else if(0.9<dFraction)
                {
                std::vector<edge *> aEdges;
                FindEdgesOnFaceAtVertex(rResult,pEnd,pFace,aEdges);
                if(aEdges.size()==2)
                    {
                    if(aEdges[0]!=(*ppCloseEdge))
                        {
                        pTestEdge=aEdges[0];
                        }
                    else
                        {
                        pTestEdge=aEdges[1];
                        }
                    }
                }
            if(pTestEdge)
                {
                double dDist1=(*ppCloseEdge)->DistanceToEdge(Pos);
                double dDist2=pTestEdge->DistanceToEdge(Pos);
                if(dDist2<dDist1)
                    {
                    *ppCloseEdge=pTestEdge;
                    }
                }
            }
        }
    }

void FixUV(surface      const* pSurface,
           SGM::Point2D const &GuessUV,
           SGM::Point2D       &uv)
    {
    // Make sure that uv did not flip over the boundary from GuessUV.

    if(pSurface->ClosedInU())
        {
        double dLength=pSurface->GetDomain().m_UDomain.Length();
        if(dLength*0.5<fabs(GuessUV.m_u-uv.m_u))
            {
            if(GuessUV.m_u<uv.m_u)
                {
                uv.m_u-=dLength;
                }
            else
                {
                uv.m_u+=dLength;
                }
            }
        }
    if(pSurface->ClosedInV())
        {
        double dLength=pSurface->GetDomain().m_VDomain.Length();
        if(dLength*0.5<fabs(GuessUV.m_v-uv.m_v))
            {
            if(GuessUV.m_v<uv.m_v)
                {
                uv.m_v-=dLength;
                }
            else
                {
                uv.m_v+=dLength;
                }
            }
        }
    }

bool face::PointInFace(SGM::Result        &rResult,
                       SGM::Point2D const &uv,
                       edge               **pInCloseEdge,
                       bool               *bOnEdge) const
    {
    // First check to see if the point is in the UV bounding box.

    if(m_UVBox.IsEmpty()==false && m_UVBox.InInterval(uv,SGM_ZERO)==false)
        {
        return false;
        }
        
    // Check for the closed face case.

    if(m_sEdges.empty())
        {
        if(pInCloseEdge)
            {
            *pInCloseEdge=nullptr;
            }
        return true;
        }

    // Find the closest entity in uv space along with the closet uv segment

    vertex *pCloseVertex=nullptr;
    SGM::Segment2D CloseSeg;
    edge *pCloseEdge=nullptr;
    if(pInCloseEdge)
        {
        *pInCloseEdge=nullptr;
        }
    SGM::Point3D Pos;
    SGM::Point2D CloseUV1;
    m_pSurface->Evaluate(uv,&Pos);
    FindClosestBoundary(rResult,this,uv,CloseSeg,&pCloseEdge,&pCloseVertex,CloseUV1);
    
    if(pCloseVertex==nullptr)
        {
        // Test to see if we are to the left or right of pEdge at CloseUV.

        SGM::Point2D CloseUV;
        CloseSeg.Distance(uv,&CloseUV);
        SGM::Point3D TestPos;
        m_pSurface->Evaluate(CloseUV,&TestPos);
        double t=pCloseEdge->GetCurve()->Inverse(TestPos);
        SGM::Vector3D Vec;
        SGM::Point3D ClosePos;
        pCloseEdge->GetCurve()->Evaluate(t,&ClosePos,&Vec);
        if(pCloseEdge->GetFaces().size()==2)
            {
            bool bUseOtherFace=false;
            double dDist=ClosePos.Distance(Pos);
            if(dDist<pCloseEdge->GetTolerance()*100 || dDist<SGM_FIT)
                {
                bUseOtherFace=true;
                }
            if( pCloseEdge->GetCurve()->GetCurveType()==SGM::NUBCurveType &&
                dDist<0.005)
                {
                bUseOtherFace=true;
                }
            if(bUseOtherFace)
                {
                std::set<face *,EntityCompare> const &sFaces=pCloseEdge->GetFaces();
                auto iter=sFaces.begin();
                face *pOtherFace=*iter;
                if(pOtherFace==this)
                    {
                    ++iter;
                    pOtherFace=*iter;
                    }
           
                SGM::Point3D OtherPos;
                SGM::Point2D OtherUV=pOtherFace->m_pSurface->Inverse(TestPos,&OtherPos);
                SGM::UnitVector3D Norm,OldNorm;
                pOtherFace->m_pSurface->Evaluate(OtherUV,nullptr,nullptr,nullptr,&Norm);
                m_pSurface->Evaluate(uv,nullptr,nullptr,nullptr,&OldNorm);
                if(pOtherFace->GetFlipped())
                    {
                    Norm.Negate();
                    }
                double dDot=(Pos-OtherPos)%Norm;
                if(bOnEdge)
                    {
                    if(pInCloseEdge)
                        {
                        *pInCloseEdge=pCloseEdge;
                        }
                    *bOnEdge=true;
                    }
                if(fabs(OldNorm%Norm)<0.9)
                    {
                    return dDot<SGM_MIN_TOL;
                    }
                }
            }

        SGM::EdgeSideType nType=GetSideType(pCloseEdge);
        SGM::Point2D a2=AdvancedInverse(pCloseEdge,nType,ClosePos);

        FixUV(m_pSurface,CloseUV1,a2);

        SGM::Point2D b2=a2+m_pSurface->FindSurfaceDirection(a2,Vec);
        SGM::Point3D A2(a2.m_u,a2.m_v,0);
        SGM::Point3D B2(b2.m_u,b2.m_v,0);
        SGM::Point3D C2(uv.m_u,uv.m_v,0);

        double dTest2=((B2-A2)*(C2-A2)).m_z;
        if(bOnEdge)
            {
            double dDist=ClosePos.Distance(Pos);
            double dDiag=pCloseEdge->GetBox(rResult).Diagonal();
            if(dDist/dDiag<SGM_FIT || std::abs(dTest2)<SGM_FIT)
                {
                if(pInCloseEdge)
                    {
                    *pInCloseEdge=pCloseEdge;
                    }
                *bOnEdge=true;
                }
            else
                {
                *bOnEdge=false;
                }
            }

        if(SGM_MIN_TOL<dTest2)
            {
            return nType == SGM::FaceOnLeftType ? !m_bFlipped : m_bFlipped;
            }
        else if(dTest2<-SGM_MIN_TOL)
            {
            return nType == SGM::FaceOnLeftType ? m_bFlipped : !m_bFlipped;
            }
        else
            {
            return true;
            }
        }
    else
        {
        if(bOnEdge)
            {
            if(pInCloseEdge)
                {
                *pInCloseEdge=pCloseEdge;
                }
            *bOnEdge=true;
            }
        SGM::Point3D const &VertexPos=pCloseVertex->GetPoint();
        if(SGM::NearEqual(Pos,VertexPos,SGM_MIN_TOL))
            {
            return true;
            }
        std::vector<edge *> aEdges;
        FindEdgesOnFaceAtVertex(rResult,pCloseVertex,this,aEdges);
        if(aEdges.size()==2)
            {
            edge *pEdge0=aEdges[0];
            edge *pEdge1=aEdges[1];

            // The edges must be ordered counter clockwise

            if(GetSideType(pEdge0)==SGM::FaceOnLeftType)
                {
                if(pEdge0->GetStart()==pCloseVertex)
                    {
                    std::swap(pEdge0,pEdge1);
                    }
                }
            else
                {
                if(pEdge0->GetEnd()==pCloseVertex)
                    {
                    std::swap(pEdge0,pEdge1);
                    }
                }
            
            SGM::Point3D Pos0;
            if(pEdge0->GetStart()==pCloseVertex)
                {
                Pos0=pEdge0->FindMidPoint(0.001);
                }
            else
                {
                Pos0=pEdge0->FindMidPoint(0.999);
                }
            SGM::Point2D uv0=AdvancedInverse(pEdge0,GetSideType(pEdge0),Pos0);

            FixUV(m_pSurface,CloseUV1,uv0);

            SGM::Point3D Pos1;
            if(pEdge1->GetStart()==pCloseVertex)
                {
                Pos1=pEdge1->FindMidPoint(0.001);
                }
            else
                {
                Pos1=pEdge1->FindMidPoint(0.999);
                }
            SGM::Point2D uv1=AdvancedInverse(pEdge1,GetSideType(pEdge1),Pos1);

            FixUV(m_pSurface,CloseUV1,uv1);

            SGM::UnitVector3D Vec0(uv0.m_u-CloseUV1.m_u,uv0.m_v-CloseUV1.m_v,0);
            SGM::UnitVector3D Vec1(uv1.m_u-CloseUV1.m_u,uv1.m_v-CloseUV1.m_v,0);
            SGM::UnitVector3D Vec2(uv .m_u-CloseUV1.m_u,uv .m_v-CloseUV1.m_v,0);
            SGM::UnitVector3D Vec3(0,0,-1);
            if(m_bFlipped)
                {
                Vec3.Negate();
                }
            double dAngle1=Vec0.Angle(Vec1,Vec3);
            double dAngle2=Vec0.Angle(Vec2,Vec3);
            bool bAnswer= dAngle2<dAngle1;

            if(bAnswer==false && bOnEdge)
                {
                double dDist=pCloseVertex->GetPoint().Distance(Pos);
                if(SGM_FIT<dDist)
                    {
                    *bOnEdge=false;
                    }
                }
            return bAnswer;
            }
        return true;
        }
    }

SGM::BoxTree const &face::GetFacetTree(SGM::Result &rResult) const
    {
    if(m_FacetTree.IsEmpty())
        {
        size_t nIndices = GetTriangles(rResult).size(); // This will cause the facet to be created
                                                        // if they do not already exist.
        
        // Get a box for each triangle
        for (size_t Index1 = 0; Index1 < nIndices; )
            {
            SGM::Point3D const &A = m_aPoints3D[m_aTriangles[Index1++]];
            SGM::Point3D const &B = m_aPoints3D[m_aTriangles[Index1++]];
            SGM::Point3D const &C = m_aPoints3D[m_aTriangles[Index1++]];
            SGM::Interval3D TriangleBox(A,B,C);

            // Convex edges, faces error:
            // estimated error in height when edge of triangle is chord of curvature
            // we use sum of Box X,Y,Z length as conservative estimate on chord length
            // error in height h = C * 0.5 * (1 - cos(theta))/sin(theta) where
            // C is chord of curvature spanned by angle tolerance on face facets
            TriangleBox.Extend(FACET_FACE_HEIGHT_ERROR_FACTOR * TriangleBox.FourthPerimeter());

            m_FacetTree.Insert(&A,TriangleBox);
            }
        }
    return m_FacetTree;
    }

SGM::Interval2D const &face::GetUVBox(SGM::Result &rResult) const
    {
    if(m_UVBox.IsEmpty())
        {
        GetTriangles(rResult).size(); // This will cause the facet to be created
                                      // if they do not already exist.

        m_UVBox=SGM::Interval2D(m_aPoints2D);
        double dDiagonal=m_UVBox.Diagonal();
        m_UVBox.Extend(dDiagonal*0.5);
        }
    return m_UVBox;
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
        m_mSeamType.clear();
        m_FacetTree.Clear();
        }
    m_Box.Reset();
    m_UVBox.Reset();
    if(m_pVolume)
        {
        m_pVolume->ResetBox(rResult);
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

SGM::UnitVector3D face::FindNormal(SGM::Point3D const &Pos) const
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
    pEdge->AddFace(rResult,this);
    ClearFacets(rResult);
    m_sVertices.clear();
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
    m_mUVBoundary.erase(pEdge);
    pEdge->RemoveFace(this);
    ClearFacets(rResult);
    m_sVertices.clear();
    }

void face::SetSurface(SGM::Result &rResult,
                      surface     *pSurface)
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
    m_mSeamType.clear();
    m_mUVBoundary.clear();
    ClearFacets(rResult);
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

std::vector<SGM::Point2D> const &face::GetUVBoundary(SGM::Result &rResult,
                                                     edge        *pEdge) const
    {
    auto const &IterEdge = m_mUVBoundary.find(pEdge);
    if(IterEdge==m_mUVBoundary.end())
        {
        return const_cast<face*>(this)->SetUVBoundary(rResult,pEdge);
        }
    return IterEdge->second;
    }

std::vector<SGM::Point2D> const &face::SetUVBoundary(SGM::Result &rResult,
                                                     edge  const *pEdge)
    {
    std::vector<SGM::Point3D> const &aPoints3D=pEdge->GetFacets(rResult);
    SGM::EdgeSideType nSideType=GetSideType(pEdge);
    std::vector<SGM::Point2D> aParams;
    size_t numPoints = aPoints3D.size();
    aParams.reserve(numPoints);
    SGM::Point2D FirstUV = AdvancedInverse(pEdge,nSideType,aPoints3D[0]);
    aParams.push_back(FirstUV);
    size_t Index1;
    if (numPoints==2)
        {
        SGM::Point2D LastUV = AdvancedInverse(pEdge,nSideType,aPoints3D[1]);
        aParams.push_back(LastUV);
        }
    else
        {
        for(Index1=1;Index1<numPoints;++Index1)
            {
            SGM::Point2D UV = AdvancedInverse(pEdge,nSideType,aPoints3D[Index1]);
            aParams.push_back(UV);
            }
        }
    auto IterEdgePair = m_mUVBoundary.emplace((edge *)pEdge,aParams);
    return IterEdgePair.first->second;
    }

void face::ClearUVBoundary(edge const *pEdge)
    {
    m_mUVBoundary.erase((edge *)pEdge);
    }

SGM::Interval2D face::FindUVBox(SGM::Result &rResult) const
    {
    SGM::Interval2D UVBox;
    for(edge *pEdge : m_sEdges)
        {
        std::vector<SGM::Point2D> const &aPoints2D=GetUVBoundary(rResult,pEdge);
        UVBox+=SGM::Interval2D(aPoints2D);
        }
    return UVBox;
    }

bool face::IsSliver(SGM::Result &rResult) const
    {
    if(m_sEdges.size()==2)
        {
        auto iter=m_sEdges.begin();
        edge *pEdge0=*iter;
        ++iter;
        edge *pEdge1=*iter;
        if( pEdge0->GetCurve()->GetCurveType()==SGM::LineType && 
            pEdge1->GetCurve()->GetCurveType()==SGM::LineType)
            {
            return true;
            }
        }
    else if(m_sEdges.size()==3)
        {
        if(SliverValue(rResult)<0.01)
            {
            return true;
            }
        }
    return false;
    }

double face::SliverValue(SGM::Result &rResult) const
    {
    GetTriangles(rResult);
    std::vector<unsigned> aAdjacencies;
    SGM::FindAdjacencies2D(m_aTriangles,aAdjacencies);
    std::vector<entity *> aEntities;
    FindPointEntities(rResult,aEntities);
    size_t Index1;
    size_t nTriangles=m_aTriangles.size();
    double dArea=0;
    double dLength=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned a=m_aTriangles[Index1];
        unsigned b=m_aTriangles[Index1+1];
        unsigned c=m_aTriangles[Index1+2];
        SGM::Point3D const &A=m_aPoints3D[a];
        SGM::Point3D const &B=m_aPoints3D[b];
        SGM::Point3D const &C=m_aPoints3D[c];
        dArea+=fabs(((B-A)*(C-A)).Magnitude()*0.5);
        if(aAdjacencies[Index1]==std::numeric_limits<unsigned int>::max())
            {
            if(aEntities[a]!=this && aEntities[b]!=this)
                {
                dLength+=A.Distance(B);
                }
            }
        if(aAdjacencies[Index1+1]==std::numeric_limits<unsigned int>::max())
            {
            if(aEntities[c]!=this && aEntities[b]!=this)
                {
                dLength+=C.Distance(B);
                }
            }
        if(aAdjacencies[Index1+2]==std::numeric_limits<unsigned int>::max())
            {
            if(aEntities[a]!=this && aEntities[c]!=this)
                {
                dLength+=A.Distance(C);
                }
            }
        }
    double dAnswer=4*SGM_PI*dArea/(dLength*dLength);
    if(1<dAnswer)
        {
        dAnswer=1.0;
        }
    return dAnswer;
    }

bool face::IsTight(SGM::Result &rResult) const
    {
    std::vector<SGM::Point2D> aPoints2D;
    std::vector<SGM::Point3D> aPoints3D;
    std::vector<std::vector<unsigned int> > aaPolygons;
    if( FacetFaceLoops(rResult,this,aPoints2D,aPoints3D,aaPolygons,nullptr,nullptr) &&
        aaPolygons.size()==1)
        {
        std::vector<SGM::Point3D> aFirstPoints,aSecondPoints;
        std::vector<double> aRatios;
        if(SGM::FindGlobalPinchPoints(SGM::PointsFromPolygon3D(aPoints3D,aaPolygons[0]),0.01,aFirstPoints,aSecondPoints,aRatios))
            {
            return true;
            }
        }
    return false;
    }

bool face::IsParametricRectangle(SGM::Result &rResult,double dPercentTol) const
    {
    if(m_sEdges.size()<4)
        {
        return false;
        }
    SGM::Interval2D const &Domain=m_pSurface->GetDomain();
    double dTolU=Domain.m_UDomain.Length()*dPercentTol;
    double dTolV=Domain.m_VDomain.Length()*dPercentTol;
    for(auto pEdge : m_sEdges)
        {
        auto aUVs=GetUVBoundary(rResult,pEdge);
        for(SGM::Point2D const &uv : aUVs)
            {
            if( Domain.m_UDomain.OnBoundary(uv.m_u,dTolU)==false &&
                Domain.m_VDomain.OnBoundary(uv.m_v,dTolV)==false)
                {
                return false;
                }
            }
        }
    return true;
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

#if 1
SGM::EdgeSeamType FindEdgeSeamType(edge const *pEdge,
                                   face const *pFace)
    {
    surface const *pSurface=pFace->GetSurface();
    if(pSurface->ClosedInU() || pSurface->ClosedInV())
        {
        SGM::EdgeSideType nSideType=pFace->GetSideType(pEdge);
        double dFraction=0.5;
        SGM::Point3D MidPos=pEdge->FindMidPoint(dFraction); 
        SGM::Point2D uv=pSurface->Inverse(MidPos);
        SGM::Interval2D const &Domain=pSurface->GetDomain();
        bool bFlipped=pFace->GetFlipped();
        if( pSurface->ClosedInU() && 
            pSurface->ClosedInV() && 
            Domain.m_UDomain.OnBoundary(uv.m_u,SGM_MIN_TOL)  && 
            Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL))
            {
            dFraction=0.4;
            MidPos=pEdge->FindMidPoint(dFraction);
            uv=pSurface->Inverse(MidPos);
            }
         
        if(Domain.OnBoundary(uv,SGM_MIN_TOL))
            {
            SGM::Point3D MidPosPlus=pEdge->FindMidPoint(dFraction+SGM_FIT);
            SGM::Point2D uvPlus=pSurface->Inverse(MidPosPlus);
            if(Domain.OnBoundary(uvPlus,SGM_MIN_TOL))
                {
                if( pSurface->ClosedInU() &&                                // There is a seam
                    Domain.m_UDomain.OnBoundary(uv.m_u,SGM_MIN_TOL) &&      // It is on the seam
                    fabs(uv.m_v-uvPlus.m_v)<Domain.m_UDomain.Length()*0.5)  // It is staying on the seam.
                    {
                    if(uv.m_v<uvPlus.m_v ) // Going up
                        {
                        if(nSideType==SGM::FaceOnLeftType)
                            {
                            return bFlipped ? SGM::EdgeSeamType::LowerUSeamType : SGM::EdgeSeamType::UpperUSeamType;
                            }
                        else
                            {
                            return bFlipped ? SGM::EdgeSeamType::UpperUSeamType : SGM::EdgeSeamType::LowerUSeamType;
                            }
                        }
                    else // Going down
                        {
                        if(nSideType==SGM::FaceOnLeftType)
                            {
                            return bFlipped ? SGM::EdgeSeamType::UpperUSeamType : SGM::EdgeSeamType::LowerUSeamType;
                            }
                        else
                            {
                            return bFlipped ? SGM::EdgeSeamType::LowerUSeamType : SGM::EdgeSeamType::UpperUSeamType;
                            }
                        }
                    }
                else if( pSurface->ClosedInV() && 
                         Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL) && 
                         fabs(uv.m_u-uvPlus.m_u)<Domain.m_VDomain.Length()*0.5)
                    {
                    if(uv.m_u<uvPlus.m_u ) // Going right.
                        {
                        if(nSideType==SGM::FaceOnLeftType)
                            {
                            return bFlipped ? SGM::EdgeSeamType::UpperVSeamType : SGM::EdgeSeamType::LowerVSeamType;
                            }
                        else
                            {
                            return bFlipped ? SGM::EdgeSeamType::LowerVSeamType : SGM::EdgeSeamType::UpperVSeamType;
                            }
                        }
                    else // Going left
                        {
                        if(nSideType==SGM::FaceOnLeftType)
                            {
                            return bFlipped ? SGM::EdgeSeamType::LowerVSeamType : SGM::EdgeSeamType::UpperVSeamType;
                            }
                        else
                            {
                            return bFlipped ? SGM::EdgeSeamType::UpperVSeamType : SGM::EdgeSeamType::LowerVSeamType;
                            }
                        }
                    }
                }
            }
        }
    return SGM::EdgeSeamType::NotASeamType;
    }

#else
SGM::EdgeSeamType FindEdgeSeamType(edge const *pEdge,
                                   face const *pFace)
    {
    surface const *pSurface=pFace->GetSurface();
    bool bFlip=pFace->GetFlipped();
    if(pFace->GetSideType(pEdge)==SGM::FaceOnRightType)
        {
        bFlip=!bFlip;
        }
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
                        return bFlip ? SGM::EdgeSeamType::LowerUSeamType : SGM::EdgeSeamType::UpperUSeamType;
                        }
                    else
                        {
                        return bFlip ? SGM::EdgeSeamType::UpperUSeamType : SGM::EdgeSeamType::LowerUSeamType;
                        }
                    }
                if(pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_MIN_TOL))
                    {
                    if(uv.m_u<uvPlus.m_u && fabs(uv.m_u-uvPlus.m_u)<Domain.m_VDomain.Length()*0.5)
                        {
                        return bFlip ? SGM::EdgeSeamType::UpperVSeamType : SGM::EdgeSeamType::LowerVSeamType;
                        }
                    else
                        {
                        return bFlip ? SGM::EdgeSeamType::LowerVSeamType : SGM::EdgeSeamType::UpperVSeamType;
                        }
                    }
                }
            }
        }
    return SGM::EdgeSeamType::NotASeamType;
    }
#endif

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

void face::SetSeamType(edge        const *pEdge,
                       SGM::EdgeSeamType  nSeamType) const
    {
    m_mSeamType[(edge *)pEdge]=nSeamType;
    }

SGM::Point2D face::AdvancedInverse(edge         const *pEdge,
                                   SGM::EdgeSideType   nType,
                                   SGM::Point3D const &Pos,
                                   bool                bFirstCall) const
    {
    SGM::EdgeSeamType nSeamType=GetSeamType(pEdge);
    SGM::Point2D uv=m_pSurface->Inverse(Pos);
    SGM::Interval2D const &Domain=m_pSurface->GetDomain();
    if(Domain.OnBoundary(uv,SGM_FIT))
        {
        if(nSeamType!=SGM::EdgeSeamType::NotASeamType)
            {
            if(nSeamType==SGM::EdgeSeamType::UpperUSeamType)
                {
                uv.m_u=Domain.m_UDomain.m_dMax;
                }
            else if(nSeamType==SGM::EdgeSeamType::UpperVSeamType)
                {
                uv.m_v=Domain.m_VDomain.m_dMax;
                }
            else if(nSeamType==SGM::EdgeSeamType::LowerUSeamType)
                {
                uv.m_u=Domain.m_UDomain.m_dMin;
                }
            else if(nSeamType==SGM::EdgeSeamType::LowerVSeamType)
                {
                uv.m_v=Domain.m_VDomain.m_dMin;
                }
            }
        else if((m_pSurface->ClosedInU() && Domain.m_UDomain.OnBoundary(uv.m_u,SGM_FIT)) ||
                (m_pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_FIT)))
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
            if(m_pSurface->ClosedInU() && Domain.m_UDomain.OnBoundary(uv.m_u,SGM_FIT))
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
            if(m_pSurface->ClosedInV() && Domain.m_VDomain.OnBoundary(uv.m_v,SGM_FIT))
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

        // Check for singularities.
        if( pEdge->GetStart() && 
            m_pSurface->SingularHighV() && 
            SGM::NearEqual(uv.m_v,Domain.m_VDomain.m_dMax,SGM_FIT,false) &&
            bFirstCall)
            {
            SGM::Point3D PosE;
            if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_FIT))
                {
                PosE=pEdge->FindMidPoint(0.01);
                }
            else 
                {
                PosE=pEdge->FindMidPoint(0.99);
                }
            SGM::Point2D uvE=AdvancedInverse(pEdge,nType,PosE,false);
            uv.m_u=uvE.m_u;
            }
        else if( pEdge->GetStart() && 
                 m_pSurface->SingularHighU() && 
                 SGM::NearEqual(uv.m_u,Domain.m_UDomain.m_dMax,SGM_FIT,false) &&
                 bFirstCall)
            {
            SGM::Point3D PosE;
            if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_FIT))
                {
                PosE=pEdge->FindMidPoint(0.01);
                }
            else 
                {
                PosE=pEdge->FindMidPoint(0.99);
                }
            SGM::Point2D uvE=AdvancedInverse(pEdge,nType,PosE,false);
            uv.m_v=uvE.m_v;
            }
        else if( pEdge->GetStart() && 
                 m_pSurface->SingularLowV() && 
                 SGM::NearEqual(uv.m_v,Domain.m_VDomain.m_dMin,SGM_FIT,false) &&
                 bFirstCall)
            {
            SGM::Point3D PosE;
            if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_FIT))
                {
                PosE=pEdge->FindMidPoint(0.01);
                }
            else 
                {
                PosE=pEdge->FindMidPoint(0.99);
                }
            SGM::Point2D uvE=AdvancedInverse(pEdge,nType,PosE,false);
            uv.m_u=uvE.m_u;
            }
        else if( pEdge->GetStart() && 
                 m_pSurface->SingularLowU() && 
                 SGM::NearEqual(uv.m_u,Domain.m_UDomain.m_dMin,SGM_FIT,false) &&
                 bFirstCall)
            {
            SGM::Point3D PosE;
            if(SGM::NearEqual(Pos,pEdge->GetStart()->GetPoint(),SGM_FIT))
                {
                PosE=pEdge->FindMidPoint(0.01);
                }
            else 
                {
                PosE=pEdge->FindMidPoint(0.99);
                }
            SGM::Point2D uvE=AdvancedInverse(pEdge,nType,PosE,false);
            uv.m_v=uvE.m_v;
            }

        // Deal with the case of a point at (0,0) on a torus.

        if( bFirstCall &&
            m_pSurface->ClosedInU() && 
            m_pSurface->ClosedInV() &&
            Domain.OnCorner(uv,SGM_FIT))
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
            SGM::Point2D TestUV=AdvancedInverse(pEdge,nType,TestPos,false);
            uv.m_u=TestUV.m_u<Domain.m_UDomain.MidPoint() ? Domain.m_UDomain.m_dMin : Domain.m_UDomain.m_dMax;
            uv.m_v=TestUV.m_v<Domain.m_VDomain.MidPoint() ? Domain.m_VDomain.m_dMin : Domain.m_VDomain.m_dMax;
            }
        }
    return uv;
    }

Signature const & face::GetSignature(SGM::Result &rResult) const
{
    if (!m_Signature.IsValid())
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

    if (m_pSurface->GetSurfaceType() != SGM::PlaneType)
    {
        if (aPoints.size())
        {
            SGM::Point3D Origin;
            SGM::UnitVector3D XAxis, YAxis, ZAxis;
            FindLeastSquarePlane(aPoints, Origin, XAxis, YAxis, ZAxis);

            std::vector<SGM::Point3D> aSurfPoints;
            std::vector<SGM::IntersectionType> aTypes;
            IntersectLineAndSurface(rResult, Origin, ZAxis, SGM::Interval1D(-SGM_MAX, SGM_MAX),
                                    m_pSurface, SGM_MIN_TOL, aSurfPoints, aTypes);

            std::vector<SGM::Point3D> aPosOnFace;
            for (auto Pos : aSurfPoints)
            {
                SGM::Point2D PosUV = m_pSurface->Inverse(Pos);
                if (PointInFace(rResult,PosUV))
                {
                    aPosOnFace.emplace_back(Pos);
                }
            }

            bool bClosestOnFace = false;
            SGM::Point3D Closest;
            double minDist= std::numeric_limits<double>::max();
            for (auto Pos : aPosOnFace)
            {
                double distance = Pos.Distance(Origin);
                if (distance < minDist)
                {
                    Closest = Pos;
                    bClosestOnFace = true;
                }
            }
            if (bClosestOnFace)
            {
                aPoints.emplace_back(Closest);
            }
        }
        else
        {
            if (m_pSurface->ClosedInU() && m_pSurface->SingularHighV() && m_pSurface->SingularLowV())
            {
                SGM::Interval2D Domain = m_pSurface->GetDomain();

                SGM::Point3D Pos;
                m_pSurface->Evaluate(Domain.LowerLeft(), &Pos);
                aPoints.emplace_back(Pos);
                m_pSurface->Evaluate(Domain.UpperLeft(), &Pos);
                aPoints.emplace_back(Pos);

                for (int index=0; index<4; index++)
                {
                    m_pSurface->Evaluate(Domain.MidPoint((double)index/4.0, 0.5), &Pos);
                    aPoints.emplace_back(Pos);
                }
            }
            else if (m_pSurface->ClosedInV() && m_pSurface->SingularHighU() && m_pSurface->SingularLowU())
            {
                SGM::Interval2D Domain = m_pSurface->GetDomain();

                SGM::Point3D Pos;
                m_pSurface->Evaluate(Domain.LowerLeft(), &Pos);
                aPoints.emplace_back(Pos);
                m_pSurface->Evaluate(Domain.LowerRight(), &Pos);
                aPoints.emplace_back(Pos);

                for (int index=0; index<4; index++)
                {
                    m_pSurface->Evaluate(Domain.MidPoint(0.5, (double)index/4.0), &Pos);
                    aPoints.emplace_back(Pos);
                }
            }
            else if (m_pSurface->ClosedInU() && m_pSurface->ClosedInV())
            {
                SGM::Interval2D domain = m_pSurface->GetDomain();
                for (size_t nV=0; nV<4; nV++)
                {
                    for (size_t nU=0; nU<4; nU++)
                    {
                        SGM::Point3D Pos;
                        m_pSurface->Evaluate(domain.MidPoint((double)nU/4.0, (double)nV/4.0), &Pos);
                        aPoints.emplace_back(Pos);
                    }
                }
            }
        }
    }
}

} // End SGMInternal namespace
