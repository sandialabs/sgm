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
        GetVolume()->RemoveFace(rResult,this);
        SetVolume(nullptr);        
    }
    topology::RemoveParentsInSet(rResult, sParents);
}

void face::SeverRelations(SGM::Result &rResult)
    {
    if(GetVolume())
        GetVolume()->RemoveFace(rResult,this);
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
                       SGM::Point2D const &uv) const
    {
    // First check for the closed face case.

    if(m_sEdges.empty())
        {
        return true;
        }

    // Find the closest edge or vertex.

    SGM::Interval1D const &UDomain=m_pSurface->GetDomain().m_UDomain;
    SGM::Interval1D const &VDomain=m_pSurface->GetDomain().m_VDomain;
    double dULength=UDomain.Length();
    double dVLength=VDomain.Length();
    double dUMaxLength=dULength*0.5;
    double dVMaxLength=dVLength*0.5;
    double dMinDist=std::numeric_limits<double>::max();
    edge *pCloseEdge=nullptr;
    SGM::Segment2D CloseSeg;
    size_t nSegment=0;
    for(auto pEdge : m_sEdges)
        {
        std::vector<SGM::Point2D> const &aUVParams=GetUVBoundary(rResult,pEdge);
        size_t nUVParams=aUVParams.size();
        size_t Index1;
        for(Index1=1;Index1<nUVParams;++Index1)
            {
            SGM::Point2D uv0=aUVParams[Index1-1];
            SGM::Point2D uv1=aUVParams[Index1];
            if(m_pSurface->ClosedInU())
                {
                if(dUMaxLength<fabs(uv0.m_u-uv1.m_u))
                    {
                    if(SGM::NearEqual(UDomain.m_dMin,uv0.m_u,SGM_MIN_TOL,false))
                        {
                        uv0.m_u+=dULength;
                        }
                    else if(SGM::NearEqual(UDomain.m_dMax,uv0.m_u,SGM_MIN_TOL,false))
                        {
                        uv0.m_u-=dULength;
                        }
                    else if(SGM::NearEqual(UDomain.m_dMin,uv1.m_u,SGM_MIN_TOL,false))
                        {
                        uv1.m_u+=dULength;
                        }
                    else if(SGM::NearEqual(UDomain.m_dMax,uv1.m_u,SGM_MIN_TOL,false))
                        {
                        uv1.m_u-=dULength;
                        }
                    }
                }
            if(m_pSurface->ClosedInV())
                {
                if(dVMaxLength<fabs(uv0.m_v-uv1.m_v))
                    {
                    if(SGM::NearEqual(VDomain.m_dMin,uv0.m_v,SGM_MIN_TOL,false))
                        {
                        uv0.m_v+=dVLength;
                        }
                    else if(SGM::NearEqual(VDomain.m_dMax,uv0.m_v,SGM_MIN_TOL,false))
                        {
                        uv0.m_v-=dVLength;
                        }
                    else if(SGM::NearEqual(VDomain.m_dMin,uv1.m_v,SGM_MIN_TOL,false))
                        {
                        uv1.m_v+=dVLength;
                        }
                    else if(SGM::NearEqual(VDomain.m_dMax,uv1.m_v,SGM_MIN_TOL,false))
                        {
                        uv1.m_v-=dVLength;
                        }
                    }
                }
            SGM::Segment2D Seg(uv0,uv1);
            SGM::Point2D cuv;
            double dDist=Seg.DistanceSquared(uv);
            if(dDist<dMinDist)
                {
                dMinDist=dDist;
                pCloseEdge=pEdge;
                CloseSeg=Seg;
                nSegment=Index1;
                }
            }
        }

    // Check to see if the point is on the close edge, or if the
    // close edge is double sided.

    SGM::EdgeSideType nType=GetSideType(pCloseEdge);
    if(nType==SGM::FaceOnBothSidesType)
        {
        return true;
        }
    SGM::Point3D Pos;
    m_pSurface->Evaluate(uv,&Pos);
    SGM::Point3D ClosePos;
    entity *pCloseEnt;
    FindClosestPointOnEdge3D(rResult,Pos,pCloseEdge,ClosePos,pCloseEnt);
    if(SGM::NearEqual(Pos,ClosePos,SGM_MIN_TOL))
        {
        return true;
        }

    // Look for close vertex.

    std::set<vertex *,EntityCompare> sVertices;
    FindVertices(rResult,this,sVertices);
    vertex *pFoundVertex=nullptr;
    SGM::Point2D FoundVertexUV(0,0);
    for(vertex *pVertex : sVertices)
        {
        std::vector<edge *> aVertexEdge;
        FindEdgesOnFaceAtVertex(rResult,pVertex,this,aVertexEdge);
        edge *pVertexEdge=aVertexEdge[0];
        SGM::EdgeSideType nVertexEdgeType=GetSideType(pVertexEdge);
        SGM::Point2D VertexUV=EvaluateParamSpace(pVertexEdge,nVertexEdgeType,pVertex->GetPoint());
        if(SGM::NearEqual(uv.DistanceSquared(VertexUV),dMinDist,SGM_MIN_TOL,false))
            {
            FoundVertexUV=VertexUV;
            pFoundVertex=pVertex;
            break;
            }
        }
        
    if(pFoundVertex==nullptr)
        {
        // Test to see if we are to the left or right of pEdge at CloseUV.

        SGM::Point2D CloseUV;
        CloseSeg.Distance(uv,&CloseUV);
        SGM::Point3D TestPos;
        m_pSurface->Evaluate(CloseUV,&TestPos);
        double t=pCloseEdge->GetCurve()->Inverse(TestPos);
        SGM::Vector3D Vec;
        pCloseEdge->GetCurve()->Evaluate(t,&TestPos,&Vec);

        //m_pSurface->Inverse(TestPos,nullptr,&CloseSeg.m_Start);
        SGM::Point2D a2=EvaluateParamSpace(pCloseEdge,nType,TestPos);
        SGM::Point2D b2=a2+m_pSurface->FindSurfaceDirection(a2,Vec);
        SGM::Point3D A2(a2.m_u,a2.m_v,0);
        SGM::Point3D B2(b2.m_u,b2.m_v,0);
        SGM::Point3D C2(uv.m_u,uv.m_v,0);
        double dTest2=((B2-A2)*(C2-A2)).m_z;
        if(SGM_MIN_TOL<dTest2)
            {
            if(nType==SGM::FaceOnLeftType)
                {
                return m_bFlipped ? false : true;
                }
            else
                {
                return m_bFlipped ? true : false;
                }
            }
        else if(dTest2<-SGM_MIN_TOL)
            {
            if(nType==SGM::FaceOnLeftType)
                {
                return m_bFlipped ? true : false;
                }
            else
                {
                return m_bFlipped ? false : true;
                }
            }
        else
            {
            return true;
            }
        }
    else
        {
        if(pFoundVertex)
            {
            if(SGM::NearEqual(Pos,pFoundVertex->GetPoint(),SGM_MIN_TOL))
                {
                return true;
                }
            std::vector<edge *> aEdges;
            FindEdgesOnFaceAtVertex(rResult,pFoundVertex,this,aEdges);
            if(aEdges.size()==2)
                {
                edge *pEdge0=aEdges[0];
                edge *pEdge1=aEdges[1];
                SGM::Point2D uv0,uv1;
                bool bOut0=true,bOut1=true;
                if(pEdge0->GetStart()==pFoundVertex)
                    {
                    //uv0=m_pSurface->Inverse(pEdge0->FindMidPoint(0.001));
                    uv0=EvaluateParamSpace(pEdge0,GetSideType(pEdge0),pEdge0->FindMidPoint(0.001));
                    }
                else
                    {
                    //uv0=m_pSurface->Inverse(pEdge0->FindMidPoint(0.999));
                    uv0=EvaluateParamSpace(pEdge0,GetSideType(pEdge0),pEdge0->FindMidPoint(0.999));
                    bOut0=false;
                    }
                if(pEdge1->GetStart()==pFoundVertex)
                    {
                    //uv1=m_pSurface->Inverse(pEdge1->FindMidPoint(0.001));
                    uv1=EvaluateParamSpace(pEdge1,GetSideType(pEdge1),pEdge1->FindMidPoint(0.001));
                    }
                else
                    {
                    //uv1=m_pSurface->Inverse(pEdge1->FindMidPoint(0.999));
                    uv1=EvaluateParamSpace(pEdge1,GetSideType(pEdge1),pEdge1->FindMidPoint(0.999));
                    bOut1=false;
                    }
                if(GetSideType(pEdge0)==SGM::FaceOnRightType)
                    {
                    bOut0=!bOut0;
                    }
                if(GetSideType(pEdge1)==SGM::FaceOnRightType)
                    {
                    bOut1=!bOut1;
                    }
                bool bAnswer;
                if(bOut1)
                    {
                    bAnswer=InAngle(FoundVertexUV,uv1,uv0,uv);
                    }
                else
                    {
                    bAnswer=InAngle(FoundVertexUV,uv0,uv1,uv);
                    }
                if(m_bFlipped)
                    {
                    bAnswer=!bAnswer;
                    }
                return bAnswer;
                }
            else
                {
                for(auto pEdge : aEdges)
                    {
                    if(GetSideType(pEdge)==SGM::FaceOnBothSidesType)
                        {
                        return true;
                        }
                    }
                throw; // Add more code
                }
            }
        return true;
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
        m_mSeamType.clear();
        }
    m_Box.Reset();
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
    pEdge->AddFace(rResult,this);
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
    m_mUVBoundary.erase(pEdge);
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

std::vector<SGM::Point2D> const &face::GetUVBoundary(SGM::Result &rResult,
                                                     edge        *pEdge) const
    {
    if(m_mUVBoundary.find(pEdge)==m_mUVBoundary.end())
        {
        pEdge->GetFacets(rResult);
        }
    return m_mUVBoundary[pEdge];
    }

void face::SetUVBoundary(edge                const *pEdge,
                         std::vector<SGM::Point2D> &aSurfParams)
    {
    m_mUVBoundary[(edge *)pEdge]=aSurfParams;
    }

void face::ClearUVBoundary(edge const *pEdge)
    {
    m_mUVBoundary.erase((edge *)pEdge);
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
