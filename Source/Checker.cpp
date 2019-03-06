#include "SGMEntityFunctions.h"
#include "SGMTriangle.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include "Topology.h"

#include <sstream>

namespace SGMInternal
{

// A convenience for writing, for example, "face 5", or "NUBcurve 23".

inline std::ostream& operator<<(std::ostream& out, const entity *pEntity)
    {
    return out << SGM::EntityTypeName(pEntity->GetType()) << " " << pEntity->GetID();
    }

// TODO - bring this back if we decide to prevent deleting an entity with parents
//void CheckPreexistingConditions(SGM::Result              &rResult,
//                                std::vector<std::string> &aCheckStrings)
//{
//    switch(rResult.GetResult())
//    {
//    case SGM::ResultTypeOK:
//    {
//        return;
//        break;
//    }
//    case SGM::ResultTypeDeleteWillCorruptModel:
//    {
//        aCheckStrings.emplace_back(rResult.Message().c_str());
//        aCheckStrings.emplace_back("An invalid delete was attempted but not performed.  Clearing Result.");
//        rResult.SetResult(SGM::ResultTypeOK);
//        return;
//    }
//    default:
//      return;
//    }
//}

bool thing::Check(SGM::Result              &rResult,
                  SGM::CheckOptions  const &Options,
                  std::vector<std::string> &aCheckStrings,
                  bool                      ) const
    {
    bool bAnswer = true;

    for (auto const &iter : m_mAllEntities)
        {
        if (!iter.second->Check(rResult, Options, aCheckStrings,false))
            bAnswer = false;
        }
    return bAnswer;
    }

bool CheckChildren(SGM::Result              &rResult,
                   entity             const *pEntity,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings)
    {
    std::set<entity *,EntityCompare> sChildren;
    pEntity->FindAllChildren(sChildren);
    bool bAnswer=true;
    for(auto pChild : sChildren)
        {
        if(pChild->Check(rResult,Options,aCheckStrings,false)==false)
            {
            bAnswer=false;
            }
        }
    return bAnswer;
    }

bool CheckChildHasOwner(entity             const *pChild,
                        entity             const *pOwner,
                        std::vector<std::string> &aCheckStrings)
{
    bool bAnswer = true;
    auto const &sOwners = pChild->GetOwners();
    if (sOwners.find(const_cast<entity *>(pOwner)) == sOwners.end())
    {
        std::stringstream ss;
        ss << pOwner << " owns " << pChild << " but the child does not point back to the owner.";
        aCheckStrings.emplace_back(ss.str());
        bAnswer = false;
    }
    return bAnswer;
}

bool CheckOwnersHaveChild(entity const *pChild, std::vector<std::string> &aCheckStrings)
    {
    bool bAnswer = true;
    auto const &sOwners = pChild->GetOwners();
    for (auto pOwner : sOwners)
        {
        std::set<entity *, SGMInternal::EntityCompare> sChildren;
        pOwner->FindAllChildren(sChildren);
        if (sChildren.find(const_cast<entity *>(pChild)) == sChildren.end())
            {
            std::stringstream ss;
            ss << pChild << " has owner " << pOwner << " but is not a child of the owner.";
            aCheckStrings.emplace_back(ss.str());
            bAnswer = false;
            }
        }
    return bAnswer;
    }

bool EdgesOverlap(edge const *pEdge1,edge const *pEdge2)
    {
    // Only check the end points of pEdge1, to see if they overlap.

    if(pEdge1->GetStart())
        {
        if(pEdge2->GetStart()!=pEdge1->GetStart() && pEdge2->GetStart()!=pEdge1->GetEnd())
            {
            SGM::Point3D Pos=pEdge1->GetStart()->GetPoint();
            SGM::Point3D CPos;
            double t=pEdge2->GetCurve()->Inverse(Pos,&CPos);
            if(Pos.Distance(CPos)<SGM_ZERO)
                {
                if(pEdge2->GetDomain().InInterval(t,SGM_MIN_TOL))
                    {
                    return true;
                    }
                }
            }
        }
    if(pEdge2->GetStart())
        {
        if(pEdge1->GetStart()!=pEdge2->GetStart() && pEdge1->GetStart()!=pEdge2->GetEnd())
            {
            SGM::Point3D Pos=pEdge2->GetStart()->GetPoint();
            SGM::Point3D CPos;
            double t=pEdge1->GetCurve()->Inverse(Pos,&CPos);
            if(Pos.Distance(CPos)<SGM_ZERO)
                {
                if(pEdge1->GetDomain().InInterval(t,SGM_MIN_TOL))
                    {
                    return true;
                    }
                }
            }
        }

    return false;
    }

bool OverlappingEdges(SGM::Result              &rResult,
                      entity             const *pEntity,
                      std::vector<std::string> &aCheckStrings)
    {
    std::set<edge *,EntityCompare> sEdges;
    FindEdges(rResult,pEntity,sEdges);

    // Note that this is n^2 and could be n*ln(n).

    for(edge *pEdge1 : sEdges)
        {
        for(edge *pEdge2 : sEdges)
            {
            if(pEdge1!=pEdge2)
                {
                if(EdgesOverlap(pEdge1,pEdge2))
                    {
                    std::stringstream ss;
                    ss << pEntity << " has overlapping edges, " << pEdge1 << " and " << pEdge2 << " .";
                    aCheckStrings.emplace_back(ss.str());
                    return true;
                    }
                }
            }
        }
    return false;
    }

bool body::Check(SGM::Result              &rResult,
                 SGM::CheckOptions  const &Options,
                 std::vector<std::string> &aCheckStrings,
                 bool                      bChildren) const
    {
    bool bAnswer=true;

    // Check to see if all its volumes point to it.

    for (auto pVolume: m_sVolumes)
        {
        if (this!=pVolume->GetBody())
            {
            bAnswer=false;
            std::stringstream ss;
            ss << pVolume << " of " << this << " does not point back to this body.";
            aCheckStrings.emplace_back(ss.str());
            }
        }

    if(m_sVolumes.empty() && m_aPoints.empty())
        {
        bAnswer=false;
        std::stringstream ss;
        ss << this << " has no volumes and no points.";
        aCheckStrings.emplace_back(ss.str());
        }

    //if(OverlappingEdges(rResult,this,aCheckStrings))
    //    {
    //    bAnswer=false;
    //    }

    if(bChildren)
        {
        if(!CheckChildren(rResult,this,Options,aCheckStrings))
            {
            bAnswer=false;
            }
        }

    return bAnswer;
    }

bool complex::Check(SGM::Result              &rResult,
                    SGM::CheckOptions  const &,//Options,
                    std::vector<std::string> &aCheckStrings,
                    bool                      /*bChildren*/) const
    {
    bool bAnswer=true;

    bAnswer = CheckIndexMax(rResult,m_aPoints.size());

    size_t nPoints=m_aPoints.size();
    size_t Index1;
    size_t nSegments=m_aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        if(nPoints<=m_aSegments[Index1])
            {
            bAnswer=false;
            std::stringstream ss;
            ss << SGM::EntityTypeName(GetType()) << ' ' << GetID() << " has out of bounds segment indexes.";
            aCheckStrings.emplace_back(ss.str());
            }
        }
    size_t nTriangles=m_aTriangles.size();
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        if(nPoints<=m_aTriangles[Index1])
            {
            bAnswer=false;
            std::stringstream ss;
            ss << SGM::EntityTypeName(GetType()) << ' ' << GetID() << " has out of bounds triangle indexes.";
            aCheckStrings.emplace_back(ss.str());
            }
        }

    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        if(m_aSegments[Index1]==m_aSegments[Index1+1])
            {
            bAnswer=false;
            std::stringstream ss;
            ss << SGM::EntityTypeName(GetType()) << ' ' << GetID() << " has a degenerate segment at index " << Index1;
            aCheckStrings.emplace_back(ss.str());
            }
        }

    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        if( m_aTriangles[Index1]==m_aTriangles[Index1+1] ||
            m_aTriangles[Index1]==m_aTriangles[Index1+2] ||
            m_aTriangles[Index1+2]==m_aTriangles[Index1+1])
            {
            bAnswer=false;
            std::stringstream ss;
            ss << SGM::EntityTypeName(GetType()) << ' ' << GetID() << " has a degenerate triangle at index " << Index1;
            aCheckStrings.emplace_back(ss.str());
            }
        }

    //if(bChildren)
    //    {
    //    if(CheckChildren(rResult,this,Options,aCheckStrings)==false)
    //        {
    //        bAnswer=false;
    //        }
    //    }

    return bAnswer;
    }

bool volume::Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
    {
    bool bAnswer=true;

    // Check to see if all its faces point to it.

    if(m_sFaces.empty() && m_sEdges.empty())
        {
        bAnswer=false;
        std::stringstream ss;
        ss << SGM::EntityTypeName(GetType()) << ' ' << GetID() << " is empty.";
        aCheckStrings.emplace_back(ss.str());
        }
    else
        {
        for (auto pFace : m_sFaces)
            {
            if(this!=pFace->GetVolume())
                {
                bAnswer=false;
                std::stringstream ss;
                ss << SGM::EntityTypeName(pFace->GetType()) << ' ' << pFace->GetID() << " of " <<
                      SGM::EntityTypeName(GetType()) << ' ' << GetID() << "does not point back to its volume.";
                aCheckStrings.emplace_back(ss.str());
                }
            }
        }

    if(bChildren)
        {
        if(CheckChildren(rResult,this,Options,aCheckStrings)==false)
            {
            bAnswer=false;
            }
        }

    return bAnswer;
    }

bool face::Check(SGM::Result              &rResult,
                 SGM::CheckOptions  const &Options,
                 std::vector<std::string> &aCheckStrings,
                 bool                      bChildren) const
    {
    bool bAnswer=true;
    
    // Check to see if all its edges point to it.

    for (auto pEdge : m_sEdges)
        {
        auto sFaces=pEdge->GetFaces();
        if(sFaces.find((face *)this)==sFaces.end())
            {
            std::stringstream ss;
            ss << SGM::EntityTypeName(pEdge->GetType()) << pEdge->GetID() << " of " <<
                  SGM::EntityTypeName(GetType()) << GetID() << " does not point back to its face.";
            aCheckStrings.emplace_back(ss.str());
            }
        }

    // Check the loops

    std::vector<std::vector<edge *> > aaLoops;
    std::vector<std::vector<SGM::EdgeSideType> > aaFlipped;
    size_t nLoops=FindLoops(rResult,aaLoops,aaFlipped);
    size_t Index1,Index2;
    size_t nTotalEdges=0;
    size_t nDoubleEdges=0;
    for(Index1=0;Index1<nLoops;++Index1)
        {
        std::vector<edge *> const &aLoop=aaLoops[Index1];
        std::vector<SGM::EdgeSideType> const &aFlipped=aaFlipped[Index1];
        size_t nEdges=aLoop.size();
        nTotalEdges+=nEdges;
        std::vector<vertex *> aStarts,aEnds;
        aStarts.reserve(nEdges);
        aEnds.reserve(nEdges);
        if(m_bFlipped)
            {
            for(Index2=0;Index2<nEdges;++Index2)
                {
                edge *pEdge=aLoop[Index2];
                if(GetSideType(pEdge)==SGM::FaceOnBothSidesType)
                    {
                    ++nDoubleEdges;
                    }
                aStarts.push_back(aFlipped[Index2] ? pEdge->GetStart() : pEdge->GetEnd());
                aEnds.push_back(aFlipped[Index2] ? pEdge->GetEnd() : pEdge->GetStart());
                }
            }
        else
            {
            for(Index2=0;Index2<nEdges;++Index2)
                {
                edge *pEdge=aLoop[Index2];
                if(GetSideType(pEdge)==SGM::FaceOnBothSidesType)
                    {
                    ++nDoubleEdges;
                    }
                aStarts.push_back(aFlipped[Index2] ? pEdge->GetEnd() : pEdge->GetStart());
                aEnds.push_back(aFlipped[Index2] ? pEdge->GetStart() : pEdge->GetEnd());
                }
            }
        
        for(Index2=0;Index2<nEdges;++Index2)
            {
            edge *pEdge=aLoop[Index2];
            vertex *pStart=aStarts[(Index2+1)%nEdges];
            vertex *pEnd=aEnds[Index2];
            if(pStart!=pEnd)
                {
                bAnswer=false;
                std::stringstream ss;
                ss << "start " << SGM::EntityTypeName(pStart->GetType()) << ' ' << pStart->GetID() << " and " <<
                      " end "  << SGM::EntityTypeName(pEnd->GetType())   << ' ' << pEnd->GetID() <<
                      " of "   << SGM::EntityTypeName(pEdge->GetType())  << ' ' << pEdge->GetID() <<
                      " on "   << SGM::EntityTypeName(GetType())         << ' ' << GetID() << " do not match.";
                aCheckStrings.emplace_back(ss.str());
                }
            }
        }
    nDoubleEdges/=2;

    if(nTotalEdges-nDoubleEdges!=m_sEdges.size())
        {
        bAnswer=false;
        std::stringstream ss;
        ss << "The loops of " << this << " are missing " << (nDoubleEdges+m_sEdges.size())-nTotalEdges << " edges.";
        aCheckStrings.emplace_back(ss.str());
        }

    // Check the facets

    std::vector<SGM::Point3D> const &aPoints=GetPoints3D(rResult);
    std::vector<SGM::UnitVector3D> const &aNormals=GetNormals(rResult);
    std::vector<unsigned int> const &aTriangles=GetTriangles(rResult);
    size_t nTriangles=aTriangles.size();
    double dMaxAngle=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        unsigned int a=aTriangles[Index1];
        unsigned int b=aTriangles[Index1+1];
        unsigned int c=aTriangles[Index1+2];
        SGM::Point3D const &A=aPoints[a];
        SGM::Point3D const &B=aPoints[b];
        SGM::Point3D const &C=aPoints[c];
        SGM::Vector3D TestNorm=(B-A)*(C-A);
        double dMagnitude=TestNorm.Magnitude();
        if(dMagnitude<SGM_FIT)
            {
            continue;
            }
        SGM::UnitVector3D Norm=TestNorm;
        double dDotA=a<aNormals.size() ? Norm%aNormals[a] : -1;
        double dDotB=b<aNormals.size() ? Norm%aNormals[b] : -1;
        double dDotC=c<aNormals.size() ? Norm%aNormals[c] : -1;
        double dTol=0.70710678118654752440084436210485; // cos(45) degrees
        if(dDotA<dTol || dDotB<dTol || dDotC<dTol)
            {
#if 0
            line *pLine1=new line(rResult,A,B);
            SGM::Interval1D Domain1(0.0,A.Distance(B));
            line *pLine2=new line(rResult,B,C);
            SGM::Interval1D Domain2(0.0,B.Distance(C));
            line *pLine3=new line(rResult,C,A);
            SGM::Interval1D Domain3(0.0,C.Distance(A));
            CreateEdge(rResult,pLine1,&Domain1);
            CreateEdge(rResult,pLine2,&Domain2);
            CreateEdge(rResult,pLine3,&Domain3);
#endif
            // Check to see if this is at a singularity 

            if( m_pSurface->SingularHighU()==false && 
                m_pSurface->SingularHighU()==false && 
                m_pSurface->SingularHighU()==false && 
                m_pSurface->SingularHighU()==false)
                {
                dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotA)*180/SGM_PI);
                dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotB)*180/SGM_PI);
                dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotC)*180/SGM_PI);
                }
            }
        }
    if(dMaxAngle!=0)
        {
        bAnswer=false;
        std::stringstream ss;
        ss << "Facets of " << this << " differ from surface normal by angle " << dMaxAngle;
        aCheckStrings.emplace_back(ss.str());
        }
    if(nTriangles==0)
        {
        bAnswer=false;
        std::stringstream ss;
        ss << this << " does not have facets.";
        aCheckStrings.emplace_back(ss.str());
        }
    else 
        {
        std::vector<unsigned int> aTemp=m_aTriangles;
        SGM::MergeTriangles3D(m_aPoints3D,aTemp,SGM_MIN_TOL);
        if(SGM::AreEdgeConnected(aTemp)==false)
            {
            bAnswer=false;
            std::stringstream ss;
            ss << this << " has facets that are not edge connected.";
            aCheckStrings.emplace_back(ss.str());
            }
        //std::vector<unsigned int> aBoundary;
        //std::set<unsigned int> sInterior;
        //SGM::FindBoundary(aTemp,aBoundary,sInterior);
        //size_t nComps=SGM::FindComponents1D(aBoundary);
        //if(nComps!=nLoops)
        //    {
        //    bAnswer=false;
        //    std::stringstream ss;
        //    ss << this << " has facets with boundary components " << nComps << " not equal to " << nLoops << " loops.";
        //    aCheckStrings.emplace_back(ss.str());
        //    }
        }

    if(bChildren)
        {
        if(CheckChildren(rResult,this,Options,aCheckStrings)==false)
            {
            bAnswer=false;
            }
        }

    return bAnswer;
    }

bool edge::Check(SGM::Result              &rResult,
                 SGM::CheckOptions  const &Options,
                 std::vector<std::string> &aCheckStrings,
                 bool                      bChildren) const
    {
    bool bAnswer=true;

    // Check to see if the start and end vertices point to this edge.

    if(m_pStart)
        {
        auto const &sEdges=m_pStart->GetEdges();
        if(sEdges.find((edge*)this) == sEdges.end())
            {
            bAnswer=false;
            std::stringstream ss;
            ss << "start " << m_pStart << " of " << this << " does not point back to the edge";
            aCheckStrings.emplace_back(ss.str());
            }
        }
    else if(!m_pCurve->GetClosed() || m_pEnd)
        {
        bAnswer=false;
        std::stringstream ss;
        ss << this << " does not have a start vertex";
        aCheckStrings.emplace_back(ss.str());
        }
    if(m_pEnd)
        {
        auto const &sEdges=m_pEnd->GetEdges();
        if(sEdges.find((edge*)this) == sEdges.end())
            {
            bAnswer=false;
            std::stringstream ss;
            ss << "The end " << m_pEnd << " of " << this << " does not point back to the edge";
            aCheckStrings.emplace_back(ss.str());
            }
        }
    else if(!m_pCurve->GetClosed() || m_pStart)
        {
        bAnswer=false;
        std::stringstream ss;
        ss << this << " does not have an end vertex";
        aCheckStrings.emplace_back(ss.str());
        }

    // check if the curve's edges are connected to the curve
    auto const &sEdges = m_pCurve->GetEdges();
    if (sEdges.find(const_cast<edge*>(this)) == sEdges.end())
    {
        std::stringstream ss;
        ss << this << " points to " << m_pCurve << " but the curve does not point back to the edge";
        aCheckStrings.emplace_back(ss.str());
        bAnswer = false;
    }

    // Check to see if it is consistently oriented with respect to its faces.

    if(m_sFaces.size()==2)
        {
        auto FaceIter=m_sFaces.begin();
        face *pFace1=*FaceIter;
        ++FaceIter;
        face *pFace2=*FaceIter;
        SGM::EdgeSideType bFlipped1=pFace1->GetSideType(this);
        SGM::EdgeSideType bFlipped2=pFace2->GetSideType(this);
        if(bFlipped1==bFlipped2 && bFlipped1!=SGM::EdgeSideType::FaceOnBothSidesType)
            {
            bAnswer=false;
            std::stringstream ss;
            ss << this << " has the same orientation of both faces.";
            aCheckStrings.emplace_back(ss.str());
            }
        }

    // Check to see if the edge is in only one volume.

    std::set<volume *> sVolumes;
    for(auto pFace : m_sFaces)
        {
        sVolumes.insert(pFace->GetVolume());
        }
    if(sVolumes.size()>1)
        {
        bAnswer=false;
        std::stringstream ss;
        ss << this << " belongs to more than one volume.";
        aCheckStrings.emplace_back(ss.str());
        }

    if(bChildren)
        {
        if(CheckChildren(rResult,this,Options,aCheckStrings)==false)
            {
            bAnswer=false;
            }
        }

    return bAnswer;
    }

bool vertex::Check(SGM::Result              &,//rResult,
                   SGM::CheckOptions  const &,//Options,
                   std::vector<std::string> &,//aCheckStrings,
                   bool                      /*bChildren*/) const
    {
    bool bAnswer=true;
    for (auto pEdge : m_sEdges)
        {
        if ((pEdge->GetStart() != this) && (pEdge->GetEnd() != this))
            {
            bAnswer = false;
            std::stringstream ss;
            ss << this << " points to " << pEdge << " but the edge does not point back to the vertex";
            }
        }
    return bAnswer;
    }

bool curve::Check(SGM::Result              &rResult,
                  SGM::CheckOptions  const &Options,
                  std::vector<std::string> &aCheckStrings,
                  bool                      bChildren) const
    {
    bool bAnswer=TestCurve(rResult,this,m_Domain.MidPoint());

    if(!bAnswer)
        {
        std::stringstream ss;
        ss << this << " does not pass derivative and inverse checks.";
        aCheckStrings.emplace_back(ss.str());
        }

    // check if the edges are connected
    if (!m_sEdges.empty())
    {
        for (auto pEdge : m_sEdges)
        {
            if(!(pEdge->GetCurve() == this))
            {
                std::stringstream ss;
                ss << this << " points to " << pEdge << " but the edge does not point back to the curve.";
                aCheckStrings.emplace_back(ss.str());
                bAnswer = false;
            }
        }
    }

    bAnswer = CheckOwnersHaveChild((entity*)this, aCheckStrings) && bAnswer;

    if(bChildren)
        {
        bAnswer = CheckChildren(rResult,this,Options,aCheckStrings) && bAnswer;
        }

    return bAnswer;
    }

bool surface::CheckImplementation(SGM::Result              &rResult,
                                  SGM::CheckOptions  const &Options,
                                  std::vector<std::string> &aCheckStrings,
                                  bool                      bChildren) const
    {
    SGM::Point2D uv=m_Domain.MidPoint();
    if(!m_sFaces.empty())
        {
        face *pFace=*(m_sFaces.begin());
        std::vector<SGM::Point2D> const &aPoints=pFace->GetPoints2D(rResult);
        if(!aPoints.empty())
            {
            uv=SGM::FindCenterOfMass2D(aPoints);
            }
        }
    bool bAnswer=TestSurface(rResult,this,uv);

    if(!bAnswer)
        {
        std::stringstream ss;
        ss << this << " does not pass derivative and inverse checks.";
        aCheckStrings.emplace_back(ss.str());
        }

    bAnswer = CheckOwnersHaveChild((entity*)this, aCheckStrings) && bAnswer;

    if(bChildren)
        {
        bAnswer = CheckChildren(rResult,this,Options,aCheckStrings) && bAnswer;
        }

    return bAnswer;
    }

bool cone::Check(SGM::Result              &rResult,
                 SGM::CheckOptions const  &Options,
                 std::vector<std::string> &aCheckStrings,
                 bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool cylinder::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool extrude::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    bool bAnswer = true;

    if (nullptr == m_pCurve)
    {
        std::stringstream ss;
        ss << "Extrude " << GetID() << " has a NULL curve pointer.";
        aCheckStrings.emplace_back(ss.str());
        rResult.SetResult(SGM::ResultTypeSurfaceMissingChild);
        bAnswer = false;
        return bAnswer;
    }

    if (!surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren))
    {
        bAnswer = false;
    }

    if (!CheckChildHasOwner(m_pCurve, this, aCheckStrings))
    {
        bAnswer = false;
    }

    return bAnswer;
}

bool NUBsurface::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool NURBsurface::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool offset::Check(SGM::Result              &,//rResult,
                   SGM::CheckOptions const  &,//Options,
                   std::vector<std::string> &,//aCheckStrings,
                   bool                      ) const//bChildren) const
{
    return false;
    //bool bAnswer = true;

    //if (nullptr == m_pSurface)
    //{
    //    std::stringstream ss;
    //    ss << "Offset " << GetID() << " has a NULL surface pointer.";
    //    aCheckStrings.emplace_back(ss.str());
    //    rResult.SetResult(SGM::ResultTypeSurfaceMissingChild);
    //    bAnswer = false;
    //    return bAnswer;
    //}

    //if (!surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren))
    //{
    //    bAnswer = false;
    //}

    //if (!CheckChildHasOwner(m_pSurface, this, aCheckStrings))
    //{
    //    bAnswer = false;
    //}

    //return bAnswer;
}

bool plane::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool revolve::Check(SGM::Result              &rResult,
                    SGM::CheckOptions const  &Options,
                    std::vector<std::string> &aCheckStrings,
                    bool                      bChildren) const
{
    bool bAnswer = true;

    if (nullptr == m_pCurve)
    {
        std::stringstream ss;
        ss << "Revolve " << GetID() << " has a NULL curve pointer.";
        aCheckStrings.emplace_back(ss.str());
        rResult.SetResult(SGM::ResultTypeSurfaceMissingChild);
        bAnswer = false;
        return bAnswer;
    }

    if (!surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren))
    {
        bAnswer = false;
    }

    if (!CheckChildHasOwner(m_pCurve, this, aCheckStrings))
    {
        bAnswer = false;
    }

    return bAnswer;
}

bool sphere::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

bool torus::Check(SGM::Result              &rResult,
                   SGM::CheckOptions const  &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
{
    return surface::CheckImplementation(rResult, Options, aCheckStrings, bChildren);
}

}
