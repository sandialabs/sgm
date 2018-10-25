#include "SGMVector.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include "Primitive.h"

#include <set>
#include <vector>
#include <map>
#include <string>
#include <cstdio>
#include <algorithm>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

bool CheckAllChildren(entity const             &parentEntity,
                      SGM::Result              &rResult,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    bool bAnswer=true;
    std::set<entity *,EntityCompare> sChildren;
    parentEntity.FindAllChildren(sChildren);
    for (auto pChild : sChildren)
        if (!pChild->Check(rResult,Options,aCheckStrings,false))
            bAnswer = false;
    return bAnswer;
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
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Volume %lu of Body %lu does not point to its body.\n",pVolume->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }

    if(m_sVolumes.empty() && m_aPoints.empty())
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer, sizeof(Buffer), "Body %lu is empty.\n",this->GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    // Check all children.

    if (bChildren)
        if (!CheckAllChildren(*this, rResult, Options, aCheckStrings))
            bAnswer = false;

    return bAnswer;
    }

bool complex::Check(SGM::Result              &,//rResult,
                    SGM::CheckOptions  const &,//Options,
                    std::vector<std::string> &aCheckStrings,
                    bool                      ) const //bChildren)
    {
    bool bAnswer=true;

    size_t nPoints=m_aPoints.size();
    size_t Index1;
    size_t nSegments=m_aSegments.size();
    for(Index1=0;Index1<nSegments;++Index1)
        {
        if(nPoints<=m_aSegments[Index1])
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Complex %lu has out of bounds segment indexes.\n",this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }
    size_t nTriangles=m_aTriangles.size();
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        if(nPoints<=m_aTriangles[Index1])
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Complex %lu has out of bounds triangle indexes.\n",this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }

    for(Index1=0;Index1<nSegments;Index1+=2)
        {
        if(m_aSegments[Index1]==m_aSegments[Index1+1])
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Complex %ld has a degenerate segment at index %ld.\n",this->GetID(),Index1);
            aCheckStrings.emplace_back(Buffer);
            }
        }

    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        if( m_aTriangles[Index1]==m_aTriangles[Index1+1] ||
            m_aTriangles[Index1]==m_aTriangles[Index1+2] ||
            m_aTriangles[Index1+2]==m_aTriangles[Index1+1])
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Complex %ld has a degenerate triangle at index %ld.\n",this->GetID(),Index1);
            aCheckStrings.emplace_back(Buffer);
            }
        }

    return bAnswer;
    }

bool volume::Check(SGM::Result              &rResult,
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      bChildren) const
    {
    bool bAnswer=true;

    // Check to see if all its faces point to it.

    for (auto pFace : m_sFaces)
        {
        if(this!=pFace->GetVolume())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Face %lu of Volume %lu does not point to its volume.\n",pFace->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }

    // Check all children.

    if (bChildren)
        if (!CheckAllChildren(*this, rResult, Options, aCheckStrings))
            bAnswer = false;

    return bAnswer;
    }

bool face::Check(SGM::Result              &rResult,
                 SGM::CheckOptions  const &Options,
                 std::vector<std::string> &aCheckStrings,
                 bool                      bChildren) const
    {
    bool bAnswer=true;
    size_t nBigTriangleVertices = 30000;

    // Check to see if size of triangles

    if (m_aTriangles.size() > nBigTriangleVertices)
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Face %lu had more than %lu triangles.\n",GetID(),nBigTriangleVertices/3);
        aCheckStrings.emplace_back(Buffer);
        }

    // Check to see if all its edges point to it.

    for (auto pEdge : m_sEdges)
        {
        auto sFaces=pEdge->GetFaces();
        if(sFaces.find((face *)this)==sFaces.end())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Edge %lu of face %lu does not point to its face.\n",pEdge->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
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
                char Buffer[1000];
                snprintf(Buffer,sizeof(Buffer),"Vertices of Edge %lu of Face %lu does not match its neighbors.\n",pEdge->GetID(),this->GetID());
                aCheckStrings.emplace_back(Buffer);
                }
            }
        }
    if(nTotalEdges-nDoubleEdges!=m_sEdges.size())
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"The loops of Face %lu are missing %lu edges.\n",this->GetID(),(nDoubleEdges+m_sEdges.size())-nTotalEdges);
        aCheckStrings.emplace_back(Buffer);
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
        double dDotA=Norm%aNormals[a];
        double dDotB=Norm%aNormals[b];
        double dDotC=Norm%aNormals[c];
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
            dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotA)*180/SGM_PI);
            dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotB)*180/SGM_PI);
            dMaxAngle = std::max(dMaxAngle, SGM::SAFEacos(dDotC)*180/SGM_PI);
            }
        }
    if(dMaxAngle!=0)
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Facets of Face %lu differ from surface normal by %lf.\n",this->GetID(),dMaxAngle);
        aCheckStrings.emplace_back(Buffer);
        }
    if(nTriangles==0)
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Face %lu does not have facets.\n",this->GetID());
        aCheckStrings.emplace_back(Buffer);
        }
    else 
        {
        std::vector<unsigned int> aTemp=m_aTriangles;
        SGM::MergeTriangles3D(m_aPoints3D,aTemp,SGM_MIN_TOL);
        if(SGM::AreEdgeConnected(aTemp)==false)
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Face %lu has facets that are not edge connected.\n",this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        //std::vector<unsigned int> aBoundary;
        //std::set<unsigned int> sInterior;
        //SGM::FindBoundary(aTemp,aBoundary,sInterior);
        //size_t nComps=SGM::FindComponents1D(aBoundary);
        //if(nComps!=nLoops)
        //    {
        //    bAnswer=false;
        //    char Buffer[1000];
        //    snprintf(Buffer,sizeof(Buffer),"Face %lu has facets with the wrong number of boundary components.\n",this->GetID());
        //    aCheckStrings.emplace_back(Buffer);
        //    }
        }

    // Check all children.

    if (bChildren)
        if (!CheckAllChildren(*this, rResult, Options, aCheckStrings))
            bAnswer = false;
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
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"The start Vertex %lu of Edge %lu does not point back to the edge\n",m_pStart->GetID(),GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }
    else if(!m_pCurve->GetClosed() || m_pEnd)
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Edge %lu does not have a start vertex\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }
    if(m_pEnd)
        {
        auto const &sEdges=m_pEnd->GetEdges();
        if(sEdges.find((edge*)this) == sEdges.end())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"The end Vertex %lu of Edge %lu does not point back to the edge\n",m_pEnd->GetID(),GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }
    else if(!m_pCurve->GetClosed() || m_pStart)
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Edge %lu does not have an end vertex\n",GetID());
        aCheckStrings.emplace_back(Buffer);
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
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Edge %lu has the same orientation of both faces.\n",GetID());
            aCheckStrings.emplace_back(Buffer);
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
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Edge %lu belongs to more than one volume.\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    // Check all children.

    if (bChildren)
        if (!CheckAllChildren(*this, rResult, Options, aCheckStrings))
            bAnswer = false;

    return bAnswer;
    }

bool vertex::Check(SGM::Result              &,//rResult,
                   SGM::CheckOptions  const &,//Options,
                   std::vector<std::string> &aCheckStrings,
                   bool                      /* bChildren */) const
    {
    bool bAnswer=true;

    if(m_sEdges.empty())
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Vertex %lu does not point to an edge\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    return bAnswer;
    }

bool curve::Check(SGM::Result              &,//rResult,
                  SGM::CheckOptions  const &,//Options,
                  std::vector<std::string> &aCheckStrings,
                  bool                      /* bChildren */) const
    {
    bool bAnswer=TestCurve(this,m_Domain.MidPoint());

    if(!bAnswer)
        {
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Curve %lu does not pass derivative and inverse checks.\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    return bAnswer;
    }

bool surface::Check(SGM::Result              &rResult,
                    SGM::CheckOptions  const &,
                    std::vector<std::string> &aCheckStrings,
                    bool                     /* bChildren */) const
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
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Surface %lu does not pass derivative and inverse checks.\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    return bAnswer;
    }

}