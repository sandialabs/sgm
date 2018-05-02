#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"
#include <set>
#include <vector>
#include <map>
#include <string>
#include <cstdio>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGM { namespace Impl {

bool thing::Check(SGM::Result              &rResult,
                  SGM::CheckOptions  const &Options,
                  std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;
    auto iter=m_mAllEntities.begin();
    while(iter!=m_mAllEntities.end())
        {
        entity *pEntity=iter->second;
        switch(pEntity->GetType())
            {
            case SGM::BodyType:
                {
                body const *pBody=(body *)pEntity;
                if(false==pBody->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::VolumeType:
                {
                volume const *pVolume=(volume *)pEntity;
                if(false==pVolume->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::FaceType:
                {
                face const *pFace=(face *)pEntity;
                if(false==pFace->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::EdgeType:
                {
                edge const *pEdge=(edge *)pEntity;
                if(false==pEdge->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::VertexType:
                {
                vertex const *pVertex=(vertex *)pEntity;
                if(false==pVertex->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::CurveType:
                {
                curve const *pCurve=(curve *)pEntity;
                if(false==pCurve->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            case SGM::SurfaceType:
                {
                surface const *pSurface=(surface *)pEntity;
                if(false==pSurface->Check(rResult,Options,aCheckStrings))
                    {
                    bAnswer=false;
                    }
                break;
                }
            default:
                {
                throw;
                }
            }
        ++iter;
        }
    return bAnswer;
    }

bool body::Check(SGM::Result              &,//rResult,
                 SGM::CheckOptions  const &,//Options,
                 std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;

    // Check to see if all its volumes point to it.

    std::set<volume *>::const_iterator VolumeIter=m_sVolumes.begin();
    while(VolumeIter!=m_sVolumes.end())
        {
        volume *pVolume=*VolumeIter;
        if(this!=pVolume->GetBody())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer, sizeof(Buffer), "Volume %ld of Body %ld does not point to its body.\n",pVolume->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        ++VolumeIter;
        }

    return bAnswer;
    }

bool complex::Check(SGM::Result              &,//rResult,
                    SGM::CheckOptions  const &,//Options,
                    std::vector<std::string> &) const//aCheckStrings) const
    {
    bool bAnswer=true;

    return bAnswer;
    }

bool volume::Check(SGM::Result              &,//rResult,
                   SGM::CheckOptions  const &,//Options,
                   std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;

    // Check to see if all its faces point to it.

    std::set<face *>::const_iterator FaceIter=m_sFaces.begin();
    while(FaceIter!=m_sFaces.end())
        {
        face *pFace=*FaceIter;
        if(this!=pFace->GetVolume())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Face %ld of Volume %ld does not point to its volume.\n",pFace->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        ++FaceIter;
        }

    return bAnswer;
    }

bool face::Check(SGM::Result              &rResult,
                 SGM::CheckOptions  const &,//Options,
                 std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;

    // Check to see if all its edges point to it.

    std::set<edge *>::const_iterator EdgeIter=m_sEdges.begin();
    while(EdgeIter!=m_sEdges.end())
        {
        edge *pEdge=*EdgeIter;
        std::set<face *> const &sFaces=pEdge->GetFaces();
        if(sFaces.find((face *)this)==sFaces.end())
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Edge %ld of Face %ld does not point to its face.\n",pEdge->GetID(),this->GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        ++EdgeIter;
        }

    // Check the loops

    std::vector<std::vector<edge *> > aaLoops;
    std::vector<std::vector<SGM::EdgeSideType> > aaFlipped;
    size_t nLoops=FindLoops(rResult,aaLoops,aaFlipped);
    size_t Index1,Index2;
    for(Index1=0;Index1<nLoops;++Index1)
        {
        std::vector<edge *> const &aLoop=aaLoops[Index1];
        std::vector<SGM::EdgeSideType> const &aFlipped=aaFlipped[Index1];
        size_t nEdges=aLoop.size();
        std::vector<vertex *> aStarts,aEnds;
        aStarts.reserve(nEdges);
        aEnds.reserve(nEdges);
        for(Index2=0;Index2<nEdges;++Index2)
            {
            edge *pEdge=aLoop[Index2];
            aStarts.push_back(aFlipped[Index2] ? pEdge->GetEnd() : pEdge->GetStart());
            aEnds.push_back(aFlipped[Index2] ? pEdge->GetStart() : pEdge->GetEnd());
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
                snprintf(Buffer,sizeof(Buffer),"Edge %ld of Face %ld does not match its neighbors.\n",pEdge->GetID(),this->GetID());
                aCheckStrings.emplace_back(Buffer);
                }
            }
        }

    return bAnswer;
    }

bool edge::Check(SGM::Result              &,//rResult,
                 SGM::CheckOptions  const &,//Options,
                 std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;

    // Check to see if the start and end vertices point to this edge.

    if(m_pStart)
        {
        std::set<edge *> const &sEdges=m_pStart->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sEdges.begin();
        bool bFound=false;
        while(EdgeIter!=sEdges.end())
            {
            if(*EdgeIter==this)
                {
                bFound=true;
                break;
                }
            ++EdgeIter;
            }
        if(bFound==false)
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"The start Vertex %ld of Edge %ld does not point back to the edge\n",m_pStart->GetID(),GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }
    else
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Edge %ld does not have a start vertex\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }
    if(m_pEnd)
        {
        std::set<edge *> const &sEdges=m_pEnd->GetEdges();
        std::set<edge *>::const_iterator EdgeIter=sEdges.begin();
        bool bFound=false;
        while(EdgeIter!=sEdges.end())
            {
            if(*EdgeIter==this)
                {
                bFound=true;
                }
            ++EdgeIter;
            }
        if(bFound==false)
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"The end Vertex %ld of Edge %ld does not point back to the edge\n",m_pEnd->GetID(),GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }
    else
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Edge %ld does not have an end vertex\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    // Check to see if it is consistently oriented with respect to its faces.

    if(m_sFaces.size()==2)
        {
        std::set<face *>::const_iterator FaceIter=m_sFaces.begin();
        face *pFace1=*FaceIter;
        ++FaceIter;
        face *pFace2=*FaceIter;
        SGM::EdgeSideType bFlipped1=pFace1->GetEdgeType(this);
        SGM::EdgeSideType bFlipped2=pFace2->GetEdgeType(this);
        if(bFlipped1==bFlipped2)
            {
            bAnswer=false;
            char Buffer[1000];
            snprintf(Buffer,sizeof(Buffer),"Edge %ld has the same orientation of both faces.\n",GetID());
            aCheckStrings.emplace_back(Buffer);
            }
        }

    return bAnswer;
    }

bool vertex::Check(SGM::Result              &,//rResult,
                   SGM::CheckOptions  const &,//Options,
                   std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;

    if(m_sEdges.empty())
        {
        bAnswer=false;
        char Buffer[1000];
        snprintf(Buffer,sizeof(Buffer),"Vertex %ld does not point to an edge\n",GetID());
        aCheckStrings.emplace_back(Buffer);
        }

    return bAnswer;
    }

bool curve::Check(SGM::Result              &,//rResult,
                  SGM::CheckOptions  const &,//Options,
                  std::vector<std::string> &) const //aCheckStrings) const
    {
    bool bAnswer=true;

    return bAnswer;
    }

bool surface::Check(SGM::Result              &,//rResult,
                    SGM::CheckOptions  const &,//Options,
                    std::vector<std::string> &) const//aCheckStrings) const
    {
    bool bAnswer=true;

    return bAnswer;
    }

bool entity::Check(SGM::Result              &rResult, 
                   SGM::CheckOptions  const &Options,
                   std::vector<std::string> &aCheckStrings) const
    {
    bool bAnswer=true;
    switch(m_Type)
        {
        case SGM::ThingType:
            {
            bAnswer=((thing const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::ComplexType:
            {
            bAnswer=((complex const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::BodyType:
            {
            bAnswer=((body const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::VolumeType:
            {
            bAnswer=((volume const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            };
        case SGM::FaceType:
            {
            bAnswer=((face const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            };
        case SGM::EdgeType:
            {
            bAnswer=((edge const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::VertexType:
            {
            bAnswer=((vertex const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::CurveType:
            {
            bAnswer=((curve const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        case SGM::SurfaceType:
            {
            bAnswer=((surface const *)this)->Check(rResult,Options,aCheckStrings);
            break;
            }
        default:
            {
            aCheckStrings.emplace_back("Given unknown entity type\n");
            bAnswer=false;
            break;
            }
        }
    return bAnswer;
    }

}}
