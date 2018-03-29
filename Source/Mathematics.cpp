#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "Faceter.h"
#include "math.h"
#include <vector>
#include <set>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////
//
//  Point vector functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Interval3D SGM::FindBoundingBox3D(std::vector<SGM::Point3D> const &aPoints)
    {
    SGM::Interval3D Answer;
    Answer.m_XDomain.m_dMin=DBL_MAX;
    Answer.m_XDomain.m_dMax=-DBL_MAX;
    Answer.m_YDomain.m_dMin=DBL_MAX;
    Answer.m_YDomain.m_dMax=-DBL_MAX;
    Answer.m_ZDomain.m_dMin=DBL_MAX;
    Answer.m_ZDomain.m_dMax=-DBL_MAX;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        if(Pos.m_x<Answer.m_XDomain.m_dMin)
            {
            Answer.m_XDomain.m_dMin=Pos.m_x;
            }
        if(Pos.m_y<Answer.m_YDomain.m_dMin)
            {
            Answer.m_YDomain.m_dMin=Pos.m_y;
            }
        if(Pos.m_z<Answer.m_ZDomain.m_dMin)
            {
            Answer.m_ZDomain.m_dMin=Pos.m_z;
            }
        if(Answer.m_XDomain.m_dMax<Pos.m_x)
            {
            Answer.m_XDomain.m_dMax=Pos.m_x;
            }
        if(Answer.m_YDomain.m_dMax<Pos.m_y)
            {
            Answer.m_YDomain.m_dMax=Pos.m_y;
            }
        if(Answer.m_ZDomain.m_dMax<Pos.m_z)
            {
            Answer.m_ZDomain.m_dMax=Pos.m_z;
            }
        }
    return Answer;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Polygon Functions
//
///////////////////////////////////////////////////////////////////////////////

double SGM::PolygonArea(std::vector<SGM::Point2D> const &aPolygon)
    {
    double dArea=0;
    size_t nPoints=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[Index1];
        SGM::Point2D const &Pos1=aPolygon[(Index1+1)%nPoints];
        dArea+=Pos0.m_u*Pos1.m_v-Pos1.m_u*Pos0.m_v;
        }
    return dArea*0.5;
    }

size_t SGM::FindConcavePoints(std::vector<SGM::Point2D> const &aPolygon,
                              std::vector<size_t>             &aConcavePoints)
    {
    double dArea=SGM::PolygonArea(aPolygon);
    size_t nPoints=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[(Index1+nPoints-1)%nPoints];
        SGM::Point2D const &Pos1=aPolygon[Index1];
        SGM::Point2D const &Pos2=aPolygon[(Index1+1)%nPoints];
        double dVec0u=Pos2.m_u-Pos1.m_u;
        double dVec0v=Pos2.m_v-Pos1.m_v;
        double dVec1u=Pos0.m_u-Pos1.m_u;
        double dVec1v=Pos0.m_v-Pos1.m_v;
        double dAngle=dVec0u*dVec1v-dVec1u*dVec0v;
        if(0<dAngle*dArea)
            {
            aConcavePoints.push_back(Index1);
            }
        }
    return aConcavePoints.size();
    }

bool SGM::PointInPolygon(SGM::Point2D              const &Pos,
                         std::vector<SGM::Point2D> const &aPolygon)
    {
    size_t nPoints=aPolygon.size();
    size_t Index1,Index2;
    size_t nCrosses=0;
    double u=Pos.m_u;
    double v=Pos.m_v;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point2D const &Pos0=aPolygon[Index1];
        SGM::Point2D const &Pos1=aPolygon[(Index1+1)%nPoints];
        if(v<Pos0.m_v)
            {
            if(Pos1.m_v<v)
                {
                double CrossU=Pos0.m_u+((Pos0.m_v-v)/(v-Pos1.m_v))*(Pos1.m_u-Pos0.m_u);
                if(u<CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if(Pos1.m_v==v && u<Pos1.m_u)
                {
                // If the first point past Index1 is below v then it crossed.
                for(Index2=1;Index2<nPoints;++Index2)
                    {
                    SGM::Point2D const &Pos2=aPolygon[(Index1+Index2)%nPoints];
                    if(Pos2.m_v<v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if(v<Pos2.m_v)
                        {
                        break;
                        }
                    else if(Pos2.m_u<=u)
                        {
                        break;
                        }
                    }
                }
            }
        if(Pos0.m_v<v)
            {
            if(v<Pos1.m_v)
                {
                double CrossU=Pos0.m_u+((Pos0.m_v-v)/(v-Pos1.m_v))*(Pos1.m_u-Pos0.m_u);
                if(u<CrossU)
                    {
                    ++nCrosses;
                    }
                }
            else if(Pos1.m_v==v && u<Pos1.m_u)
                {
                // If the first point past Index1 is above v then it crossed.
                for(Index2=1;Index2<nPoints;++Index2)
                    {
                    SGM::Point2D const &Pos2=aPolygon[(Index1+Index2)%nPoints];
                    if(v<Pos2.m_v)
                        {
                        ++nCrosses;
                        break;
                        }
                    else if(Pos2.m_v<v)
                        {
                        break;
                        }
                    else if(Pos2.m_u<=u)
                        {
                        break;
                        }
                    }
                }
            }
        }
    return nCrosses%2==1;
    }

double SGM::Determinate(double dA,double dB,double dC,
                        double dD,double dE,double dF,
                        double dG,double dH,double dI)
    {
    return dA*(dE*dI-dF*dH)+dB*(dF*dG-dD*dI)+dC*(dD*dH-dE*dG);
    }

double sign(SGM::Point2D const &P1,
            SGM::Point2D const &P2,
            SGM::Point2D const &P3)
    {
    return (P1.m_u-P3.m_u)*(P2.m_v-P3.m_v)-(P2.m_u-P3.m_u)*(P1.m_v-P3.m_v);
    }

bool SGM::InTriangle(SGM::Point2D const &A,
                     SGM::Point2D const &B,
                     SGM::Point2D const &C,
                     SGM::Point2D const &D)
    {
    bool b1=sign(D,A,B)<0;
    bool b2=sign(D,B,C)<0;
    bool b3=sign(D,C,A)<0;
    return ((b1==b2) && (b2==b3));
    }

bool SGM::InCircumcircle(SGM::Point2D const &A,
                         SGM::Point2D const &B,
                         SGM::Point2D const &C,
                         SGM::Point2D const &D)
    {
    double dA=A.m_u-D.m_u;
    double dB=A.m_v-D.m_v;
    double dC=dA*dA+dB*dB;
    double dD=B.m_u-D.m_u;
    double dE=B.m_v-D.m_v;
    double dF=dD*dD+dE*dE;
    double dG=C.m_u-D.m_u;
    double dH=C.m_v-D.m_v;
    double dI=dG*dG+dH*dH;
    double dDet=SGM::Determinate(dA,dB,dC,dD,dE,dF,dG,dH,dI);
    return 1E-12<dDet;
    }

class EdgeData
    {
    public:

        EdgeData(size_t nPosA,size_t nPosB,size_t nTriangle,int nEdge):
            m_nPosA(nPosA),m_nPosB(nPosB),m_nTriangle(nTriangle),m_nEdge(nEdge) 
            {
            if(nPosB<nPosA)
                {
                std::swap(m_nPosA,m_nPosB);
                }
            }

        bool operator<(EdgeData const &ED) const
            {
            if(m_nPosA<ED.m_nPosA)
                {
                return true;
                }
            else if(m_nPosA==ED.m_nPosA)
                {
                if(m_nPosB<ED.m_nPosB)
                    {
                    return true;
                    }
                }
            return false;
            }

        size_t m_nPosA;
        size_t m_nPosB;
        size_t m_nTriangle;
        int    m_nEdge;
    };

size_t SGM::FindAdjacences2D(std::vector<size_t> const &aTriangles,
                             std::vector<size_t>       &aAdjacency)
    {
    std::vector<EdgeData> aEdges;
    size_t nTriangles=aTriangles.size();
    aAdjacency.assign(nTriangles,SIZE_MAX);
    aEdges.reserve(nTriangles);
    size_t Index1,Index2;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        size_t a=aTriangles[Index1];
        size_t b=aTriangles[Index1+1];
        size_t c=aTriangles[Index1+2];
        aEdges.push_back(EdgeData(a,b,Index1,0));
        aEdges.push_back(EdgeData(b,c,Index1,1));
        aEdges.push_back(EdgeData(c,a,Index1,2));
        }
    std::sort(aEdges.begin(),aEdges.end());
    size_t nEdges=aEdges.size();
    Index1=0;
    while(Index1<nEdges)
        {
        size_t nStart=Index1;
        size_t nPosA=aEdges[Index1].m_nPosA;
        size_t nPosB=aEdges[Index1].m_nPosB;
        ++Index1;
        while( Index1<nEdges && 
               aEdges[Index1].m_nPosA==nPosA &&
               aEdges[Index1].m_nPosB==nPosB)
            {
            ++Index1;
            }
        for(Index2=nStart;Index2<Index1;++Index2)
            {
            EdgeData const &ED1=aEdges[Index2];
            EdgeData const &ED2=aEdges[Index2+1<Index1 ? Index2+1 : nStart];
            if(ED1.m_nTriangle!=ED2.m_nTriangle)
                {
                aAdjacency[ED1.m_nTriangle+ED1.m_nEdge]=ED2.m_nTriangle;
                }
            }
        }
    return nTriangles;
    }

bool SGM::CramersRule(double a1,double b1,double c1,
                      double a2,double b2,double c2,
                      double &x,double &y)
    {
    double D=a1*b2-b1*a2;
    double Dx=c1*b2-b1*c2;
    double Dy=a1*c2-c1*a2;
    if(fabs(D)<1E-12)
        {
        return false;
        }
    x=Dx/D;
    y=Dy/D;
    return true;
    }

// Returns true if the given segment intersects the given polygon
// at a segment other than a segment contains the point with 
// index nNotHere.

bool SegmentIntersectPolygon(std::vector<SGM::Point2D> const &aPoints,
                             SGM::Segment2D            const &Segment,
                             std::vector<size_t>       const &aPolygon,
                             size_t                           nNotHere)
    {
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        if(Index1!=nNotHere && (Index1+1)%nPolygon!=nNotHere)
            {
            SGM::Segment2D TestSeg(aPoints[aPolygon[Index1]],
                                aPoints[aPolygon[(Index1+1)%nPolygon]]);
            SGM::Point2D Pos;
            if(Segment.Intersect(TestSeg,Pos))
                {
                return true;
                }
            }
        }
    return false;
    }

void BridgePolygon(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<size_t>       const &aInsidePolygon,
                   size_t                           nExtreamPoint,
                   std::vector<size_t>             &aOutsidePolygon)
    {
    size_t Index1,Index2;
    size_t nInside=aInsidePolygon.size();
    size_t nOutside=aOutsidePolygon.size();
    std::vector<std::pair<double,SGM::Segment2D> > aSegments;
    SGM::Point2D const &Pos1=aPoints[aInsidePolygon[nExtreamPoint]];
    std::vector<std::pair<double,size_t> > aSpans;
    aSpans.reserve(nOutside);
    for(Index1=0;Index1<nOutside;++Index1)
        {
        SGM::Point2D const &Pos2=aPoints[aOutsidePolygon[Index1]];
        double dLengthSquared(Pos1.DistanceSquared(Pos2));
        aSpans.push_back(std::pair<double,size_t>(dLengthSquared,Index1));
        }
    std::sort(aSpans.begin(),aSpans.end());
    for(Index1=0;Index1<nOutside;++Index1)
        {
        size_t nSpanHit=aSpans[Index1].second;
        SGM::Segment2D TestSeg(Pos1,aPoints[aOutsidePolygon[nSpanHit]]);
        if( SegmentIntersectPolygon(aPoints,TestSeg,aInsidePolygon,nExtreamPoint)==false &&
            SegmentIntersectPolygon(aPoints,TestSeg,aOutsidePolygon,nSpanHit)==false)
            {
            // Connect nExtreamPoint on the inner polygon to 
            // nSpanHit on the outer polygon.

            std::vector<size_t> aNewPoly;
            aNewPoly.reserve(nInside+nOutside+2);
            for(Index2=0;Index2<=nSpanHit;++Index2)
                {
                aNewPoly.push_back(aOutsidePolygon[Index2]);
                }
            for(Index2=0;Index2<=nInside;++Index2)
                {
                aNewPoly.push_back(aInsidePolygon[(nExtreamPoint+Index2)%nInside]);
                }
            for(Index2=nSpanHit;Index2<nOutside;++Index2)
                {
                aNewPoly.push_back(aOutsidePolygon[Index2]);
                }
            aOutsidePolygon=aNewPoly;
            break;
            }
        }
    }

size_t GetPrevious(size_t                   nEar,
                   std::vector<bool> const &aCutOff)
    {
    size_t nAnswer=nEar;
    size_t nPolygon=aCutOff.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+nPolygon-Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

size_t GetNext(size_t                   nEar,
               std::vector<bool> const &aCutOff)
    {
    size_t nAnswer=nEar;
    size_t nPolygon=aCutOff.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        nAnswer=(nEar+Index1)%nPolygon;
        if(aCutOff[nAnswer]==false)
            {
            break;
            }
        }
    return nAnswer;
    }

bool GoodEar(std::vector<SGM::Point2D> const &aPoints,
             std::vector<size_t>             &aPolygon,
             std::vector<bool>         const &aCutOff,
             size_t                           nEar)
    {
    size_t nPolygon=aPolygon.size();
    size_t nEarA=aPolygon[GetPrevious(nEar,aCutOff)];
    size_t nEarB=aPolygon[nEar];
    size_t nEarC=aPolygon[GetNext(nEar,aCutOff)];
    SGM::Point2D const &A=aPoints[nEarA];
    SGM::Point2D const &B=aPoints[nEarB];
    SGM::Point2D const &C=aPoints[nEarC];
    size_t Index1;
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        size_t nPos=aPolygon[Index1];
        if(nPos!=nEarA && nPos!=nEarB && nPos!=nEarC)
            {
            SGM::Point2D const &Pos=aPoints[nPos];
            if(InTriangle(A,B,C,Pos))
                {
                return false;
                }
            }
        }
    return true;
    }

class PolyData
    {
    public:

        double dExtreamU;
        size_t nWhichPoly;
        size_t nWhichPoint;

        bool operator<(PolyData const &PD) const
            {
            return PD.dExtreamU<dExtreamU;
            }
    };

double FindAngle(std::vector<SGM::Point2D> const &aPoints,
                 std::vector<size_t>       const &aPolygon,
                 std::vector<bool>         const &aCutOff,
                 size_t                           nB)
    {
    size_t nA=0,nC=0;
    size_t nPolygon=aPolygon.size();
    size_t Index1;
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        size_t nWhere=(nB+Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nC=nWhere;
            break;
            }
        }
    for(Index1=1;Index1<nPolygon;++Index1)
        {
        size_t nWhere=(nB+nPolygon-Index1)%nPolygon;
        if(aCutOff[nWhere]==false)
            {
            nA=nWhere;
            break;
            }
        }
    SGM::Point2D const &PosA=aPoints[aPolygon[nA]];
    SGM::Point2D const &PosB=aPoints[aPolygon[nB]];
    SGM::Point2D const &PosC=aPoints[aPolygon[nC]];
    SGM::UnitVector2D VecAB=PosA-PosB;
    SGM::UnitVector2D VecCB=PosC-PosB;
    double dUp=VecAB.m_v*VecCB.m_u-VecAB.m_u*VecCB.m_v;
    if(dUp<0)
        {
        return 10;
        }
    else
        {
        return 1.0-VecAB%VecCB;
        }
    }

void FixBackPointers(size_t                     nTri,
                     std::vector<size_t> const &aTriangles,
                     std::vector<size_t>       &aAdjacences)
    {
    size_t nT0=aAdjacences[nTri];
    size_t nT1=aAdjacences[nTri+1];
    size_t nT2=aAdjacences[nTri+2];
    size_t a=aTriangles[nTri];
    size_t b=aTriangles[nTri+1];
    size_t c=aTriangles[nTri+2];
    
    if(nT0!=SIZE_MAX)
        {
        size_t a0=aTriangles[nT0];
        size_t b0=aTriangles[nT0+1];
        if(a0!=a && a0!=b)
            {
            aAdjacences[nT0+1]=nTri;
            }
        else if(b0!=a && b0!=b)
            {
            aAdjacences[nT0+2]=nTri;
            }
        else
            {
            aAdjacences[nT0]=nTri;
            }
        }

    if(nT1!=SIZE_MAX)
        {
        size_t a1=aTriangles[nT1];
        size_t b1=aTriangles[nT1+1];
        if(a1!=c && a1!=b)
            {
            aAdjacences[nT1+1]=nTri;
            }
        else if(b1!=c && b1!=b)
            {
            aAdjacences[nT1+2]=nTri;
            }
        else
            {
            aAdjacences[nT1]=nTri;
            }
        }

    if(nT2!=SIZE_MAX)
        {
        size_t a2=aTriangles[nT2];
        size_t b2=aTriangles[nT2+1];
        if(a2!=a && a2!=c)
            {
            aAdjacences[nT2+1]=nTri;
            }
        else if(b2!=a && b2!=c)
            {
            aAdjacences[nT2+2]=nTri;
            }
        else
            {
            aAdjacences[nT2]=nTri;
            }
        }
    }

bool FlipTriangles(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<size_t>             &aTriangles,
                   std::vector<size_t>             &aAdjacences,
                   size_t                           nTri,
                   size_t                           nEdge)
    {
    size_t a=aTriangles[nTri];
    size_t b=aTriangles[nTri+1];
    size_t c=aTriangles[nTri+2];
    size_t nT=aAdjacences[nTri+nEdge];
    if(nT==SIZE_MAX)
        {
        return false;
        }
    size_t d=aTriangles[nT];
    size_t e=aTriangles[nT+1];
    size_t f=aTriangles[nT+2];
    size_t g,nTA,nTB;
    if(d!=a && d!=b && d!=c)
        {
        g=d;
        nTA=aAdjacences[nT+2];
        nTB=aAdjacences[nT];
        }
    else if(e!=a && e!=b && e!=c)
        {
        g=e;
        nTA=aAdjacences[nT];
        nTB=aAdjacences[nT+1];
        }
    else
        {
        g=f;
        nTA=aAdjacences[nT+1];
        nTB=aAdjacences[nT+2];
        }
    SGM::Point2D const &A=aPoints[a];
    SGM::Point2D const &B=aPoints[b];
    SGM::Point2D const &C=aPoints[c];
    SGM::Point2D const &G=aPoints[g];
    if(SGM::InCircumcircle(A,B,C,G))
        {
        size_t nT0=aAdjacences[nTri];
        size_t nT1=aAdjacences[nTri+1];
        size_t nT2=aAdjacences[nTri+2];
        if(nEdge==0)
            {
            aTriangles[nTri]=g;
            aTriangles[nTri+1]=c;
            aTriangles[nTri+2]=a;
            aTriangles[nT]=g;
            aTriangles[nT+1]=b;
            aTriangles[nT+2]=c;

            aAdjacences[nTri]=nT;
            aAdjacences[nTri+1]=nT2;
            aAdjacences[nTri+2]=nTA;
            aAdjacences[nT]=nTB;
            aAdjacences[nT+1]=nT1;
            aAdjacences[nT+2]=nTri;
            }
        else if(nEdge==1)
            {
            aTriangles[nTri]=g;
            aTriangles[nTri+1]=a;
            aTriangles[nTri+2]=b;
            aTriangles[nT]=g;
            aTriangles[nT+1]=c;
            aTriangles[nT+2]=a;

            aAdjacences[nTri]=nT;
            aAdjacences[nTri+1]=nT0;
            aAdjacences[nTri+2]=nTA;
            aAdjacences[nT]=nTB;
            aAdjacences[nT+1]=nT2;
            aAdjacences[nT+2]=nTri;
            }
        else
            {
            aTriangles[nTri]=g;
            aTriangles[nTri+1]=a;
            aTriangles[nTri+2]=b;
            aTriangles[nT]=g;
            aTriangles[nT+1]=b;
            aTriangles[nT+2]=c;

            aAdjacences[nTri]=nTB;
            aAdjacences[nTri+1]=nT0;
            aAdjacences[nTri+2]=nT;
            aAdjacences[nT]=nTri;
            aAdjacences[nT+1]=nT1;
            aAdjacences[nT+2]=nTA;
            }
        FixBackPointers(nT,aTriangles,aAdjacences);
        FixBackPointers(nTri,aTriangles,aAdjacences);
        return true;
        }
    return false;
    }

void DelaunayFlips(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<size_t>             &aTriangles,
                   std::vector<size_t>             &aAdjacences)
    {
    size_t nTriangles=aTriangles.size();
    bool bFlipped=true;
    size_t Index1;
    while(bFlipped)
        {
        bFlipped=false;
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            if(FlipTriangles(aPoints,aTriangles,aAdjacences,Index1,0))
                {
                bFlipped=true;
                }
            if(FlipTriangles(aPoints,aTriangles,aAdjacences,Index1,1))
                {
                bFlipped=true;
                }
            if(FlipTriangles(aPoints,aTriangles,aAdjacences,Index1,2))
                {
                bFlipped=true;
                }
            }
        }
    }

void SGM::TriangulatePolygon(SGM::Result                             &rResult,
                             std::vector<SGM::Point2D>         const &aPoints,
                             std::vector<std::vector<size_t> > const &aaPolygons,
                             std::vector<size_t>                     &aTriangles,
                             std::vector<size_t>                     &aAdjacences)
    {
    if(aaPolygons.empty() || aPoints.empty())
        {
        rResult.SetResult(ResultTypeInsufficientData);
        return;
        }

    // First create one polygon.

    std::vector<size_t> aPolygon=aaPolygons[0];
    size_t nPolygons=aaPolygons.size();
    size_t Index1,Index2;
    if(1<nPolygons)
        {
        // Sort the inside polygons by extream u value.

        std::vector<PolyData> aUValues;
        for(Index1=1;Index1<nPolygons;++Index1)
            {
            double dUValue=-DBL_MAX;
            std::vector<size_t> const &aInsidePoly=aaPolygons[Index1];
            size_t nInsidePoly=aInsidePoly.size();
            size_t nWhere=0;
            for(Index2=0;Index2<nInsidePoly;++Index2)
                {
                SGM::Point2D const &Pos=aPoints[aInsidePoly[Index2]];
                if(dUValue<Pos.m_u)
                    {
                    dUValue=Pos.m_u;
                    nWhere=Index2;
                    }
                }
            PolyData PD;
            PD.dExtreamU=dUValue;
            PD.nWhichPoint=nWhere;
            PD.nWhichPoly=Index1;
            aUValues.push_back(PD);
            }
        std::sort(aUValues.begin(),aUValues.end());

        for(Index1=0;Index1<nPolygons-1;++Index1)
            {
            BridgePolygon(aPoints,
                          aaPolygons[aUValues[Index1].nWhichPoly],
                          aUValues[Index1].nWhichPoint,
                          aPolygon);
            }
        }

    // Find and cut off ears with the smallest angle first.
    // First find the angle of each vertex of the polygon.
    // Then cut on the nPolygon-3 ears.

    size_t nPolygon=aPolygon.size();
    std::vector<bool> aCutOff;
    aCutOff.assign(nPolygon,false);
    aTriangles.reserve(3*(nPolygon-2));
    std::set<std::pair<double,size_t> > sAngles;
    std::vector<double> aAngles;
    aAngles.reserve(nPolygon);
    for(Index1=0;Index1<nPolygon;++Index1)
        {
        SGM::Point2D const &PosA=aPoints[aPolygon[(Index1+nPolygon-1)%nPolygon]];
        SGM::Point2D const &PosB=aPoints[aPolygon[Index1]];
        SGM::Point2D const &PosC=aPoints[aPolygon[(Index1+1)%nPolygon]];
        SGM::UnitVector2D VecAB=PosA-PosB;
        SGM::UnitVector2D VecCB=PosC-PosB;
        double dUp=VecAB.m_v*VecCB.m_u-VecAB.m_u*VecCB.m_v;
        if(dUp<0)
            {
            sAngles.insert(std::pair<double,size_t>(10.0,Index1));
            aAngles.push_back(10.0);
            }
        else
            {
            double dAngle=1.0-VecAB%VecCB;
            sAngles.insert(std::pair<double,size_t>(dAngle,Index1));
            aAngles.push_back(dAngle);
            }
        }
    for(Index1=0;Index1<nPolygon-2;++Index1)
        {
        std::set<std::pair<double,size_t> >::iterator iter=sAngles.begin();
        while(iter!=sAngles.end())
            {
            std::pair<double,size_t> Angle=*iter;
            size_t nEar=Angle.second;
            if(GoodEar(aPoints,aPolygon,aCutOff,nEar))
                {
                size_t nEarA=GetPrevious(nEar,aCutOff);
                size_t nA=aPolygon[nEarA];
                size_t nB=aPolygon[nEar];
                size_t nEarC=GetNext(nEar,aCutOff);
                size_t nC=aPolygon[nEarC];
                aTriangles.push_back(nA);
                aTriangles.push_back(nB);
                aTriangles.push_back(nC);
            
                // Fix angles at nEar

                sAngles.erase(Angle);
                Angle.first=10;
                sAngles.insert(Angle);
                aAngles[nEar]=10;
                aCutOff[nEar]=true;

                // Fix angles at nEarA

                std::pair<double,size_t> AngleA(aAngles[nEarA],nEarA);
                sAngles.erase(AngleA);
                AngleA.first=FindAngle(aPoints,aPolygon,aCutOff,nEarA);
                sAngles.insert(AngleA);
                aAngles[nEarA]=AngleA.first;

                // Fix angles at nEarC

                std::pair<double,size_t> AngleC(aAngles[nEarC],nEarC);
                sAngles.erase(AngleC);
                AngleC.first=FindAngle(aPoints,aPolygon,aCutOff,nEarC);
                sAngles.insert(AngleC);
                aAngles[nEarC]=AngleC.first;

                break;
                }
            ++iter;
            }
        }
    
    // Flip to improve triangles.

    SGM::FindAdjacences2D(aTriangles,aAdjacences);
    DelaunayFlips(aPoints,aTriangles,aAdjacences);
    }

size_t SGM::Linear(double a,double b,
                   std::vector<double> aRoots)
    {
    // a*x+b=0 -> ax=-b -> x=-b/a
    if(fabs(a)<SGM_ZERO)
        {
        if(fabs(b)<SGM_ZERO)
            {
            aRoots.push_back(0);
            return 1;
            }
        return 0;
        }
    aRoots.push_back(-b/a);
    return 1;
    }

size_t SGM::Quadratic(double a,double b,double c,
                      std::vector<double> &aRoots)
    {
    if(fabs(a)<SGM_ZERO)
        {
        return Linear(b,c,aRoots);
        }
    double r=b*b-4*a*c;
    if(fabs(r)<SGM_ZERO)
        {
        aRoots.push_back(-b/(2*a));
        }
    else if(0<r)
        {
        double Q=b<0 ? -sqrt(r) : sqrt(r);
        double q=-0.5*(b+Q);
        aRoots.push_back(q/a);
        if(SGM_ZERO<fabs(q))
            {
            aRoots.push_back(c/q);
            }
        }
    std::sort(aRoots.begin(),aRoots.end());
    return aRoots.size();
    }

double SGM::SAFEacos(double x)
    {
    if(1.0<x)
        {
        return 0;
        }
    else if(x<-1.0)
        {
        return SGM_PI;
        }
    else
        {
        return acos(x);
        }
    }

size_t SGM::Cubic(double c3,double c2,double c1,double c0,
                  std::vector<double> &aRoots)
    {
    if(fabs(c3)<SGM_ZERO)
        {
        return Quadratic(c2,c1,c0,aRoots);
        }
    else
        {
        double a=c2/c3;
        double b=c1/c3;
        double c=c0/c3;
        double a2=a*a;
        double Q=(a2-3*b)/9;
        double a3=a2*a;
        double R=(2*a3-9*a*b+27*c)/54;
        double R2=R*R;
        double Q3=Q*Q*Q;
        double G=a/3;
        if(R2<Q3)
            {
            double F=-2*sqrt(Q);
            double T=SGM::SAFEacos(R/sqrt(Q3));
            aRoots.push_back(F*cos(T/3)-G);
            aRoots.push_back(F*cos((T+SGM_TWO_PI)/3)-G);
            aRoots.push_back(F*cos((T-SGM_TWO_PI)/3)-G);
            }
        else
            {
            double E=a/3.0;
            double A=pow(abs(R)+sqrt(R2-Q3),1.0/3.0);
            if(fabs(A)<SGM_ZERO)
                {
                aRoots.push_back(-E);
                }
            else
                {
                if(0<R)
                    {
                    A=-A;
                    }
                double B=fabs(A)<SGM_ZERO ? 0 : Q/A;
                aRoots.push_back(A+B-E);
                if(fabs(A-B)<SGM_ZERO)
                    {
                    aRoots.push_back(-(0.5*(A+B)+E));
                    }
                }
            }
        }
    std::sort(aRoots.begin(),aRoots.end());
    return aRoots.size();
    }

size_t SGM::Quartic(double a,double b,double c,double d,double e,
                    std::vector<double> &aRoots)
    {
    if(fabs(a)<SGM_ZERO)
        {
        return SGM::Cubic(b,c,d,e,aRoots);
        }

    double a2=a*a;
    double b2=b*b;
    double c2=c*c;
    double d2=d*d;
    double a3=a2*a;
    double b3=b2*b;
    double c3=c2*c;
    
    double p=(8*a*c-3*b2)/(8*a2);
    double q=(b3-4*a*b*c+8*a2*d)/(8*a3);
    double D0=c2-3*b*d+12*a*e;
    double D1=2*c3-9*b*c*d+27*b2*e+27*a*d2-72*a*c*e;
    if(fabs(D0)<SGM_ZERO)
        {
        if(fabs(D1)<SGM_ZERO)
            {
            // Look for a triple root, which is a root of 12a*x^2+6*b*x+2*c
            std::vector<double> aQuadRoots;
            size_t nQuadRoots=Quadratic(12*a,6*b,2*c,aQuadRoots);
            size_t Index1;
            for(Index1=0;Index1<nQuadRoots;++Index1)
                {
                double u=aQuadRoots[Index1];
                double u2=u*u;
                double u3=u2*u;
                double u4=u2*u2;
                double f=a*u4+b*u3+c*u2+d*u+e;
                if(fabs(f)<SGM_ZERO)
                    {
                    // Then given a triple root u the other root is -3u-b/a
                    double v=-3*u-b/a;
                    if(fabs(u-v)<SGM_ZERO)
                        {
                        aRoots.push_back(u); // quadruple root.
                        }
                    else
                        {
                        aRoots.push_back(u);
                        aRoots.push_back(-3*u-b/a);
                        }
                    break;
                    }
                }
            }
        else
            {
            throw;
            }
        }
    else
        {
        double D03=D0*D0*D0;
        double r=D1*D1-4*D03;
        double Q=0;
        if(0<=r)
            {
            double cr=(D1+sqrt(r))/2.0;
            if(cr<0)
                {
                Q=pow(-cr,1.0/3.0);
                }
            else
                {
                Q=pow(cr,1.0/3.0);
                }
            }
        double S=0;
        if(Q<SGM_ZERO)
            {
            double T=SGM::SAFEacos(D1/(2*sqrt(D03)));
            S=0.5*sqrt((2*sqrt(D0)*cos(T/3)/a-2*p)/3);
            }
        else
            {
            S=0.5*sqrt(((Q+D0/Q)/a-2.0*p)/3.0);
            }
        if(fabs(S)<SGM_ZERO)
            {
            throw;
            }

        double S2=S*S;
        double QS=q/S;
        double E=p*2+S2*4;
        double R12=QS-E;
        double R34=-QS-E;
        double b4a=-b/(4*a);

        if(fabs(R12)<SGM_ZERO)
            {
            aRoots.push_back(b4a-S);
            }
        else if(0<R12)
            {
            double F=0.5*sqrt(R12);
            aRoots.push_back(b4a-S-F);
            aRoots.push_back(b4a-S+F);
            }

        if(fabs(R34)<SGM_ZERO)
            {
            aRoots.push_back(b4a+S);
            }
        else if(0<R34)
            {
            double F=0.5*sqrt(R34);
            aRoots.push_back(b4a+S-F);
            aRoots.push_back(b4a+S+F);
            }
        }
    std::sort(aRoots.begin(),aRoots.end());
        
    return aRoots.size();
    }