#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "SGMBoxTree.h"

#include "EntityClasses.h"

namespace SGMInternal
{

complex::complex(SGM::Result &rResult) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(),
        m_aSegments(),
        m_aTriangles()
    {}

complex::complex(SGM::Result &rResult,
                 std::vector<SGM::Point3D> const &aPoints) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles()
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<unsigned int> const &aSegments,
                 std::vector<SGM::Point3D> const &aPoints) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles()
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<unsigned int> const &aTriangles) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(),
        m_aTriangles(aTriangles)
    {}

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<unsigned int> const &aSegments,
                 std::vector<unsigned int> const &aTriangles) :
        topology(rResult, SGM::EntityType::ComplexType),
        m_aPoints(aPoints),
        m_aSegments(aSegments),
        m_aTriangles(aTriangles)
    {}

SGM::Interval3D const &complex::GetBox(SGM::Result &rResult) const
    {
    if (m_Box.IsEmpty())
        m_Box=SGM::Interval3D(GetPoints());
    return m_Box;
    }

void complex::Transform(SGM::Transform3D const &Trans)
    {
    size_t nPoints=m_aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        m_aPoints[Index1]*=Trans;
        }
    }

complex *complex::Cover(SGM::Result &) const
    {
    return nullptr;
    }

std::vector<complex *> complex::FindBoundary(SGM::Result &) const
    {
    std::vector<complex *> aAnswer;
    return aAnswer;
    }

complex *complex::Merge(SGM::Result &rResult) const
    {
    // Find duplicate points.

    SGM::BoxTree BTree;
    size_t Index1;
    size_t nPoints=m_aPoints.size();
    double dTolerance=SGM_MIN_TOL;
    std::map<size_t,size_t> mMergeMap;
    SGM::Point3D const *pBase=&m_aPoints[0];
    std::vector<SGM::Point3D> aNewPoints;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=m_aPoints[Index1];
        SGM::Interval3D Bound(Pos,dTolerance);
        std::vector<SGM::BoxTree::BoundedItemType> aHits=BTree.FindIntersectsPoint(Pos,dTolerance);
        if(aHits.empty())
            {
            BTree.Insert(&m_aPoints[Index1],Bound);
            mMergeMap[Index1]=aNewPoints.size();
            aNewPoints.push_back(Pos);
            }
        else
            {
            mMergeMap[Index1]=mMergeMap[(SGM::Point3D const *)aHits[0].first-pBase];
            }
        }

    // Remap points to their first version.

    size_t nSegments=m_aSegments.size();
    std::vector<unsigned int> aNewSegments;
    aNewSegments.reserve(nSegments);
    for(Index1=0;Index1<nSegments;++Index1)
        {
        aNewSegments.push_back((unsigned int)mMergeMap[m_aSegments[Index1]]);
        }

    size_t nTriangles=m_aTriangles.size();
    std::vector<unsigned int> aNewTriangles;
    aNewTriangles.reserve(nTriangles);
    for(Index1=0;Index1<nTriangles;++Index1)
        {
        aNewTriangles.push_back((unsigned int)mMergeMap[m_aTriangles[Index1]]);
        }

    return new complex(rResult,aNewPoints,aNewSegments,aNewTriangles);
    }

double complex::Area() const
    {
    size_t Index1;
    size_t nTriangles=m_aTriangles.size();
    double dArea=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        SGM::Point3D const &A=m_aPoints[Index1];
        SGM::Point3D const &B=m_aPoints[Index1+1];
        SGM::Point3D const &C=m_aPoints[Index1+2];
        dArea+=((A-B)*(C-B)).Magnitude();
        }
    return dArea*0.5;
    }
}