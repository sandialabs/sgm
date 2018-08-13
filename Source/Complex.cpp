#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
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