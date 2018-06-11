#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"

namespace SGMInternal
{

    complex::complex(SGM::Result &rResult) :
            entity(rResult, SGM::EntityType::ComplexType),
            m_aPoints(),
            m_aSegments(),
            m_aTriangles(),
            m_pThing(rResult.GetThing()),
            m_Box()
    {}

    complex::complex(SGM::Result &rResult,
                     std::vector<SGM::Point3D> const &aPoints) :
            entity(rResult, SGM::EntityType::ComplexType),
            m_aPoints(aPoints),
            m_aSegments(),
            m_aTriangles(),
            m_pThing(rResult.GetThing()),
            m_Box(m_aPoints)
    {}

    complex::complex(SGM::Result &rResult,
                     std::vector<size_t> const &aSegments,
                     std::vector<SGM::Point3D> const &aPoints) :
            entity(rResult, SGM::EntityType::ComplexType),
            m_aPoints(aPoints),
            m_aSegments(aSegments),
            m_aTriangles(),
            m_pThing(rResult.GetThing()),
            m_Box(m_aPoints)
    {}

    complex::complex(SGM::Result &rResult,
                     std::vector<SGM::Point3D> const &aPoints,
                     std::vector<size_t> const &aTriangles) :
            entity(rResult, SGM::EntityType::ComplexType),
            m_aPoints(aPoints),
            m_aSegments(),
            m_aTriangles(aTriangles),
            m_pThing(rResult.GetThing()),
            m_Box(m_aPoints)
    {}

SGM::Interval3D const &complex::GetBox() const
    {
    if(m_Box.IsEmpty())
        {
        m_Box = SGM::Interval3D(m_aPoints);
        }
    return m_Box;
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