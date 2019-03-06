#ifndef SGM_INTERNAL_EDGEDATA_H
#define SGM_INTERNAL_EDGEDATA_H

#include <utility>

namespace SGMInternal
{

class EdgeData
    {
public:

    EdgeData() = default;

    EdgeData(EdgeData const &other) = default;

    EdgeData(unsigned nPosA,unsigned nPosB,unsigned nTriangle,unsigned nEdge):
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

    unsigned m_nPosA;
    unsigned m_nPosB;
    unsigned m_nTriangle;
    unsigned m_nEdge;
    };

} // namespace SGMInternal

#endif //SGM_INTERNAL_EDGEDATA_H
