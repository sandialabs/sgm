#ifndef SGM_INTERNAL_VERTEXDATA_H
#define SGM_INTERNAL_VERTEXDATA_H

namespace SGMInternal
{

class VertexData
    {
public:

    VertexData(unsigned nPos,unsigned nSegment,unsigned nVertex):
        m_nPos(nPos),m_nSegment(nSegment),m_nVertex(nVertex)
        {
        }

    bool operator<(VertexData const &VD) const
        {
        return m_nPos<VD.m_nPos;
        }

    unsigned m_nPos;
    unsigned m_nSegment;
    unsigned m_nVertex;
    };

} // namespace SGMInternal

#endif //SGM_INTERNAL_VERTEXDATA_H
