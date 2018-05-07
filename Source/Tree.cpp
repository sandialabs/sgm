#include "SGMTree.h"
#include <cfloat>

#define SGM_MAX_NODES 10
#define SGM_MIN_NODES  4
#define SGM_REINSERT   3

#if 0
SGM::BoxTree::BoxTree(size_t nReserve):m_nRoot(SIZE_MAX)
    {
    m_aData.reserve(nReserve);
    }

void SGM::BoxTree::AddBox(SGM::Interval3D const &Box,
                          void            const *pData)
    {
    // First get or add memory for a node on the tree.

    size_t nWhere;
    if(m_aFree.empty())
        {
        SGM::TreeNode Node;
        Node.m_Box=Box;
        Node.m_pData=pData;
        nWhere=m_aData.size();
        m_aData.push_back(Node);
        AddNode(nWhere);
        }
    else
        {
        nWhere=m_aFree.back();
        m_aFree.pop_back();
        SGM::TreeNode &Node=m_aData[nWhere];
        Node.m_Box=Box;
        Node.m_pData=pData;
        AddNode(nWhere);
        }
    }

void SGM::BoxTree::AddNode(size_t nID)
    {
    SGM::TreeNode &Node=m_aData[nID];
    size_t nWhere=m_nRoot;
    if(nWhere==SIZE_MAX)
        {
        m_nRoot=nID;
        Node.m_nParent=SIZE_MAX;
        }
    else
        {
        SGM::TreeNode &Parent=m_aData[nWhere];
        std::set<size_t> &sChildren=Parent.m_sChildren;
        while(sChildren.empty()==false)
            {
            SGM::Interval3D const &Box=Node.m_Box;
            size_t nBest=0;
            double dMinVolume=DBL_MAX;
            std::set<size_t>::iterator child=sChildren.begin();
            while(child!=sChildren.end())
                {
                SGM::Interval3D ChildBox=m_aData[*child].m_Box;
                ChildBox+=Box;
                double dVolume=ChildBox.Volume();
                if(dVolume<dMinVolume)
                    {
                    dMinVolume=dVolume;
                    nBest=*child;
                    }
                ++child;
                }
            nWhere=nBest;
            Parent=m_aData[nWhere];
            sChildren=Parent.m_sChildren;
            }
        m_aData[nWhere].m_sChildren.insert(nWhere);
        }
    }

void SGM::BoxTree::RemoveBox(SGM::Interval3D const &,//Box,
                             void            const *)//pData)
    {
    }

void SGM::BoxTree::AddPoint(SGM::Point3D const &,//Point,
                            void         const *)//pData)
    {
    }

void *SGM::BoxTree::FindPoint(SGM::Point3D const &,//Center,
                              double              )//dRadius)
    {
    return nullptr;
    }

void SGM::BoxTree::RemovePoint(SGM::Point3D const &,//Point,
                               void         const *)//pData)
    {
    }
#endif