#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "SGMVector.h"
#include "SGMPointTree.h"

TEST(pointtree_check, empty)
    {
    SGM::PointTree<SGM::Point2D, SGM::Point2D*> PointTree;
    EXPECT_TRUE(PointTree.Empty());
    EXPECT_EQ(PointTree.Size(),0);
    EXPECT_EQ(PointTree.N,2);
    EXPECT_FALSE(PointTree.Contains({0.,0.}));
    }

TEST(pointtree_check, basic_2D)
    {
    std::vector<std::pair<SGM::Point2D, unsigned>> aPointAndIndices =
        {
            {{0.0, 0.0}, 0},
            {{1.0, 0.0}, 1},
            {{0.0, 1.0}, 2},
            {{1.0, 1.0}, 3},
            {{0.0, 0.0}, 4}   // repeated Point
        };
    SGM::PointTree<SGM::Point2D, unsigned> PointTree(aPointAndIndices);
    EXPECT_TRUE(PointTree.Size()==5);
    std::vector<unsigned> aNear = PointTree.NearestNeighbors({0.9,0.9},1);
    EXPECT_EQ(aNear.size(),1);
    EXPECT_EQ(aNear[0],3);
    }
