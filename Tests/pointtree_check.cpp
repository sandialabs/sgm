#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "SGMVector.h"
#include "SGMPointTree.h"

TEST(pointtree_check, empty)
    {
    SGM::PointTreeIndices2D Tree;
    EXPECT_TRUE(Tree.Empty());
    EXPECT_EQ(Tree.Size(),0);
    EXPECT_EQ(Tree.N,2);
    EXPECT_FALSE(Tree.Contains({0.,0.}));
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
    SGM::PointTreeIndices2D Tree(aPointAndIndices);
    EXPECT_TRUE(Tree.Size()==5);
    std::vector<unsigned> aNear = Tree.NearestNeighbors({0.9,0.9},1);
    EXPECT_EQ(aNear.size(),1);
    EXPECT_EQ(aNear[0],3);
    }
