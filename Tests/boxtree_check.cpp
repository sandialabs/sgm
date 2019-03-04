#include <string>
#include <gtest/gtest.h>

#include "SGMBoxTree.h"

using SGM::UnitVector3D;
using SGM::Interval3D;
using SGM::Point3D;

using SGM::Ray3D;
using SGM::BoxTree;

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#endif

TEST(boxtree_check, intersect_single_item_tree)
{
    BoxTree boxTree;

    Point3D point0(3, 3, 0);
    boxTree.Insert(&point0, {3,3,3,3,0,0});
    Ray3D ray_start_outside_parallel_x({1.0,3.0,0.0}, {1., 0., 0.});
    auto container = boxTree.FindIntersectsLine(ray_start_outside_parallel_x);
    EXPECT_EQ(container.size(), 1);
    EXPECT_EQ(container[0].first, &point0);
}

TEST(boxtree_check, copy_construct)
{
    BoxTree boxTree;
    // build a tree with a handful of (object,bbox) items
    std::vector<Point3D> points;
    for (int i = 0; i < 43; ++i)
        {
                points.emplace_back((double)i*17, (double)i*19, (double)i*23);
                Point3D &point = points.back();
                boxTree.Insert(&point, Interval3D(point));
        }
    // make a copy of tree
    BoxTree boxTreeCopy(boxTree);
    // get items in each tree and compare
    auto contents = boxTree.FindAll();
    auto contentsCopy = boxTreeCopy.FindAll();
    EXPECT_EQ(contents, contentsCopy);
}

TEST(boxtree_check, assignment)
{
    BoxTree boxTree;
    // build a tree with a handful of (object,bbox) items
    std::vector<Point3D> points;
    for (int i = 0; i < 43; ++i)
        {
        points.emplace_back((double)i*17, (double)i*19, (double)i*23);
        Point3D &point = points.back();
        boxTree.Insert(&point, Interval3D(point));
        }
    // make a copy of tree
    BoxTree boxTreeCopy;
    boxTreeCopy = boxTree;
    // get items in each tree and compare
    auto contents = boxTree.FindAll();
    auto contentsCopy = boxTreeCopy.FindAll();
    EXPECT_EQ(contents, contentsCopy);
}

TEST(boxtree_check, replace)
{
    BoxTree box_tree;
    BoxTree box_tree_other;
    std::vector<Point3D> points;
    std::vector<Point3D> other_points;
    std::map<const void*, const void*> point_map;
    // make two different vectors of identical points,
    // with a map from one to the other's entries
    // create a tree from the first vector
    // and a tree from the second vector
    for (int i = 0; i < 23; ++i)
        {
        points.emplace_back((double)i*17, (double)i*19, (double)i*23);
        other_points.emplace_back((double)i*17, (double)i*19, (double)i*23);
        Point3D &point = points.back();
        Point3D &other_point = other_points.back();
        point_map[&point] = &other_point;
        box_tree.Insert(&point, Interval3D(point));
        box_tree_other.Insert(&other_point, Interval3D(other_point));
        }
    // replace the items (pointers to points) in the first tree with those from the second vector
    box_tree.Replace(point_map);

    // get items in each tree and compare
    auto contents = box_tree.FindAll();
    auto contentsCopy = box_tree_other.FindAll();
    EXPECT_EQ(contents, contentsCopy);
}

TEST(boxtree_check, erase_enclosed)
    {
    BoxTree boxTree;
    // build a tree with a handful of (object,bbox) items
    std::vector<Point3D> points;
    for (int i = 0; i < 2; ++i)
        {
        points.emplace_back((double)i*1, (double)i*2, (double)i*3);
        Point3D &point = points.back();
        boxTree.Insert(&point, Interval3D(point));
        }
    // erase the points
    Interval3D bound(-0.1,0.1,-0.1,0.1,-0.1,0.1);
    boxTree.EraseEnclosed(bound);
    EXPECT_EQ(boxTree.Size(),1);
    }

#ifdef __clang__
#pragma clang diagnostic pop
#endif
