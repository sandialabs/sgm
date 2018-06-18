#include <chrono>
#include <random>
#include <ctime>
#include <string>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <functional>

#include "SGMBoxTree.h"

#if 0
int main(int /*argc*/, char ** /*argv*/)
    {
    return 0;
    }
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Example of timing the insertion of a large number of randomly
// sized bounding boxes in the SGM::BoxTree.
//
// Also queries of all boxes in the tree enclosed in a given box,
// and removing those boxes.
//
///////////////////////////////////////////////////////////////////////////////

struct LeafCounter {
	int count;
	bool bContinueVisiting;
	
	LeafCounter() : count(0), bContinueVisiting(true) {};
	
	void operator()(SGM::BoxTree::Leaf const *)
	{
		count++;
		//std::cout << "Visiting " << count << std::endl;
	}
};

std::string ToString(SGM::Interval3D const& box)
{
	std::stringstream ss("");
	ss << '[';
	ss << '(' << box.m_XDomain.m_dMin << ',' << box.m_XDomain.m_dMax << "),";
	ss << '(' << box.m_YDomain.m_dMin << ',' << box.m_YDomain.m_dMax << "),";
	ss << '(' << box.m_ZDomain.m_dMin << ',' << box.m_ZDomain.m_dMax << ")]";
	return ss.str();
}

typedef std::chrono::steady_clock::time_point time_point;
typedef std::chrono::steady_clock::time_point::duration duration;

duration print_elapsed_time(const time_point &start) {
    duration diff = std::chrono::steady_clock::now() - start;
    std::cout << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
    return diff;
}

int main(int /*argc*/, char ** /*argv*/)
{
	SGM::BoxTree tree;
	LeafCounter leafCounter;

	// setup C++ random
	//std::random_device rd;  // random seed
	//std::mt19937 mt(rd());

    std::mt19937 mt;
    auto dist_origin = std::bind(std::uniform_real_distribution<double>(0.,900.), mt);
    auto dist_length = std::bind(std::uniform_real_distribution<double>(1.,100.), mt);

    time_point start;
    duration sum;

	const size_t nodes = 100000;

	const void * ptr = nullptr;

	sum = std::chrono::milliseconds(0);

	std::cout << "Insert: " << nodes << " nodes " << std::flush;
    start = std::chrono::steady_clock::now();
    for (int i = 0; i<nodes; i++) {
        double xmin = dist_origin();
        double ymin = dist_origin();
        double zmin = dist_origin();
        double xlen = dist_length();
        double ylen = dist_length();
        double zlen = dist_length();
        tree.Insert(ptr, SGM::Interval3D(xmin, xmin+xlen, ymin, ymin+ylen, zmin, zmin+zlen));
        }
    sum += print_elapsed_time(start);

	SGM::Interval3D bound(100.0, 300.0, 100.0, 400.0, 100.0, 500.0);

    std::cout << "Visit all nodes: " << std::flush;
    start = std::chrono::steady_clock::now();
    leafCounter = tree.Query(SGM::BoxTree::IsAny(), LeafCounter());
    std::cout << "IsAny: " << leafCounter.count << " nodes visited (" << tree.Size() << " nodes in tree) ";
    sum += print_elapsed_time(start);

    std::cout << "Search in a box: " << std::flush;
    start = std::chrono::steady_clock::now();
    leafCounter = tree.Query(SGM::BoxTree::IsEnclosing(bound), LeafCounter());
    std::cout << "IsEnclosing " << leafCounter.count << " nodes (" << tree.Size() << " nodes in tree) ";
    sum += print_elapsed_time(start);

    std::cout << "Remove enclosed in box: " << std::flush;
    start = std::chrono::steady_clock::now();
    tree.EraseEnclosed(bound);
    std::cout << "EraseEnclosed " << ToString(bound) << " ";
    sum += print_elapsed_time(start);

    std::cout << "Search in a box: " << std::flush;
    start = std::chrono::steady_clock::now();
    leafCounter = tree.Query(SGM::BoxTree::IsEnclosing(bound), LeafCounter());
    std::cout << "IsEnclosing " << leafCounter.count << " nodes. (" << tree.Size() << " nodes in tree) ";
    sum += print_elapsed_time(start);

    std::cout << "Total time: " << std::chrono::duration<double, std::milli>(sum).count() << " ms" << std::endl;

	return 0;
}

