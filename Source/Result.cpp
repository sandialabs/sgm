#include "SGMResult.h"
#include "SGMEntityFunctions.h"

#include <iostream>
#include <iomanip>

namespace SGM
{

#ifdef SGM_TIMING_RAY_FIRE

void Result::IncrementIntersectLineAndEntityCount(SGM::EntityType EntityType)
    {
    auto iter = m_mIntersectLineAndEntityCount.find(EntityType);
    if (iter == m_mIntersectLineAndEntityCount.end())
        m_mIntersectLineAndEntityCount.insert(std::pair<SGM::EntityType,size_t>(EntityType,1));
    else
        m_mIntersectLineAndEntityCount[EntityType] = iter->second + 1;
    }

void Result::PrintIntersectLineAndEntityCount() const
    {
    std::cout << std::left << std::setw(15) << "Surface Type" << "Number of Line Intersects" << std::endl;
    for (auto const &item : m_mIntersectLineAndEntityCount)
        {
        // align output to left, fill goes to right
        std::cout << std::left << std::setw(15) << EntityTypeName(item.first) << item.second << std::endl;
        }
    }

#endif // SGM_TIMING_RAY_FIRE

} // end namespace SGM

