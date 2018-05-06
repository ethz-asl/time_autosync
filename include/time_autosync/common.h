#ifndef COMMON_AUTO_TIMESYNC_H
#define COMMON_AUTO_TIMESYNC_H

#include <list>
#include <vector>

#include <Eigen/Eigen>

// Keep things in a range where machine precision shouldn't be an issue
constexpr double kMinVariance = 1e-20;
constexpr double kMaxVariance = 1e20;

// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

namespace Eigen {
template <typename Type>
using ArrayX = Eigen::Array<Type, Eigen::Dynamic, Eigen::Dynamic>;
}

#endif  // COMMON_AUTO_TIMESYNC_H
