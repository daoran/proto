#ifndef PROTOTYPE_CORE_DATA_IMPL_HPP
#define PROTOTYPE_CORE_DATA_IMPL_HPP

#include "prototype/core/data.hpp"

namespace proto {

template <typename T>
void pop_front(std::vector<T> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

template <typename T1, typename T2>
void pop_front(std::vector<T1, T2> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

template <typename T>
void extend(std::vector<T> &x, std::vector<T> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

template <typename T1, typename T2>
void extend(std::vector<T1, T2> &x, std::vector<T1, T2> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

template <typename T>
T set_union(const T &s1, const T &s2) {
  T result = s1;
  result.insert(s2.begin(), s2.end());
  return result;
}

template <typename T>
T set_diff(const T &a, const T &b) {
  T results;
  std::set_difference(
    a.begin(), a.end(),
    b.begin(), b.end(),
    std::inserter(results, results.end())
  );
  return results;
}

template <typename T>
T set_symmetric_diff(const T &a, const T &b) {
  T results;
  std::set_symmetric_difference(
      a.begin(), a.end(),
      b.begin(), b.end(),
      std::back_inserter(results));
  return results;
}

template <typename T>
std::set<T> intersection(const std::list<std::vector<T>> &vecs) {
  // Obtain element count across all vectors
  std::unordered_map<T, size_t> counter;
  for(const auto &vec: vecs) {  // Loop over all vectors
    for (const auto& p : vec) { // Loop over elements in vector
      counter[p] += 1;
    }
  }

  // Build intersection result
  std::set<T> retval;
  for (const auto &el : counter) {
    if (el.second == vecs.size()) {
      retval.insert(el.first);
    }
  }

  return retval;
}

} //  namespace proto
#endif // PROTOTYPE_CORE_DATA_IMPL_HPP
