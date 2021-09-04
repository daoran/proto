#ifndef PROTO_DATA_HPP
#define PROTO_DATA_HPP

#include "core.hpp"

namespace proto {

char *malloc_string(const char *s);
int csv_rows(const char *fp);
int csv_cols(const char *fp);
char **csv_fields(const char *fp, int *nb_fields);
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);
int **load_iarrays(const char *csv_path, int *nb_arrays);
real_t **load_darrays(const char *csv_path, int *nb_arrays);
int csv_rows(const std::string &file_path);
int csv_cols(const std::string &file_path);
int csv2mat(const std::string &file_path, const bool header, matx_t &data);
int mat2csv(const std::string &file_path, const matx_t &data);

// /**
//  * Convert vector to csv file.
//  * @returns 0 for success, -1 for failure
//  */
// int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data);

// /**
//  * Convert timestamps to csv file.
//  * @returns 0 for success, -1 for failure
//  */
// int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data);

void print_progress(const real_t percentage);
bool all_true(const std::vector<bool> x);

// /**
//  * Pop front of an `std::vector`.
//  */
// template <typename T>
// void pop_front(std::vector<T> &vec) {
//   assert(!vec.empty());
//   vec.front() = std::move(vec.back());
//   vec.pop_back();
// }
//
// /**
//  * Pop front of an `std::vector`.
//  */
// template <typename T1, typename T2>
// void pop_front(std::vector<T1, T2> &vec) {
//   assert(!vec.empty());
//   vec.front() = std::move(vec.back());
//   vec.pop_back();
// }
//
// /**
//  * Extend `std::vector`.
//  */
// template <typename T>
// void extend(std::vector<T> &x, std::vector<T> &add) {
//   x.reserve(x.size() + add.size());
//   x.insert(x.end(), add.begin(), add.end());
// }
//
// /**
//  * Extend `std::vector`.
//  */
// template <typename T1, typename T2>
// void extend(std::vector<T1, T2> &x, std::vector<T1, T2> &add) {
//   x.reserve(x.size() + add.size());
//   x.insert(x.end(), add.begin(), add.end());
// }
//
// /**
//  * Slice `std::vector`.
//  */
// template<typename T>
// std::vector<T> slice(std::vector<T> const &v, int m, int n) {
//   auto first = v.cbegin() + m;
//   auto last = v.cbegin() + n + 1;
//
//   std::vector<T> vec(first, last);
//   return vec;
// }
//
// /**
//  * Slice `std::vector`.
//  */
// template<typename T1, typename T2>
// std::vector<T1, T2> slice(std::vector<T1, T2> const &v, int m, int n) {
//   auto first = v.cbegin() + m;
//   auto last = v.cbegin() + n + 1;
//
//   std::vector<T1, T2> vec(first, last);
//   return vec;
// }
//
// /**
//  * Get raw pointer of a value in a `std::map`.
//  */
// template <typename K, typename V>
// const V *lookup(const std::map<K, V> &map, K key) {
//   typename std::map<K, V>::const_iterator iter = map.find(key);
//   if (iter != map.end()) {
//     return &iter->second;
//   } else {
//     return nullptr;
//   }
// }
//
// /**
//  * Get raw pointer of a value in a `std::map`.
//  */
// template <typename K, typename V>
// V *lookup(std::map<K, V> &map, K key) {
//   return const_cast<V *>(lookup(const_cast<const std::map<K, V> &>(map), key));
// }

// /**
//  * Get raw pointer of a value in a `std::map`.
//  */
// template <typename K, typename V>
// const V *lookup(const std::unordered_map<K, V> &map, K key) {
//   typename std::unordered_map<K, V>::const_iterator iter = map.find(key);
//   if (iter != map.end()) {
//     return &iter->second;
//   } else {
//     return nullptr;
//   }
// }
//
// /**
//  * Get raw pointer of a value in a `std::map`.
//  */
// template <typename K, typename V>
// V *lookup(std::unordered_map<K, V> &map, K key) {
//   return const_cast<V *>(
//       lookup(const_cast<const std::unordered_map<K, V> &>(map), key));
// }

// /**
//  * Union between set `a` and set `b`.
//  */
// template <typename T>
// T set_union(const T &s1, const T &s2) {
//   T result = s1;
//   result.insert(s2.begin(), s2.end());
//   return result;
// }
//
// /**
//  * Difference between `a` and set `b`.
//  */
// template <typename T>
// T set_diff(const T &a, const T &b) {
//   T results;
//   std::set_difference(a.begin(),
//                       a.end(),
//                       b.begin(),
//                       b.end(),
//                       std::inserter(results, results.end()));
//   return results;
// }
//
// /**
//  * Symmetric difference between `a` and `b`.
//  */
// template <typename T>
// T set_symmetric_diff(const T &a, const T &b) {
//   T results;
//   std::set_symmetric_difference(a.begin(),
//                                 a.end(),
//                                 b.begin(),
//                                 b.end(),
//                                 std::back_inserter(results));
//   return results;
// }
//
// /**
//  * Intersection between std::vectors `vecs`.
//  * @returns Number of common elements
//  */
// template <typename T>
// std::set<T> intersection(const std::list<std::vector<T>> &vecs) {
//   // Obtain element count across all vectors
//   std::unordered_map<T, size_t> counter;
//   for (const auto &vec : vecs) { // Loop over all vectors
//     for (const auto &p : vec) {  // Loop over elements in vector
//       counter[p] += 1;
//     }
//   }
//
//   // Build intersection result
//   std::set<T> retval;
//   for (const auto &el : counter) {
//     if (el.second == vecs.size()) {
//       retval.insert(el.first);
//     }
//   }
//
//   return retval;
// }

// /**
//  * Ordered Set
//  */
// template <class T>
// class ordered_set_t {
// public:
//   using iterator                     = typename std::vector<T>::iterator;
//   using const_iterator               = typename std::vector<T>::const_iterator;
//
//   iterator begin()                   { return vector.begin(); }
//   iterator end()                     { return vector.end(); }
//   const_iterator begin() const       { return vector.begin(); }
//   const_iterator end() const         { return vector.end(); }
//   const T& at(const size_t i) const  { return vector.at(i); }
//   const T& front() const             { return vector.front(); }
//   const T& back() const              { return vector.back(); }
//   void insert(const T& item)         { if (set.insert(item).second) vector.push_back(item); }
//   size_t count(const T& item) const  { return set.count(item); }
//   bool empty() const                 { return set.empty(); }
//   size_t size() const                { return set.size(); }
//   void clear()                       { vector.clear(); set.clear(); }
//
// private:
//   std::vector<T> vector;
//   std::set<T>    set;
// };

void save_features(const std::string &path, const vec3s_t &features);
void save_pose(FILE *csv_file,
               const timestamp_t &ts,
               const quat_t &rot,
               const vec3_t &pos);
void save_pose(FILE *csv_file, const timestamp_t &ts, const vecx_t &pose);
void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const quats_t &orientations,
                const vec3s_t &positions);
void save_poses(const std::string &path, const mat4s_t &poses);

int check_jacobian(const std::string &jac_name,
                   const matx_t &fdiff,
                   const matx_t &jac,
                   const real_t threshold,
                   const bool print=false);

} // namespace proto
#endif // PROTO_DATA_HPP
