#include "CameraChain.hpp"
#include "SolvePnp.hpp"

namespace xyz {

CameraChain::CameraChain(
    const std::map<int, CameraGeometryPtr> &camera_geometries,
    const std::map<int, CameraData> &camera_measurements) {
  // Get all camera timestamps
  std::set<timestamp_t> timestamps;
  for (const auto &[_, camera_data] : camera_measurements) {
    for (const auto &[ts, _] : camera_data) {
      timestamps.insert(ts);
    }
  }

  // Form camera adjacency matrix
  for (const auto ts : timestamps) {
    // Form (target_id, camera_ids) map at time ts
    std::map<int, std::vector<int>> target_counter;
    for (const auto &[camera_id, camera_data] : camera_measurements) {
      if (camera_data.count(ts) == 0) {
        continue;
      }
      for (const auto &[target_id, target] : camera_data.at(ts)) {
        target_counter[target_id].push_back(camera_id);
      }
    }

    // From the common observations estimate camera-camera extrinsics. Here we
    // loop through different calibration targets observed by different cameras
    // in the `camera_ids` vector and form the adjacency matrix at time `ts`.
    for (const auto &[target_id, camera_ids] : target_counter) {
      // Pre-check
      if (camera_ids.size() < 2) {
        continue;
      }

      // Obtain co-observations
      std::map<int, CalibTargetPtr> targets;
      for (auto camera_id : camera_ids) {
        targets[camera_id] =
            camera_measurements.at(camera_id).at(ts).at(target_id);
      }

      // Estimate relative pose T_CiT
      const int index_i = camera_ids[0];
      const auto &target_i = targets[index_i];
      const auto camera_i = camera_geometries.at(index_i);
      Mat4 T_CiT;
      if (SolvePnp::estimate(camera_i, target_i, T_CiT) != 0) {
        continue;
      }

      // Estimate relative pose T_CjT
      for (size_t j = 1; j < camera_ids.size(); j++) {
        const int index_j = camera_ids[j];
        const auto &target_j = targets[index_j];
        const auto camera_j = camera_geometries.at(index_j);

        // Solvepnp T_CjT
        Mat4 T_CjT;
        if (SolvePnp::estimate(camera_j, target_j, T_CjT) != 0) {
          continue;
        }

        // Insert into adjacency list
        insert(index_i, index_j, T_CiT * T_CjT.inverse());
      }
    }
  }
}

void CameraChain::insert(const int i, const int j, const Mat4 &T_ij) {
  adjlist_[i][j].push_back(T_ij);
  adjlist_[j][i].push_back(T_ij.inverse());
}

int CameraChain::find(const int i, const int j, Mat4 &T_CiCj) const {
  // Average extrinsic
  auto average_extrinsic = [&](const int i, const int j) {
    Vec3s positions;
    std::vector<Quat> rotations;
    for (const auto &extrinsic : adjlist_.at(i).at(j)) {
      positions.push_back(tf_trans(extrinsic));
      rotations.push_back(tf_quat(extrinsic));
    }

    Vec3 pos = mean(positions);
    Quat rot = quat_average(rotations);
    return tf(rot, pos);
  };

  // Straight-forward case
  if (i == j) {
    T_CiCj = I(4);
    return 0;
  }

  // Check if we have even inserted the cameras before
  if (adjlist_.count(i) == 0 || adjlist_.count(j) == 0) {
    return -1;
  }

  // Check if we already have the extrinsics pair
  if (adjlist_.count(i) && adjlist_.at(i).count(j)) {
    T_CiCj = average_extrinsic(i, j);
    return 0;
  }

  // Iterative BFS - To get path from camera i to j
  bool found_target = false;
  std::deque<int> queue;
  std::map<int, bool> visited;
  std::map<int, int> path_map;

  queue.push_back(i);
  while (!queue.empty()) {
    const auto parent = queue.front();
    queue.pop_front();
    visited[parent] = true;

    for (const auto &[child, _] : adjlist_.at(parent)) {
      if (visited[child]) {
        continue;
      }

      queue.push_back(child);
      path_map[child] = parent;

      if (child == j) {
        found_target = true;
        break;
      }
    }
  }

  // Check if we've found the target
  if (found_target == false) {
    return -2;
  }

  // Traverse the path backwards and chain the transforms
  Mat4 T_CjCi = I(4);
  int child = j;
  while (path_map.count(child)) {
    const int parent = path_map[child];
    T_CiCj = T_CjCi * average_extrinsic(child, parent);
    child = parent;
  }
  T_CiCj = T_CjCi.inverse();

  return 0;
}

} // namespace xyz
