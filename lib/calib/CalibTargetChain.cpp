#include "CalibTargetChain.hpp"
#include "SolvePnp.hpp"

namespace cartesian {

CalibTargetChain::CalibTargetChain(
    const std::map<int, CameraGeometryPtr> &camera_geometries,
    const std::map<int, CameraData> &camera_measurements) {
  // Lambda function - Get all camera timestamps
  auto get_timestamps =
      [](const std::map<int, CameraData> &camera_measurements) {
        std::set<timestamp_t> timestamps;
        for (const auto &[_, camera_data] : camera_measurements) {
          for (const auto &[ts, _] : camera_data) {
            timestamps.insert(ts);
          }
        }
        return timestamps;
      };

  // Lamda function - Form (camera, targets) map
  auto get_data = [](const timestamp_t ts,
                     const std::map<int, CameraData> &camera_measurements) {
    std::map<int, std::vector<CalibTargetPtr>> data;
    for (const auto &[camera_id, camera_data] : camera_measurements) {
      if (camera_data.count(ts) == 0) {
        continue;
      }
      for (const auto &[target_id, target] : camera_data.at(ts)) {
        if (target->getNumDetected() > 5) {
          data[camera_id].push_back(target);
        }
      }
    }
    return data;
  };

  // Form target-target extrinsic adjacency matrix
  // Note: From the common observations estimate target-target extrinsics. Here
  // we loop through different cameras observing different calibration targets
  // and form the adjacency matrix.
  for (const auto ts : get_timestamps(camera_measurements)) {
    for (auto &[camera_id, targets] : get_data(ts, camera_measurements)) {
      // Skip if camera has than 2 targets observed
      if (targets.size() < 2) {
        continue;
      }

      // Estimate relative pose T_CTi
      const auto camera = camera_geometries.at(camera_id);
      Mat4 T_CTi;
      Mat4 T_TiC;
      if (SolvePnp::estimate(camera, targets[0], T_CTi) != 0) {
        continue;
      }
      T_TiC = T_CTi.inverse();

      // Estimate relative pose T_CTj
      for (size_t j = 1; j < targets.size(); j++) {
        Mat4 T_CTj;
        if (SolvePnp::estimate(camera, targets[j], T_CTj) != 0) {
          continue;
        }

        // Insert into adjacency list
        const int target_i = targets[0]->getTargetId();
        const int target_j = targets[j]->getTargetId();
        insert(target_i, target_j, T_TiC * T_CTj);
      }
    }
  }
}

void CalibTargetChain::insert(const int i, const int j, const Mat4 &T_ij) {
  adjlist[i][j].push_back(T_ij);
  adjlist[j][i].push_back(T_ij.inverse());
}

int CalibTargetChain::find(const int i, const int j, Mat4 &T_TiTj) const {
  // Average extrinsic
  auto average_extrinsic = [&](const int i, const int j) {
    Vec3s positions;
    std::vector<Quat> rotations;
    for (const auto &extrinsic : adjlist.at(i).at(j)) {
      positions.push_back(tf_trans(extrinsic));
      rotations.push_back(tf_quat(extrinsic));
    }

    Vec3 pos = mean(positions);
    Quat rot = quat_average(rotations);
    return tf(rot, pos);
  };

  // Straight-forward case
  if (i == j) {
    T_TiTj = I(4);
    return 0;
  }

  // Check if we have even inserted the cameras before
  if (adjlist.count(i) == 0 || adjlist.count(j) == 0) {
    return -1;
  }

  // Check if we already have the extrinsics pair
  if (adjlist.count(i) && adjlist.at(i).count(j)) {
    T_TiTj = average_extrinsic(i, j);
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

    for (const auto &[child, _] : adjlist.at(parent)) {
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
  Mat4 T_TjTi = I(4);
  int child = j;
  while (path_map.count(child)) {
    const int parent = path_map[child];
    T_TiTj = T_TjTi * average_extrinsic(child, parent);
    child = parent;
  }
  T_TiTj = T_TjTi.inverse();

  return 0;
}

} // namespace cartesian
