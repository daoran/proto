#!/usr/bin/env python3
import multiprocessing as mp

import numpy as np


def project_points(hpts_world, K):
  uvd = hpts_world[:, :3] @ K.T
  # uvd /= uvd[:, 2]
  return uvd


def process_chunk(start_idx, chunk_size, hpts_world, K):
  num_points = hpts_world.shape[0]
  end_idx = min(start_idx + chunk_size, num_points)
  return project_points(hpts_world[start_idx:end_idx, :], K)


def project_points_multiprocessing(hpts_world, K, num_procs=3):
  num_points = hpts_world.shape[0]
  chunk_size = num_points // num_procs

  with mp.Pool(processes=num_procs) as pool:
    results = [
        pool.apply_async(
            process_chunk,
            args=(i * chunk_size, chunk_size, hpts_world, K),
        ) for i in range(num_procs)
    ]
    output = np.vstack([result.get() for result in results])

  return output


if __name__ == "__main__":
  # Setup
  N = 100000
  pts_world = np.random.rand(N, 3)
  hpts_world = np.hstack((pts_world, np.ones((N, 1))))
  K = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]])

  # Project points in a multiprocessing way
  uvd = project_points_multiprocessing(hpts_world, K, num_procs=8)
  print(uvd)
