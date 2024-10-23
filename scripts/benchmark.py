import time
import numpy as np


def method1(points, R, t):
  return np.dot(points, R.T) + t


def method2(hpoints, tf):
  transformed_points = np.dot(hpoints, tf.T)
  return transformed_points[:, :3]


def benchmark(N, repetitions=100):
  # Generate random Nx3 points
  points = np.random.rand(N, 3)
  ones = np.ones((points.shape[0], 1))
  hpoints = np.hstack((points, ones))

  # Create random rotation matrix and translation vector
  R = np.random.rand(3, 3)
  t = np.random.rand(3)

  # Create a 4x4 homogeneous transformation matrix
  tf = np.eye(4)
  tf[:3, :3] = R
  tf[:3, 3] = t

  # Benchmark Method 1
  start_time = time.time()
  for _ in range(repetitions):
    method1(points, R, t)
  time_method1 = (time.time() - start_time) / repetitions

  # Benchmark Method 2
  start_time = time.time()
  for _ in range(repetitions):
    method2(hpoints, tf)
  time_method2 = (time.time() - start_time) / repetitions

  print(f"Mean time for Method 1 (SO3 + R3): {time_method1:.2e} [s]")
  print(f"Mean time for Method 2 (SE3):      {time_method2:.2e} [s]")


# Run the benchmark
benchmark(N=100000000)
