from math import pi
from pathlib import Path

import cv2
import numpy as np
from scipy.signal import convolve2d
from scipy.stats import norm
from scipy.spatial import cKDTree
import scipy.ndimage as ndi


def normalize_image(image):
  """
  Normalize image to between 0 to  1
  """
  assert len(image.shape) == 2
  return image / 255

  # blur_size = int(np.sqrt(image.size) / 2)
  # grayb = cv2.GaussianBlur(image, (3, 3), 1)
  # gray_mu = cv2.blur(grayb, (blur_size, blur_size))
  # diff = (np.float32(grayb) - gray_mu) / 255.0
  # diff = np.clip(grayb, -0.1, 0.1) + 0.1
  # diff = (diff - np.min(diff)) / (np.max(diff) - np.min(diff))
  # return diff


def z_score_normalization(image):
  """
  Z-score Normalization
  """
  mean, std = np.mean(image), np.std(image)
  return (image - mean) / (std + 1e-8)  # Avoid division by zero


def gamma_correction(image, gamma=0.5):
  """
  Gamma correction
  """
  image = image / 255.0  # Normalize to [0,1]
  return np.power(image, gamma) * 255.0  # Apply gamma and rescale


def histogram_equalization(image):
  """
  Histogram Equalization
  """
  hist, bins = np.histogram(image.flatten(), bins=256, range=[0, 256])
  cdf = hist.cumsum()  # Cumulative distribution function
  cdf_normalized = cdf * 255 / cdf[-1]  # Normalize to [0,255]
  return np.interp(image.flatten(), bins[:-1],
                   cdf_normalized).reshape(image.shape)


def correlation_patch(angle_1, angle_2, radius):
  """
  Form correlation patch
  """
  # Width and height
  width = radius * 2 + 1
  height = radius * 2 + 1

  # Initialize template
  template = []
  for i in range(4):
    x = np.zeros((height, width))
    template.append(x)

  # Midpoint
  mu = radius
  mv = radius

  # Compute normals from angles
  n1 = [-np.sin(angle_1), np.cos(angle_1)]
  n2 = [-np.sin(angle_2), np.cos(angle_2)]

  # For all points in template do
  for u in range(width):
    for v in range(height):
      # Vector
      vec = [u - mu, v - mv]
      dist = np.linalg.norm(vec)

      # Check on which side of the normals we are
      s1 = np.dot(vec, n1)
      s2 = np.dot(vec, n2)

      if dist <= radius:
        if s1 <= -0.1 and s2 <= -0.1:
          template[0][v, u] = 1
        elif s1 >= 0.1 and s2 >= 0.1:
          template[1][v, u] = 1
        elif s1 <= -0.1 and s2 >= 0.1:
          template[2][v, u] = 1
        elif s1 >= 0.1 and s2 <= -0.1:
          template[3][v, u] = 1

  # Normalize
  for i in range(4):
    template[i] /= np.sum(template[i])

  return template


def non_maxima_suppression(image, n=3, tau=0.1, margin=2):
  """
  Non Maximum Suppression

  Args:

    image: Input image
    n: Kernel size
    tau: Corner response threshold
    margin: Offset away from image boundaries

  Returns:

    List of corners with maximum response

  """
  height, width = image.shape
  maxima = []

  for i in range(n + margin, width - n - margin, n + 1):
    for j in range(n + margin, height - n - margin, n + 1):
      # Initialize max value
      maxi = i
      maxj = j
      maxval = image[j, i]

      # Get max value in kernel
      for i2 in range(i, i + n):
        for j2 in range(j, j + n):
          currval = image[j2, i2]
          if currval > maxval:
            maxi = i2
            maxj = j2
            maxval = currval

      # Make sure maxval is larger than neighbours
      failed = 0
      for i2 in range(maxi - n, min(maxi + n, width - margin)):
        for j2 in range(maxj - n, min(maxj + n, height - margin)):
          currval = image[j2, i2]
          if currval > maxval and (i2 < i or i2 > i + n or j2 < j or
                                   j2 > j + n):
            failed = 1
            break
        if failed:
          break

      # Store maxval
      if maxval >= tau and failed == 0:
        maxima.append([maxi, maxj])

  return maxima


def find_modes_mean_shift(hist, sigma):
  """
  Efficient mean-shift approximation by histogram smoothing.

  Args:
    hist (numpy.ndarray): 1D histogram.
    sigma (float): Standard deviation of Gaussian kernel.

  Returns:
    tuple: A tuple containing two numpy arrays:
      - modes: A 2D array where each row represents a mode,
               with columns [index, smoothed_histogram_value].
      - hist_smoothed: The smoothed histogram.
  """
  hist_len = len(hist)
  hist_smoothed = np.zeros(hist_len)

  # Compute smoothed histogram
  for i in range(hist_len):
    j = np.arange(-int(round(2 * sigma)), int(round(2 * sigma)) + 1)
    idx = (i + j) % hist_len  # Handle wraparound
    hist_smoothed[i] = np.sum(hist[idx] * norm.pdf(j, 0, sigma))

  # Initialize empty array
  modes = np.array([], dtype=int).reshape(0, 2)

  # Check if all entries are nearly identical (to avoid infinite loop)
  if np.all(np.abs(hist_smoothed - hist_smoothed[0]) < 1e-5):
    return modes, hist_smoothed  # Return empty modes

  # Mode finding
  for i in range(hist_len):
    j = i
    while True:
      h0 = hist_smoothed[j]
      j1 = (j + 1) % hist_len
      j2 = (j - 1) % hist_len
      h1 = hist_smoothed[j1]
      h2 = hist_smoothed[j2]

      if h1 >= h0 and h1 >= h2:
        j = j1
      elif h2 > h0 and h2 > h1:
        j = j2
      else:
        break

    # Check if mode already found (more efficient than list search)
    if modes.size == 0 or not np.any(modes[:, 0] == j):
      modes = np.vstack((modes, [j, hist_smoothed[j]]))

  # Sort modes by smoothed histogram value (descending)
  idx = np.argsort(modes[:, 1])[::-1]  # Get indices for descending sort
  modes = modes[idx]

  return modes, hist_smoothed


def edge_orientations(img_angle, img_weight):
  # Initialize v1 and v2
  v1 = np.array([0, 0])
  v2 = np.array([0, 0])

  # Number of bins (histogram parameter)
  bin_num = 32

  # Convert images to vectors
  vec_angle = img_angle.flatten()
  vec_weight = img_weight.flatten()

  # Convert angles from normals to directions
  vec_angle = vec_angle + np.pi / 2
  vec_angle[vec_angle > np.pi] -= np.pi

  # Create histogram
  angle_hist = np.zeros(bin_num)
  for i in range(len(vec_angle)):
    bin_idx = min(max(int(np.floor(vec_angle[i] / (np.pi / bin_num))), 0),
                  bin_num - 1)
    angle_hist[bin_idx] += vec_weight[i]

  # Find modes of smoothed histogram
  modes, angle_hist_smoothed = find_modes_mean_shift(angle_hist, 1)

  # If only one or no mode => return invalid corner
  if modes.shape[0] <= 1:
    return v1, v2

  # Compute orientation at modes
  modes = np.hstack(
      (modes, ((modes[:, 0] - 1) * np.pi / bin_num).reshape(-1, 1)))

  # Extract 2 strongest modes and sort by angle
  modes = modes[:2]
  modes = modes[np.argsort(modes[:, 2])]

  # Compute angle between modes
  delta_angle = min(modes[1, 2] - modes[0, 2],
                    modes[0, 2] + np.pi - modes[1, 2])

  # If angle too small => return invalid corner
  if delta_angle <= 0.3:
    return v1, v2

  # Set statistics: orientations
  v1 = np.array([np.cos(modes[0, 2]), np.sin(modes[0, 2])])
  v2 = np.array([np.cos(modes[1, 2]), np.sin(modes[1, 2])])

  return v1, v2


def refine_corners(img_du, img_dv, img_angle, img_weight, corners, r=10):
  # Image dimensions
  height, width = img_du.shape

  # Init orientations to invalid (corner is invalid iff orientation=0)
  N = len(corners)
  corners_inliers = []
  v1 = []
  v2 = []

  # for all corners do
  for i, (cu, cv) in enumerate(corners):
    # Estimate edge orientations
    rs = max(cv - r, 1)
    re = min(cv + r, height)
    cs = max(cu - r, 1)
    ce = min(cu + r, width)
    img_angle_sub = img_angle[rs:re, cs:ce]
    img_weight_sub = img_weight[rs:re, cs:ce]
    v1_edge, v2_edge = edge_orientations(img_angle_sub, img_weight_sub)

    # Check invalid edge
    if np.array_equal(v1_edge, [0.0, 0.0]):
      continue
    if np.array_equal(v2_edge, [0.0, 0.0]):
      continue

    corners_inliers.append(corners[i])
    v1.append(v1_edge)
    v2.append(v2_edge)

  return corners, v1, v2


def max_pooling(corr, step=40, thres=0.01):
  """
  Extracts strong corner candidates from a corner response matrix using a
  grid-based local max-pooling approach.

  This function scans the input matrix in a grid-wise manner, selecting the
  strongest corner in each region while ensuring detected corners are spaced
  apart and meet a minimum response threshold.

  Parameters:
  -----------
  corr : np.ndarray
      The corner response matrix (e.g., from Harris or Shi-Tomasi corner
      detectors).

  step : int, optional (default=40)
      The size of the local region (window) for non-max suppression. Larger
      values ensure more spaced-out corners.

  thres : float, optional (default=0.01)
      Minimum response value for a corner to be considered valid.

  Returns:
  --------
  np.ndarray

      A NumPy array of shape (N, 3), where each row represents a detected
      corner with:
      - Row index of the corner
      - Column index of the corner
      - Corner response value

  Example:
  --------
  >>> corner_response = np.random.rand(100, 100)  # Simulated response matrix
  >>> corners = get_corner_candidates(corner_response, step=20, thres=0.05)
  >>> print(corners)
  [[12 34 0.08]
   [52 76 0.12]
   [85 90 0.15] ...]
  """
  out = []
  check = set()

  for i in range(0, corr.shape[0], step // 2):
    for j in range(0, corr.shape[1], step // 2):
      # Get row, column index, and max value in local region
      region = corr[i:i + step, j:j + step]
      ix = np.argmax(region)
      r, c = np.unravel_index(ix, region.shape)
      val = region[r, c]

      # Keep if larger than threshold
      if val > thres and (r + i, c + j) not in check:
        out.append((r + i, c + j, val))
        check.add((r + i, c + j))

  return np.array(out)


def detect_corners(image, radiuses=[6, 8, 10]):
  """
  Detect corners
  """
  # Convert gray image to double
  assert len(image.shape) == 2
  image = normalize_image(image)

  # Find corners
  template_props = [[0.0, pi / 2.0], [pi / 4.0, -pi / 4.0]]
  corr = np.zeros(image.shape)
  for angle_1, angle_2 in template_props:
    for radius in radiuses:
      template = correlation_patch(angle_1, angle_2, radius)

      img_corners = [
          convolve2d(image, template[0], mode="same"),
          convolve2d(image, template[1], mode="same"),
          convolve2d(image, template[2], mode="same"),
          convolve2d(image, template[3], mode="same"),
      ]
      img_corners_mu = np.mean(img_corners, axis=0)
      arr = np.array([
          img_corners[0] - img_corners_mu,
          img_corners[1] - img_corners_mu,
          img_corners_mu - img_corners[2],
          img_corners_mu - img_corners[3],
      ])
      img_corners_1 = np.min(arr, axis=0)  # Case 1: a = white, b = black
      img_corners_2 = np.min(-arr, axis=0)  # Case 2: b = white, a = black

      # Combine both
      img_corners = np.max([img_corners_1, img_corners_2], axis=0)

      # Max
      corr = np.max([img_corners, corr], axis=0)

  # Max pooling
  corners = max_pooling(corr, 10, np.max(corr) * 0.2)

  # Refine corners
  # du = np.array([
  #     [-1, 0, 1],
  #     [-1, 0, 1],
  #     [-1, 0, 1],
  # ])
  # dv = du.T
  # img_du = convolve2d(image, du, mode='same')
  # img_dv = convolve2d(image, dv, mode='same')
  # img_angle = np.arctan2(img_dv, img_du)
  # img_weight = np.sqrt(img_du**2 + img_dv**2)
  # corners, v1, v2 = refine_corners(img_du, img_dv, img_angle, img_weight,
  #                                  corners)

  vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
  for i in range(corners.shape[0]):
    py, px = corners[i, :2]
    center = (int(px), int(py))
    radius = 1
    color = (0, 0, 255)
    thickness = 2
    cv2.circle(vis, center, radius, color, thickness)
  cv2.imshow("vis", vis)
  cv2.waitKey(0)


def checkerboard_score(corners, size=(9, 6)):
  corners_reshaped = corners[:, :2].reshape(*size, 2)
  maxm = 0
  for rownum in range(size[0]):
    for colnum in range(1, size[1] - 1):
      pts = corners_reshaped[rownum, [colnum - 1, colnum, colnum + 1]]
      top = np.linalg.norm(pts[2] + pts[0] - 2 * pts[1])
      bot = np.linalg.norm(pts[2] - pts[0])
      if np.abs(bot) < 1e-9:
        return 1
      maxm = max(top / bot, maxm)
  for colnum in range(0, size[1]):
    for rownum in range(1, size[0] - 1):
      pts = corners_reshaped[[rownum - 1, rownum, rownum + 1], colnum]
      top = np.linalg.norm(pts[2] + pts[0] - 2 * pts[1])
      bot = np.linalg.norm(pts[2] - pts[0])
      if np.abs(bot) < 1e-9:
        return 1
      maxm = max(top / bot, maxm)
  return maxm


# Load the image
euroc_data = Path("/data/euroc")
calib_dir = euroc_data / "cam_checkerboard" / "mav0" / "cam0" / "data"
calib_image = calib_dir / "1403709080437837056.png"
image = cv2.imread(str(calib_image), cv2.COLOR_BGR2GRAY)
image = image.astype(np.float32)
cb_size = (7, 6)
winsize = 9

# vis = cv2.imread(str(calib_image))
# image = image.astype(np.float32)
# image = normalize_image(image)
# image = z_score_normalization(image)
# image = gamma_correction(image)
# image = histogram_equalization(image)

detect_corners(image)
