#!/usr/bin/env python3
import numpy as np
from numpy.linalg import eig
import matplotlib.pylab as plt

H = np.loadtxt("/tmp/H.csv", delimiter=",")
b = np.loadtxt("/tmp/b.csv", delimiter=",")
H_marg = np.loadtxt("/tmp/H_marg.csv", delimiter=",")
b_marg = np.loadtxt("/tmp/b_marg.csv", delimiter=",")


def shur_complement(H, b, m, eps=1e-8):
  """ Schur complement """
  # H = [H_mm, H_mr,
  #      H_rm, H_rr]
  H_mm = H[:m, :m]
  H_mr = H[:m, m:]
  H_rm = H[m:, :m]
  H_rr = H[m:, m:]

  # b = [b_mm,
  #      b_rr]
  b_mm = b[:m]
  b_rr = b[m:]

  # Invert H_mm matrix sub-block via Eigen-decomposition
  H_mm = 0.5 * (H_mm + H_mm.T)  # Enforce symmetry
  w, V = eig(H_mm)
  w_inv = np.zeros(w.shape)

  for idx, w_i in enumerate(w):
    if w_i > eps:
      w_inv[idx] = 1.0 / w_i
    else:
      w[idx] = 0.0
      w_inv[idx] = 0.0

  Lambda_inv = np.diag(w_inv)
  H_mm_inv = V @ Lambda_inv @ V.T

  # Check inverse
  check_inverse = True
  if check_inverse:
    inv_norm = np.linalg.norm((H_mm @ H_mm_inv) - np.eye(H_mm.shape[0]))
    if inv_norm > 1e-8:
      print("Hmmm... inverse check failed in MargFactor!")

  # Apply Shur-Complement
  H_marg = H_rr - H_rm @ H_mm_inv @ H_mr
  b_marg = b_rr - H_rm @ H_mm_inv @ b_mm

  return (H_marg, b_marg)


# Schur complement
m = 15
(H_marg, b_marg) = shur_complement(H, b, m)

# Plot
plt.imshow(H)
plt.colorbar()
plt.show()
