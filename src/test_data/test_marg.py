import numpy as np
from numpy.linalg import pinv
from numpy.linalg import eig
import matplotlib.pylab as plt

# Load data
H = np.genfromtxt("/tmp/H.csv", delimiter=",")
H_ = np.genfromtxt("/tmp/H_solver.csv", delimiter=",")
b = np.genfromtxt("/tmp/b.csv", delimiter=",")
H_marg_ = np.genfromtxt("/tmp/H_marg.csv", delimiter=",")
b_marg_ = np.genfromtxt("/tmp/b_marg.csv", delimiter=",")

m = 15
r = H.shape[0] - m

# H = [Hmm, Hmr,
#      Hrm, Hrr]
Hmm = H[:m, :m]
Hmr = H[:m, m:]
Hrm = H[m:, :m]
Hrr = H[m:, m:]
Hmm_inv = pinv(Hmm)

# b = [b_mm, b_rr]
bmm = b[:m]
brr = b[m:]

# Marginalize
H_marg = Hrr - Hrm @ Hmm_inv @ Hmr
b_marg = brr - Hrm @ Hmm_inv @ bmm

# Plot
plt.imshow((H_marg > 0) * 1, cmap="gray")
plt.colorbar()
plt.show()

# Decompose H_marg into J' J components
# H_marg = J' * J = V * diag(w) * V'
# J = diag(w^{0.5}) * V'
# J_inv = diag(w^-0.5) * V'
# [V, w] = eig(H_marg)
# (w, V) = eig(H_marg + np.eye(H_marg.shape[0]))
