import sympy
import numpy as np

T00, T01, T02, T03 = sympy.symbols("T00, T01, T02, T03")
T10, T11, T12, T13 = sympy.symbols("T10, T11, T12, T13")
T20, T21, T22, T23 = sympy.symbols("T20, T21, T22, T23")
T30, T31, T32, T33 = sympy.symbols("T30, T31, T32, T33")

T = np.array([[T00, T01, T02, T03],
              [T10, T11, T12, T13],
              [T20, T21, T22, T23],
              [T30, T31, T32, T33]])

p0, p1, p2 = sympy.symbols("p0, p1, p2")

p = np.array([[p0], [p1], [p2], [1.0]])

# T = sympy.MatrixSymbol("T", 4, 4)
# p = sympy.MatrixSymbol("p", 4, 1)

print(sympy.ccode(T * p))
