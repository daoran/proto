import sympy
from sympy import symbols
from sympy.matrices import Matrix
from sympy.printing.ccode import C99CodePrinter


def codegen_tf_inv():
  printer = C99CodePrinter()

  row0 = symbols('C0 C1 C2')
  row1 = symbols('C3 C4 C5')
  row2 = symbols('C6 C7 C8')
  C = Matrix([row0, row1, row2])

  r0, r1, r2 = symbols('r0, r1, r2')
  r = Matrix([r0, r1, r2])

  T = Matrix([[C.T, -C.T @ r], [0, 0, 0, 1]])
  # sympy.pprint(T)
  print(T)



codegen_tf_inv()
