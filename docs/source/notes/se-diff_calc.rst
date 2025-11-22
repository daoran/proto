Differential Calculus
=====================

Lie Group :math:`\SO{3}`

* Not a vector space
* Has no addition operator
* Has no subtraction operator


State estimation frameworks rely on small differences and gradients in order to
correct the state estimate. Orientations unlike translation and velocity do not
have an addition operator, as such it is more involving to update or find the
gradients of orientations. Forunately, since orientations are a special
orhogonal group :math:`\SO{3}` as well as a Lie group, an exponential map
exists that relates to its Lie algebra allowing orientations to be perturbed
and its gradients calculated.

Elements in Lie algebra are abstract vectors and not suitable for actual
computations. A basis :math:`\Mat{B} = [\Vec{\boldsymbol{\varphi}}_{1}
\enspace \Vec{\boldsymbol{\varphi}}_{2} \enspace
\Vec{\boldsymbol{\varphi}}_{3}]` can be used to extend the map to
:math:`\Real{3}`.

The definition of an exponential map :math:`\text{exp} : \Real{3} \mapsto
\SO{3}` of a coordinate tuple :math:`\boldsymbol{\varphi} (\varphi_1, \varphi_2,
\varphi_3) \in \Real{3}` is defined by

.. math::
  \text{exp}(\boldsymbol{\varphi}) := Exp(
    \vec{\boldsymbol{\varphi}}_{1}\varphi_{1},
    \vec{\boldsymbol{\varphi}}_{2}\varphi_{2},
    \vec{\boldsymbol{\varphi}}_{3}\varphi_{3}
  )

.. math::
  Exp((t + s) \vec{\boldsymbol{\varphi}}) =
    Exp(t\vec{\boldsymbol{\varphi}}) \circ Exp(s\vec{\boldsymbol{\varphi}})
  \qquad
  \forall t, s \in \real, \forall \vec{\boldsymbol{\varphi}} \in

.. math::
  \boxplus :& \SO{3} \times \Real{3} \rightarrow \SO{3}, \\
    &\Phi, \boldsymbol{\varphi}
      \mapsto \text{exp}(\boldsymbol{\varphi}) \circ \Phi, \\
  \boxminus :& \SO{3} \times \SO{3} \rightarrow \Real{3}, \\
    &\Phi_1, \Phi_2 \mapsto \text{log}(\Phi_1 \circ \Phi_{2}^{-1})

Similar to regular addition and subtraction, both operators have the following
identities,

.. math::
  \Phi \boxplus \Vec{0} &= \Phi \\
  (\Phi \boxplus \boldsymbol{\varphi}) \boxminus \Phi &= \boldsymbol{\varphi} \\
  \Phi_1 \boxplus (\Phi_2 \boxminus \Phi_1) &= \Phi_2



Special Orthogonal Group :math:`\SO{3}`
---------------------------------------

Special Orthogonal Group :math:`\SO{3}` describes the group of 3D rotation
matrices and it is formally defined as :math:`\SO{3} \dot{=} \{ \rot \in
\real^{3 \times 3} : \rot^{\transpose} \rot = \I, \Det{\rot} = 1 \}` .  The
group operation is the usual matrix multiplication, and the inverse is the
matrix transpose. The group :math:`\SO{3}` also forms a smooth manifold. The
tangent space to the manifold (at the identity) is denoted :math:`\so(3)`,
which is also called the *Lie algebra* and coincides with the space of `3
\times 3` skew symmetric matrices.

.. math::
  \angvel \in \real^{3},
  \enspace
  \Skew{\angvel} = \begin{bmatrix}
    0 & -\omega_{3} & \omega_{2} \\
    \omega_{3} & 0 & -\omega_{1} \\
    -\omega_{2} & \omega_{1} & 0
  \end{bmatrix}


.. math::
   \text{Exp}: \real^{3} \rightarrow \SO{3} ;
   \enspace
   \boldsymbol{\phi} \rightarrow \text{exp}(\Skew{\boldsymbol{\phi}}) \\
   \text{Log}: \SO{3} \rightarrow \real^{3} ;
   \enspace
   \rot \rightarrow \Skew{\text{log}(\rot)}


Exponential Map
^^^^^^^^^^^^^^^

The *exponential map* (at the identity) :math:`\text{exp}: \mathfrak{so}(3)
\rightarrow \SO{3}` associates an element of the Lie Algebra to a rotation:

.. math::

  \text{exp}(\Skew{\boldsymbol{\phi}}) =
    \I
    + \dfrac{\sin(|| \boldsymbol{\phi} ||)}
            {|| \boldsymbol{\phi} ||}
        \Skew{\boldsymbol{\phi}}
    + \dfrac{1 - \cos(|| \boldsymbol{\phi} ||)}
            {|| \boldsymbol{\phi} ||^{2}}
        \Skew{\boldsymbol{\phi}}^{2}


Logarithmic Map
^^^^^^^^^^^^^^^

The *logarithmic map* (at the identity) associates rotation matrix
:math:`\rot \in \SO{3}` to a skew symmetric matrix:

.. math::

  \log(\rot) = \dfrac{\psi \cdot (\rot - \rot^{\transpose})}{2 \sin(\psi)},
  \enspace
  \psi = \cos^{-1} \left( \dfrac{\Trace{\rot) - 1}}{2} \right)



Properties
^^^^^^^^^^

.. math::

  \begin{align}
    \rot \text{Exp}(\boldsymbol{\phi}) \rot^{\transpose}
    &= \text{Exp}(\rot \Skew{\boldsymbol{\phi}} \rot^{\transpose}) \\
    &= \text{Exp}(\rot \boldsymbol{\phi}) \\
    &= \text{Exp}(\boldsymbol{\phi}) \rot \\
    &= \rot^{\transpose} \text{Exp}(\boldsymbol{\phi}) \rot
  \end{align}

.. math::

  \text{Exp}(\boldsymbol{\phi} + \delta\boldsymbol{\phi}) \approx
    \text{Exp}(\boldsymbol{\phi})
    \text{Exp}(\jac_{r}(\boldsymbol{\phi}) \delta\boldsymbol{\phi})

.. math::

  \text{Log}(
    \text{Exp}(\boldsymbol{\phi})
    \text{Exp}(\delta\boldsymbol{\phi})
  )
  \approx
  \boldsymbol{\phi} + \jac_{r}^{-1}(\boldsymbol{\phi})
    \delta\boldsymbol{\phi}

.. math::

  \jac_{r}(\boldsymbol{\phi}) =
    \I
    - \dfrac{1 - \cos(||\boldsymbol{\phi}||)}{||\boldsymbol{\phi}||^{2}}
    + \dfrac{||\boldsymbol{\phi} - \sin(||\boldsymbol{\phi}||)}
      {||\boldsymbol{\phi^{3}}||}
    (\Skew{\boldsymbol{\phi}})^{2}

.. math::

  \jac_{r}^{-1}(\boldsymbol{\phi}) =
    \I
    - \dfrac{1}{2} \Skew{\boldsymbol{\phi}}
    + \left(
      \dfrac{1}{||\boldsymbol{\phi}||^{2}}
      + \dfrac{1 + \cos(||\boldsymbol{\phi}||}
        {2 ||\boldsymbol{\phi}||^{2} \sin(||\boldsymbol{\phi}||)}
    \right)
    \Skew{\boldsymbol{\phi}}^{2}

