#set page(columns: 2, margin: 0.5in)
#set text(
  font: "New Computer Modern",
  size: 9pt
)

= Marginalization

As a reminder, marginalization is about having a joint density $p(x, y)$ over
two variables $x$ and $y$, and we would like to marginalize out or "eliminate a
variable", lets say $y$ in this case:

$
  p(x) = integral_y p(x, y)
$

resulting in a density $p(x)$ over the remaining variable $x$.

Now, if the density was in covariance form with mean $mu$
and covariance $Sigma$, partitioned as follows:

$
  p(x, y) = cal(N)(
    vec(mu_x, mu_y),
    mat(
      Sigma_"xx", Sigma_"xy";
      Sigma_"yx", Sigma_"yy"
    )
  )
$

marginalization is simple, as the corresponding sub-block $Sigma_"xx"$ already
contains the covariance on $x$ i.e.,

$
  p(x) = cal(N)(mu_x, Sigma_"xx").
$

In the nonlinear-least squares formulation, however, we can only obtain the
covariance $Sigma$ with the following property,

$
  Sigma = H^(-1)
$

where $H$ is the Hessian matrix in a Gauss-Newton system
($H delta x = b$).


== Using Shur's Complement for marginalization

First let $x_1$ be the states to be marginalized out, $x_"2"$ be the
set of states related to those by error terms, and $x_rho$ be the set
of remaining states. Partitioning the Hessian, error state and R.H.S of the
Gauss-Newton system gives:

$
  mat(
    H_"11", H_"12";
    H_"21", H_"22"
  )
  vec(delta x_1, delta x_2)
  =
  vec(b_1, b_2)
$

and applying the Shur complement operation yields:

$
  H^(ast)_"22" &= H_"22" - H_"21" H_"11"^(-1) H_"12" \
  b^(ast)_"2" &= b_"2" - H_"21" H_"11"^(-1) b_"1"
$

where $b^(ast)_"2"$ and $H^(ast)_"22"$ are non-linear functions of $x_2$ and
$x_1$.


== Derivation of Schur's Complement for Marginalization

From the Gauss-Newton system, $H delta x = b$, we can
derive the marginalization of the old states in $delta x$ algebraically.
Let us decompose the system as:

$
  mat(H_"11", H_"12"; H_"21", H_"22")
  vec(delta x_"1", delta x_"2")
  =
  vec(b_"1", b_"2")
$

If we multiply out the block matrices and vectors out we get:

$
  H_"11" delta x_"1" + H_"12"  delta x_"2" = b_"1" \
  H_"21" delta x_"1" + H_"22" delta x_"2" = b_"2"
$

Suppose we want to marginalize out the $delta x_"2"$, the second
equation above can be rearranged w.r.t. $delta x_"2"$ like so:

$
  H_"21" delta x_"1" + H_"22" delta x_"2" &= b_"2" \
  H_"22" delta x_"2" &= b_"2" - H_"21" delta x_"1" \
  delta x_"2" &= H_"22"^(-1) b_"2" - H_"22"^(-1) H_"21" delta x_"1"
$

substituting $delta x_"2"$ back into $H_"11" delta x_"1" +
H_"12"  delta x_"2" = b_"1"$, and rearranging the terms so it
is w.r.t $delta x_"1"$ to get:

$
  H_"11" delta x_"1" + H_"12"
    (H_"22"^(-1) b_"2"
    - H_"22"^(-1) H_"21" delta x_"1")
    &= b_"1" \
  H_"11" delta x_"1"
    + H_"12"  H_"22"^(-1) b_"2"
    - H_"12"  H_"22"^(-1) H_"21" delta x_"1"
    &= b_"1" \
  (H_"11" - H_"12" H_"22"^(-1)H_"21") delta x_"1"
    &= b_"1" - H_"12"  H_"22"^(-1) b_"2"
$

thus the Schur Complement of $H_"22"$ in $H$ is:

$
  H slash H_"22" &= H_"11" - H_"12"  H_"22"^(-1) H_"21" \
  b slash b_"2" &= b_"1" - H_"12"  H_"22"^(-1) b_"2"
$

If you want to marginalize out $delta x_"1"$ you can follow the same
process above but w.r.t $x_"1"$.


== First Estimate Jacobians (FEJ)

In the context of real time state-estimation, a fixed-lag smoother provides a
way to bound the optimization problem in order to operate in real time. The
method to remove old states in the state vector is called marginalization. To
perform marginalization the Schur's complement is used to marginalize out old
states. Simply performing marginalization, however, introduces estimator
inconsistencies.

Let us consider the following scenario. A state vector, $x$, during the
time interval $[0, \, k]$ will contain $m$ old states to be marginalized
out and $r$ remain states which we wish to keep. i.e. $x = [x_m^T
quad x_r^T]^T$. Then the cost function, $c(dot)$, can be written
as a function of $x$ at time $k$ as,

$
  c(x_{k}) &= c(x_m, x_r) \
           &= c(x_m) + c(x_r).
$

The intuition behind the above is since the state at time $k$ can be
partitioned into $m$ and $r$, the cost can also be decomposed. Utilizing this
property, the multivariate optimization can also be decomposed as follows,

$
  min_(x_m, x_r) c(x_m, x_r)
    &= min_(x_r) (min_(x_m) c(x_m, x_r)) \
    &= min_(x_r) (c(x_r) + min_(x_m) c(x_m)).
$

The equation above shows the minimization problem can be solved by first
optimizing for the states $x_m$, and then forming a prior towards
the problem of solving for $x_r$. The reformulation of the
minimization problem entails no approximation.
