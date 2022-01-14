# Deep Learning Methods on Image Blur Detection

- Learning to Understand Image Blur, Zhang et al (2018)

Dilated fully convolutional neural network with pyramid pooling and boundary
refinement layers to geterate high-quality blur response maps.

Outputs three classes, "Good", "OK" and "bad"


- Burst Image Deblurring Using Permutation Invariant Convolutional Neural
  Networks, Aittala and Durand (2018)

Considers all the frames simultaneously as an unordered set of arbitrary size.
The key idea is to enforce *permutation invariance* by construction: when the
ordering of the frames cannot affect the output, no frame is in a special
position in relation to others, and consequently each one receives the same
consideration. Any piece of useful information can directly influsence the
solution, and subtle cues scattered around in the burst can be combined
effectively. The approach is similar in spirit to classical maximum likelihood
or Bayesian inference, where contriutions from each observations are
symmetrically accumulated onto a likelihood function from which the desired
estimate is then derived.
