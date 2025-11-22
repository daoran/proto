Feature Tracking
================

In this section we explain how feature points are detected and matched
between different camera frames. The common feature detection and matching
pipeline for localization and mapping algorithms is:

* Detect regions of interests (image feature) in the image
* Extract image feature information using descriptors
* Match extracted descriptors


FAST
----

Feature detection in computer vision is a process of gathering scene
information and deciding locally whether an image feature exists. The resulting
subset of image features in the image domain can in turn be used for
localization and mapping algorithms to estimate the camera pose.

For our requirements corners was the chosen image feature. The most widely used
corner detector is the FAST feature detector~\cite{Rosten2006}. The advantage
of using FAST includes its speed and high detection rate. It operates by
inspecting a gray-scale image and applying a Bresenham circle or patch of
configurable radius (radius of 3 for a 16 pixel circle in
Fig~\ref{fig:fast_corner}), where each pixel value on the circle is labeled
clockwise. If a set of :math:`N` contiguous pixels in the circle are all
brighter than the intensity of the center candidate pixel :math:`p` plus a
threshold value :math:`t`, or are all darker compared to :math:`p` minus a
threshold value :math:`t`, then :math:`p` is considered a corner.

.. image:: imgs/cv-fast.jpg
  :align: center

A uniform feature distribution over the image domain is known to avoid
degenerate configurations for SLAM, and reduce redundant information. Further,
a uniform and un-clustered corner distribution has the potential of increasing
computer vision pipeline efficiency, as a lower number of features are required
for the whole image. To encourage a uniform feature distribution a custom naive
implementation of Grid-FAST was implemented~\footnote{At the time of writing
OpenCV has removed the interface to the \texttt{GridAdaptedFeatureDetector}
implementation from their code base.}. The naive Grid-FAST was implemented as
follows, given an image we divide the image into :math:`r` rows and :math:`c` columns with
the goal of detecting a total max number of :math:`N` corners. The max number of
corners per grid cell :math:`n` is then given as

.. math::

    n = \dfrac{N}{r \times c}

Using :math:`n` we limit the corners detected in each image grid cell to
naively encourage a uniform distribution.

In Fig.~\ref{fig:grid_fast_comparison} both FAST and Grid-FAST observe the same
image scene with the same detection parameters. Grid-FAST divided the image
into 10 rows and columns to encourage a uniform corner detection. While
Grid-FAST detected a lower number of corners compared to FAST (714, 1000
respectively), we can observe the benefit of using Grid-FAST in
Fig.~\ref{subfig:fast_hist2d} and Fig.~\ref{subfig:grid_fast_hist2d}, where it
clearly shows that FAST detection has an undesirably high detection
concentration around the chessboard in this particular scene, Grid-FAST on the
other hand does not exhibit the same problem. Although, Grid-FAST obtains
features of lower quality in terms of repeatable detection, the threshold of
corner-ness can be increased if this is an issue.


Feature Descriptor
------------------

To correspond image features detected in two different image frames a feature
descriptor is used. Feature descriptors are a way to describe the image feature
observed for matching. There are a number of feature descriptors that extract
patch information in order to create a robust and repeatable match. Feature
descriptors such as SIFT~\cite{Lowe1999}, SURF~\cite{Bay2006}, are histogram of
gradients (HOG) based patch descriptors. These HOG descriptors are invariant to
small rotations and lighting variations, they are however, relatively expensive
to compute. The computationally expensive components are its calculation of the
image gradient and large descriptor dimension. While both descriptors provide
quality information of image features, the aforementioned computational factors
impact the matching speed significantly.

Binary descriptors such as BRIEF~\cite{Calonder2010}, ORB~\cite{Rublee2011} and
BRISK~\cite{Leutenegger2011} have been proposed to speed up the feature
descriptor and matching process. The performance boost in binary descriptors
comes in the form of using a binary sampling pattern around each image feature
previously detected (see Fig~\ref{fig:binary_descriptors}), and outputting a
binary vector, instead of computing image gradients and outputting a floating
point vector. Each binary descriptor uses its own unique sampling pattern, and
outputs a binary string to be used for matching. The matching process is
cheaper compared to the HOG based descriptors, because instead of comparing two
floating point vectors, comparing binary descriptors is performed by computing
the Hamming distance using a XOR or bit count operation, which can be performed
extremely quickly on modern CPUs~\cite{Calonder2012}.

.. figure:: imgs/cv-brief.png
  :align: center

  BRIEF Descriptor

.. figure:: imgs/cv-orb.png
  :align: center

  ORB Descriptor

.. figure:: imgs/cv-brisk.png
  :align: center

  BRISK Descriptor
