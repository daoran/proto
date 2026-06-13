#pragma once
#include "Core.hpp"

namespace cartesian {
namespace colormap {

/**
 * MATLAB-style jet colormap.
 *
 * Transitions: dark blue -> blue -> cyan -> green -> yellow -> red -> dark red
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i jet(const double t);

/**
 * Grayscale colormap.
 *
 * Transitions: black -> white
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i gray(const double t);

/**
 * Hot colormap, simulating blackbody radiation.
 *
 * Transitions: black -> red -> yellow -> white
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i hot(const double t);

/**
 * Cool colormap.
 *
 * Transitions: cyan -> magenta
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i cool(const double t);

/**
 * Turbo colormap, Google's perceptually improved jet replacement.
 *
 * Transitions: dark purple -> blue -> cyan -> green -> yellow ->
 * orange -> red -> dark red
 *
 * Reference:
 *   https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i turbo(const double t);

/**
 * Viridis colormap, the default perceptually uniform colormap from
 * matplotlib.
 *
 * Transitions: dark purple -> blue -> teal -> green -> yellow
 *
 * Reference: https://bids.github.io/colormap/
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i viridis(const double t);

/**
 * Inferno colormap, a perceptually uniform colormap from matplotlib.
 *
 * Transitions: black -> dark purple -> red -> orange -> pale yellow
 *
 * Reference: https://bids.github.io/colormap/
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i inferno(const double t);

/**
 * Plasma colormap, a perceptually uniform colormap from matplotlib.
 *
 * Transitions: dark purple -> purple -> red -> orange -> pale yellow
 *
 * Reference: https://bids.github.io/colormap/
 *
 * @param t Scalar in [0, 1].
 * @return  RGB triplet with each channel in [0, 255].
 */
Vec3i plasma(const double t);

} // namespace colormap
} // namespace cartesian
