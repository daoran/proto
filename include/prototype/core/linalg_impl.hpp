#ifndef PROTOTYPE_CORE_LINALG_IMPL_HPP
#define PROTOTYPE_CORE_LINALG_IMPL_HPP

namespace prototype {

template <typename T>
Eigen::Matrix<T, 4, 4> transform(const Eigen::Matrix<T, 3, 3> &R,
                                 const Eigen::Matrix<T, 3, 1> &t) {
  Eigen::Matrix<T, 4, 4> tf = Eigen::Matrix<T, 4, 4>::Identity();
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = t;
  return tf;
}

}
#endif // PROTOTYPE_CORE_LINALG_IMPL_HPP
