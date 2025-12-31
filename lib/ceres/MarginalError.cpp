#include "MarginalError.hpp"

namespace cartesian {

MarginalError::MarginalError() : ResidualBlock{"MarginalError"} {}

size_t MarginalError::get_residual_size() const { return r_; }

void MarginalError::set_residual_size(size_t size) { set_num_residuals(size); }

std::vector<ParamBlock *> MarginalError::get_params() {
  return remain_param_ptrs_;
}

std::vector<double *> MarginalError::get_param_ptrs() {
  std::vector<double *> params;
  for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
    params.push_back(remain_param_ptrs_[i]->param.data());
  }
  return params;
}

void MarginalError::add(std::shared_ptr<ResidualBlock> res_block) {
  // Check pointer to residual block
  if (res_block == nullptr) {
    FATAL("res_block == nullptr!");
  }
  res_blocks_.push_back(res_block);
}

void MarginalError::add_remain_param(ParamBlock *param) {
  param_blocks.push_back(param);
  remain_param_ptrs_.push_back(param);
  mutable_parameter_block_sizes()->push_back(param->global_size);
}

void MarginalError::form_hessian(matx_t &H, vecx_t &b) {
  // Reset marginalization error parameter blocks and residual size
  param_blocks.clear(); // <- ResidualBlock::param_blocks
  mutable_parameter_block_sizes()->clear();
  set_num_residuals(0);

  // Track parameter blocks
  std::map<ParamBlock *, bool> params_seem;
  std::vector<ParamBlock *> remain_pose_param_ptrs;
  std::vector<ParamBlock *> remain_sb_param_ptrs;
  std::vector<ParamBlock *> remain_camera_param_ptrs;
  std::vector<ParamBlock *> remain_extrinsics_ptrs;
  std::vector<ParamBlock *> remain_fiducial_ptrs;
  for (auto res_block : res_blocks_) {
    for (auto param_block : res_block->param_blocks) {
      if (param_block == nullptr) {
        FATAL("Param block is NULL! Implementation Error!");
      }

      // Seen parameter block already
      if (params_seem.count(param_block)) {
        continue;
      }

      // Keep track of parameter block
      if (param_block->marginalize) {
        marg_param_ptrs_.push_back(param_block);
      } else if (param_block->type == "pose_t") {
        remain_pose_param_ptrs.push_back(param_block);
      } else if (param_block->type == "sb_params_t") {
        remain_sb_param_ptrs.push_back(param_block);
      } else if (param_block->type == "camera_params_t") {
        remain_camera_param_ptrs.push_back(param_block);
      } else if (param_block->type == "extrinsics_t") {
        remain_extrinsics_ptrs.push_back(param_block);
      } else if (param_block->type == "fiducial_t") {
        remain_fiducial_ptrs.push_back(param_block);
      }
      params_seem[param_block] = true;
    }
  }

  // Determine parameter block column indicies for Hessian matrix H
  size_t H_idx = 0; // Column / row index of Hessian matrix H
  // -- Column indices for parameter blocks to be marginalized
  for (const auto &param_block : marg_param_ptrs_) {
    param_index_.insert({param_block, H_idx});
    H_idx += param_block->local_size;
    m_ += param_block->local_size;
  }
  // -- Column indices for parameter blocks to remain
  std::vector<std::vector<ParamBlock *> *> remain_params = {
      &remain_pose_param_ptrs,
      &remain_sb_param_ptrs,
      &remain_fiducial_ptrs,
      &remain_extrinsics_ptrs,
      &remain_camera_param_ptrs,
  };
  for (const auto &param_ptrs : remain_params) {
    for (const auto &param_block : *param_ptrs) {
      if (param_block->fixed) {
        continue;
      }
      param_index_.insert({param_block, H_idx});
      H_idx += param_block->local_size;
      r_ += param_block->local_size;
      remain_param_ptrs_.push_back(param_block);
      param_blocks.push_back(param_block); // <- ResidualBlock::param_blocks

      // VERY IMPORTANT!! Update param blocks and sizes for ceres::CostFunction
      mutable_parameter_block_sizes()->push_back(param_block->global_size);
    }
  }

  // !! VERY IMPORTANT !! Now we know the Hessian size, we can update the
  // number of residuals for ceres::CostFunction
  set_num_residuals(r_);

  // Allocate memory for residuals and jacobians
  residuals.resize(r_);
  for (auto param : param_blocks) {
    jacobian_blocks.push_back(zeros(r_, param->global_size));
    min_jacobian_blocks.push_back(zeros(r_, param->local_size));
  }

  // Form the H and b. Left and RHS of Gauss-Newton.
  // H = J.T * J
  // b = -J.T * e
  const auto local_size = m_ + r_;
  H = zeros(local_size, local_size);
  b = zeros(local_size, 1);

  for (auto res_block : res_blocks_) {
    // Setup parameter data
    std::vector<double *> param_ptrs;
    for (auto param_block : res_block->param_blocks) {
      param_ptrs.push_back(param_block->data());
    }

    // Setup Jacobians data
    const int r_size = res_block->num_residuals();
    std::vector<matx_row_major_t> jacs;
    std::vector<double *> jac_ptrs;
    for (auto param_block : res_block->param_blocks) {
      jacs.push_back(zeros(r_size, param_block->global_size));
      jac_ptrs.push_back(jacs.back().data());
    }

    // Setup Min-Jacobians data
    std::vector<matx_row_major_t> min_jacs;
    std::vector<double *> min_jac_ptrs;
    for (auto param_block : res_block->param_blocks) {
      min_jacs.push_back(zeros(r_size, param_block->local_size));
      min_jac_ptrs.push_back(min_jacs.back().data());
    }

    // Evaluate residual block
    vecx_t r = zeros(r_size, 1);
    res_block->EvaluateWithMinimalJacobians(param_ptrs.data(),
                                            r.data(),
                                            jac_ptrs.data(),
                                            min_jac_ptrs.data());

    // Fill Hessian
    for (size_t i = 0; i < res_block->param_blocks.size(); i++) {
      const auto &param_i = res_block->param_blocks[i];
      if (param_i->fixed) {
        continue;
      }
      const int idx_i = param_index_[param_i];
      const int size_i = param_i->local_size;
      const matx_t J_i = min_jacs[i];

      for (size_t j = i; j < res_block->param_blocks.size(); j++) {
        const auto &param_j = res_block->param_blocks[j];
        if (param_j->fixed) {
          continue;
        }
        const int idx_j = param_index_[param_j];
        const int size_j = param_j->local_size;
        const matx_t J_j = min_jacs[j];

        if (i == j) {
          // Form diagonals of H
          H.block(idx_i, idx_i, size_i, size_i) += J_i.transpose() * J_i;
        } else {
          // Form off-diagonals of H
          // clang-format off
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) += (J_i.transpose() * J_j).transpose();
          // clang-format on
        }
      }

      // RHS of Gauss Newton (i.e. vector b)
      b.segment(idx_i, size_i) += -J_i.transpose() * r;
    }
  }
}

void MarginalError::schurs_complement(const matx_t &H,
                                      const vecx_t &b,
                                      const size_t m,
                                      const size_t r,
                                      matx_t &H_marg,
                                      vecx_t &b_marg,
                                      const bool debug) {
  assert(m > 0 && r > 0);

  // Setup
  const long local_size = m + r;
  H_marg = zeros(local_size, local_size);
  b_marg = zeros(local_size, 1);

  // Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal
  // entry by its reciprocal, leaving the zeros in place, and transposing
  // the resulting matrix.
  // clang-format off
  matx_t Hmm = H.block(0, 0, m, m);
  Hmm = 0.5 * (Hmm + Hmm.transpose()); // Enforce Symmetry
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals_inv = (eig.eigenvalues().array() > eps).select(eig.eigenvalues().array().inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  if (debug) {
    const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
    if (fabs(inv_check) > 1e-4) {
      LOG_WARN("Inverse identity check: %f", inv_check);
      LOG_WARN("This is bad ... Usually means MarginalError is bad!");

      printf("nb_res_blocks: %ld\n", res_blocks_.size());
      printf("nb_param_blocks: %ld\n", remain_param_ptrs_.size());
      print_matrix("Hmm", Hmm);
      print_matrix("Hmm_inv", Hmm_inv);

    }
  }
  // clang-format on

  // Calculate Schur's complement
  // H = [Hmm, Hmr,
  //      Hrm, Hrr]
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const vecx_t bmm = b.segment(0, m);
  const vecx_t brr = b.segment(m, r);
  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  b_marg = brr - Hrm * Hmm_inv * bmm;
}

void MarginalError::marginalize(
    std::vector<ParamBlock *> &marg_params,
    std::vector<std::shared_ptr<ResidualBlock>> &marg_residuals,
    const bool debug) {
  // Form Hessian and RHS of Gauss newton
  matx_t H;
  vecx_t b;
  form_hessian(H, b);
  if (debug) {
    mat2csv("/tmp/H.csv", H);
    mat2csv("/tmp/b.csv", b);
  }

  // Compute Schurs Complement
  matx_t H_marg;
  vecx_t b_marg;
  schurs_complement(H, b, m_, r_, H_marg, b_marg);
  if (debug) {
    mat2csv("/tmp/H_marg.csv", H_marg);
    mat2csv("/tmp/b_marg.csv", b_marg);
  }

  // Decompose matrix H into J' * J to obtain J via eigen-decomposition
  // i.e.
  //
  //   H = J' * J = U * S * U'
  //   J = S^{0.5} * U'
  //
  // clang-format off
  // H_marg = 0.5 * (H_marg + H_marg.transpose()); // Enforce Symmetry
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(H_marg);
  const double eps = std::numeric_limits<double>::epsilon();
  const double tol = eps * H_marg.cols() * eig.eigenvalues().array().maxCoeff();
  const vecx_t S = (eig.eigenvalues().array() > tol).select(eig.eigenvalues().array(), 0.0);
  const vecx_t S_inv = (eig.eigenvalues().array() > tol).select(eig.eigenvalues().array().inverse(), 0);
  const vecx_t S_sqrt = S.cwiseSqrt();
  const vecx_t S_inv_sqrt = S_inv.cwiseSqrt();
  const matx_t J = S_sqrt.asDiagonal() * eig.eigenvectors().transpose();
  const matx_t J_inv = S_inv_sqrt.asDiagonal() * eig.eigenvectors().transpose();
  // -- Check decomposition
  const real_t decomp_norm = ((J.transpose() * J) - H_marg).norm();
  const bool decomp_check = decomp_norm < 1.0e-2;
  if (decomp_check == false) {
    LOG_WARN("Decompose JtJ check: %f", decomp_norm);
    LOG_WARN("This is bad ... Usually means MarginalError is bad!");
  }
  // clang-format on

  // Form:
  // - Linearized jacobians
  // - Linearized residuals
  // - Linearization point x0
  J0_ = J;
  r0_ = -J_inv * b_marg;
  for (const auto &param_block : remain_param_ptrs_) {
    x0_.insert({param_block->param.data(), param_block->param});
  }
  if (debug) {
    mat2csv("/tmp/J0.csv", J0_);
    mat2csv("/tmp/r0.csv", r0_);
  }

  // Copy parameters and residuals to be removed
  // IMPORTANT NOTE: By doing this, this transfers ownership out of
  // MarginalError. This means you have to free it yourself outside of
  // MarginalError.
  marg_params = marg_param_ptrs_;
  marg_residuals = res_blocks_;
  // Clear the pointers in the member variables
  marg_param_ptrs_.clear();
  res_blocks_.clear();

  // Update state
  marginalized_ = true;
}

ceres::ResidualBlockId MarginalError::marginalize(ceres::Problem *problem) {
  // Marginalize
  std::vector<ParamBlock *> marg_params;
  std::vector<std::shared_ptr<ResidualBlock>> marg_residuals;
  marginalize(marg_params, marg_residuals);

  // Remove parameter blocks from problem, which in turn ceres will remove
  // residual blocks linked to the parameter.
  for (const auto &marg_param : marg_params) {
    if (problem->HasParameterBlock(marg_param->param.data())) {
      problem->RemoveParameterBlock(marg_param->param.data());
    }
  }

  // // Delete residual blocks - no longer needed
  // for (auto res_block : marg_residuals) {
  //   delete res_block;
  // }

  // Change flag to denote marginalized and add it self to problem
  marginalized_ = true;
  return problem->AddResidualBlock(this, NULL, get_param_ptrs());
}

vecx_t MarginalError::compute_delta_chi(double const *const *params) const {
  // Pre-check
  if (marginalized_ == false) {
    FATAL("Implementation Error! MarginalError not marginalized yet!");
  }

  // Check if parameter is a pose
  auto is_pose = [](const ParamBlock *param) {
    if (param->type == "pose_t") {
      return true;
    } else if (param->type == "extrinsics_t") {
      return true;
    } else if (param->type == "fiducial_t" && param->global_size == 7) {
      return true;
    }
    return false;
  };

  // Stack DeltaChi vector
  vecx_t DeltaChi(r0_.size());
  for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
    const auto param_block = remain_param_ptrs_[i];
    const auto idx = param_index_.at(param_block) - m_;
    const vecx_t x0_i = x0_.at(param_block->param.data());
    const size_t size = param_block->global_size;
    const Eigen::Map<const vecx_t> x(params[i], size);

    // Calculate i-th DeltaChi
    if (is_pose(param_block)) {
      // Pose minus
      const vec3_t dr = x.head<3>() - x0_i.head<3>();
      const quat_t q_i(x(6), x(3), x(4), x(5));
      const quat_t q_j(x0_i(6), x0_i(3), x0_i(4), x0_i(5));
      const quat_t dq = q_i * q_j.inverse();
      DeltaChi.segment<3>(idx + 0) = dr;
      DeltaChi.segment<3>(idx + 3) = 2.0 * dq.vec();
    } else {
      // Trivial minus
      DeltaChi.segment(idx, size) = x - x0_i;
    }
  }

  return DeltaChi;
}

bool MarginalError::EvaluateWithMinimalJacobians(double const *const *params,
                                                 double *res,
                                                 double **jacs,
                                                 double **min_jacs) const {
  // Pre-check
  if (marginalized_ == false) {
    FATAL("Implementation Error! MarginalError not marginalized yet!");
  }

  // Residuals
  const vecx_t Delta_Chi = compute_delta_chi(params);
  Eigen::Map<vecx_t> e(res, r0_.rows());
  e = r0_ + J0_ * Delta_Chi;

  // Return First Estimate Jacobians (FEJ)
  if (jacs == nullptr) {
    return true;
  }

  const size_t J_rows = r0_.rows();
  for (size_t i = 0; i < remain_param_ptrs_.size(); i++) {
    if (jacs[i] != nullptr) {
      const auto &param = remain_param_ptrs_[i];
      const size_t index = param_index_.at(param) - m_;
      const size_t J_cols = param->global_size;
      const matx_t J_min = J0_.middleCols(index, param->local_size);

      // Global-Jacobian
      Eigen::Map<matx_row_major_t> J(jacs[i], J_rows, J_cols);
      if (param->global_size == param->local_size) {
        J = J_min;
      } else if (param->local_size == 6) {
        // We can assume this parameter is a pose / extrinsics parameter
        const mat4_t pose = tf(param->param);
        J = J_min * lift_pose_jacobian(pose);
      }

      // Local-Jacobian
      if (min_jacs && min_jacs[i]) {
        Eigen::Map<matx_row_major_t> min_J(min_jacs[i],
                                           J_min.rows(),
                                           J_min.cols());
        min_J = J_min;
      }
    }
  }

  return true;
}

} // namespace cartesian
