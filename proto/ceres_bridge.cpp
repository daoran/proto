#include "ceres_bridge.h"

#include <iostream>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>

class CostFunctionWrapper final : public ceres::CostFunction {
public:
  CostFunctionWrapper(ceres_cost_function_t cost_function,
                      void *user_data,
                      int num_residuals,
                      int num_parameter_blocks,
                      int *parameter_block_sizes)
      : cost_function_(cost_function), user_data_(user_data) {
    set_num_residuals(num_residuals);
    for (int i = 0; i < num_parameter_blocks; ++i) {
      mutable_parameter_block_sizes()->push_back(parameter_block_sizes[i]);
    }
  }

  bool Evaluate(double const *const *parameters,
                double *residuals,
                double **jacobians) const final {
    return (*cost_function_)(user_data_,
                             const_cast<double **>(parameters),
                             residuals,
                             jacobians);
  }

private:
  ceres_cost_function_t cost_function_;
  void *user_data_;
};

class LossFunctionWrapper final : public ceres::LossFunction {
public:
  explicit LossFunctionWrapper(ceres_loss_function_t loss_function,
                               void *user_data)
      : loss_function_(loss_function), user_data_(user_data) {
  }
  void Evaluate(double sq_norm, double *rho) const final {
    (*loss_function_)(user_data_, sq_norm, rho);
  }

private:
  ceres_loss_function_t loss_function_;
  void *user_data_;
};

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x,
                    const double *dx,
                    double *x_plus_dx) const {
    x_plus_dx[0] = x[0] + dx[0];
    x_plus_dx[1] = x[1] + dx[1];
    x_plus_dx[2] = x[2] + dx[2];

    const double q[4] = {x[3], x[4], x[5], x[6]};
    const double dq[4] = {1.0, 0.5 * dx[3], 0.5 * dx[4], 0.5 * dx[5]};
    x_plus_dx[3] = q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3];
    x_plus_dx[4] = q[1] * dq[0] + q[0] * dq[1] - q[3] * dq[2] + q[2] * dq[3];
    x_plus_dx[5] = q[2] * dq[0] + q[3] * dq[1] + q[0] * dq[2] - q[1] * dq[3];
    x_plus_dx[6] = q[3] * dq[0] - q[2] * dq[1] + q[1] * dq[2] + q[0] * dq[3];

    return true;
  }

  virtual bool ComputeJacobian(const double *x, double *J) const {
    // Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    // j.topRows<6>().setIdentity();
    // j.bottomRows<1>().setZero();
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 6; j++) {
        J[(i * 6) + j] = (i == j) ? 1.0 : 0.0;
      }
    }

    return true;
  }

  virtual int GlobalSize() const {
    return 7;
  }

  virtual int LocalSize() const {
    return 6;
  }
};

void ceres_init() {
  // This is not ideal, but it's not clear what to do if there is no gflags and
  // no access to command line arguments.
  char message[] = "<unknown>";
  google::InitGoogleLogging(message);
}

ceres_problem_t *ceres_create_problem() {
  return reinterpret_cast<ceres_problem_t *>(new ceres::Problem);
}

void ceres_free_problem(ceres_problem_t *problem) {
  delete reinterpret_cast<ceres::Problem *>(problem);
}

ceres_residual_block_id_t *
ceres_problem_add_residual_block(ceres_problem_t *problem,
                                 ceres_cost_function_t cost_function,
                                 void *cost_function_data,
                                 ceres_loss_function_t loss_function,
                                 void *loss_function_data,
                                 int num_residuals,
                                 int num_parameter_blocks,
                                 int *parameter_block_sizes,
                                 double **parameters) {
  auto *ceres_problem = reinterpret_cast<ceres::Problem *>(problem);

  auto callback_cost_function =
      std::make_unique<CostFunctionWrapper>(cost_function,
                                            cost_function_data,
                                            num_residuals,
                                            num_parameter_blocks,
                                            parameter_block_sizes);

  std::unique_ptr<ceres::LossFunction> callback_loss_function;
  if (loss_function != nullptr) {
    callback_loss_function =
        std::make_unique<LossFunctionWrapper>(loss_function,
                                              loss_function_data);
  }

  std::vector<double *> parameter_blocks(parameters,
                                         parameters + num_parameter_blocks);
  return reinterpret_cast<ceres_residual_block_id_t *>(
      ceres_problem->AddResidualBlock(callback_cost_function.release(),
                                      callback_loss_function.release(),
                                      parameter_blocks));
}

ceres_local_parameterization_t *ceres_create_pose_local_parameterization() {
  return reinterpret_cast<ceres_local_parameterization_t *>(
      new PoseLocalParameterization);
}

void ceres_free_local_parameterization(ceres_local_parameterization_t *p) {
  delete reinterpret_cast<ceres::LocalParameterization *>(p);
}

void ceres_set_parameterization(ceres_problem_t *c_problem,
                                double *values,
                                ceres_local_parameterization_t *c_local) {
  auto *problem = reinterpret_cast<ceres::Problem *>(c_problem);
  auto *local = reinterpret_cast<ceres::LocalParameterization *>(c_local);
  problem->SetParameterization(values, local);
}

void ceres_set_parameter_constant(ceres_problem_t *c_problem, double *values) {
  auto *problem = reinterpret_cast<ceres::Problem *>(c_problem);
  problem->SetParameterBlockConstant(values);
}

void ceres_solve(ceres_problem_t *c_problem, const int max_iter) {
  auto *problem = reinterpret_cast<ceres::Problem *>(c_problem);
  ceres::Solver::Options options;
  options.max_num_iterations = max_iter;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
