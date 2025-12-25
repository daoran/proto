#pragma once
#include <ceres/ceres.h>

#include "../core/Core.hpp"
#include "ParamBlock.hpp"

namespace xyz {

/** Residual Block */
class ResidualBlock : public ceres::CostFunction {
protected:
  // Data
  std::string type_;
  std::vector<double *> param_ptrs_;
  std::vector<ParamBlock::Type> param_types_;

public:
  /** Constructor */
  ResidualBlock() = delete;
  ResidualBlock(const std::string &type,
                const std::vector<double *> &param_ptrs,
                const std::vector<ParamBlock::Type> &param_types,
                const int num_residuals);

  /** Destructor */
  virtual ~ResidualBlock() = default;

  /** Get type */
  std::string getType() const;

  /** Get number of parameter blocks */
  int getNumParams() const;

  /** Get parameter block pointers */
  std::vector<double *> getParamPtrs() const;

  /** Evaluate with minimal jacobians */
  virtual bool eval(double const *const *params,
                    double *res,
                    double **jacs = nullptr) const = 0;

  /** Check jacobian */
  bool checkJacobian(const int param_idx,
                     const std::string &jac_name,
                     const double step = 1e-8,
                     const double tol = 1e-4,
                     const bool verbose = false) const;

  /** Ceres solver evaluate */
  bool Evaluate(double const *const *params,
                double *res,
                double **jacs = nullptr) const;
};

} // namespace xyz
