#pragma
#include "../core/Core.hpp"
#include "ParamBlock.hpp"
#include "ResidualBlock.hpp"

namespace cartesian {

class MarginalError : public ResidualBlock {
public:
  bool marginalized_ = false;

  // Residual blocks and parameters involved for marginalization
  size_t m_ = 0; // Size of params to marginalize
  size_t r_ = 0; // Size of params to remain
  std::vector<std::shared_ptr<ResidualBlock>> res_blocks_;
  std::vector<ParamBlock *> marg_param_ptrs_;
  std::vector<ParamBlock *> remain_param_ptrs_;
  std::unordered_map<ParamBlock *, int> param_index_;

  std::unordered_map<double *, VecX> x0_; // Linearization point x0
  VecX r0_;                               // Linearized residuals at x0
  MatX J0_;                               // Linearized jacobians at x0

  /* Constructor */
  MarginalError();

  /* Destructor */
  ~MarginalError() = default;

  /* Get Residual Size */
  size_t get_residual_size() const;

  /* Set Residual Size */
  void set_residual_size(size_t size);

  /* Get Parameters */
  std::vector<ParamBlock *> get_params();

  /* Get Parameter Pointers */
  std::vector<double *> get_param_ptrs();

  /* Add Cost Function */
  void add(std::shared_ptr<ResidualBlock> res_fn);

  /* Add Parameter block */
  void add_remain_param(ParamBlock *param);

  /* Form Hessian */
  void form_hessian(MatX &H, VecX &b);

  /* Schurs Complement */
  void schurs_complement(const MatX &H,
                         const VecX &b,
                         const size_t m,
                         const size_t r,
                         MatX &H_marg,
                         VecX &b_marg,
                         const bool debug = false);

  /* Marginalize */
  void marginalize(std::vector<ParamBlock *> &marg_params,
                   std::vector<std::shared_ptr<ResidualBlock>> &marg_residuals,
                   const bool debug = true);
  ceres::ResidualBlockId marginalize(ceres::Problem *problem);

  /* Compute Delta Chi */
  VecX compute_delta_chi(double const *const *params) const;

  /* Evaluate */
  bool EvaluateWithMinimalJacobians(double const *const *params,
                                    double *res,
                                    double **jacs,
                                    double **min_jacs) const;
};

} // namespace cartesian
