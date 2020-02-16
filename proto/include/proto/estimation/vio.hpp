#ifndef PROTO_ESTIMATION_VIO_HPP
#define PROTO_ESTIMATION_VIO_HPP

#include "proto/core/core.hpp"

namespace proto {

struct imu_model_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int size = 15;  ///< Size of error-state vector
  bool rk4 = false;  ///< Runge-Kutta 4th order integration

  // State
  vec4_t q_IG = vec4_t{0.0, 0.0, 0.0, 1.0}; ///< JPL Quaternion in Global frame
  vec3_t b_g = zeros(3, 1);               ///< Bias of gyroscope
  vec3_t v_G = zeros(3, 1);               ///< Velocity in Global frame
  vec3_t b_a = zeros(3, 1);               ///< Bias of accelerometer
  vec3_t p_G = zeros(3, 1);               ///< Position in Global frame

  // Extrinsics
  // -- IMU-Camera extrinsics
  vec3_t p_IC = zeros(3, 1);
  vec4_t q_IC = zeros(4, 1);
  // -- Gimbal joint angle extrinsics
  vec2_t theta = zeros(2, 1);

  // Constants
  vec3_t g_G = vec3_t{0.0, 0.0, -9.81}; ///< Gravitational acceleration

  // Misc
  matx_t P = 1e-5 * I(15); ///< Covariance matrix
  matx_t Q = 1e-2 * I(12); ///< Noise matrix
  matx_t Phi = I(15); ///< Noise matrix

  imu_model_t();
};

/**
	* Transition F matrix
	*
	* @param w_hat Estimated angular velocity
	* @param q_hat Estimated quaternion (x, y, z, w)
	* @param a_hat Estimated acceleration
	* @returns Transition jacobian matrix F
	*/
matx_t imu_model_F(imu_model_t &imu,
							 	   const vec3_t &w_hat,
									 const vec4_t &q_hat,
						 	 	   const vec3_t &a_hat);

/**
	* Input G matrix
	*
	* A matrix that maps the input vector (IMU gaussian noise) to the state
	* vector (IMU error state vector), it tells us how the inputs affect the
	* state vector.
	*
	* @param q_hat Estimated quaternion (x, y, z, w)
	* @returns Input jacobian matrix G
	*/
matx_t imu_model_G(imu_model_t &imu, const vec4_t &q_hat);

/**
	* Update
	*
	* @param a_m Measured acceleration
	* @param w_m Measured angular velocity
	* @param dt Time difference in seconds
	*/
void imu_model_update(imu_model_t &imu,
						 	 		    const vec3_t &a_m,
			 	 	 	 	 	 		  const vec3_t &w_m,
			                const double dt);

/**
	* Correct the IMU state
	*
	* @param dx Correction state vector
	*/
void imu_model_correct(imu_model_t &imu, const vecx_t &dx);

} // namespace proto
#endif // PROTO_ESTIMATION_VIO_HPP
