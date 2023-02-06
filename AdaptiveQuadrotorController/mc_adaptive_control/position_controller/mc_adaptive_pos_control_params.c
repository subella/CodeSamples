/**
 * @file mc_geometric_control_params.c
 * Parameters for the multicopter geometric controller
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 * @author Samuel Ubellacker <subella@mit.edu>
 *
 */

/**
 * log_enable
 *
 * Enable logging to console
 *
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_INT32(AMPC_LOG_ENABLE, 1);

/**
 * a_dot mode
 *
 * Determine how to compute a_dot
 */
PARAM_DEFINE_INT32(AMPC_ADOT_MODE, 0);

/**
 * Adaptive weight update enable
 *
 * Run adaptive weight update
 *
 * @group Multicopter Geometric Control
 */

PARAM_DEFINE_INT32(AMPC_ADAP_EN, 0);

/**
 * K_p
 *
 * Position gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_KP, 1.65f);

/**
 * K_v
 *
 * Linear velocity gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_KV, 0.8f);

/**
 * C1
 *
 * Adaptive c1 gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_C1, 2.0f);

/**
 * Adaptive Limit
 *
 * Limit of the adaptive term
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_ADAP_LIM, 5.0f);

/**
 * Adaptive projection bound
 *
 * Upper bound of disturbance
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_ADAP_B_TH_X, 10.0f);

/**
 * gamma_x
 *
 * Adaptive learning rate
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMPC_GAMMA_X, 8.0f);
