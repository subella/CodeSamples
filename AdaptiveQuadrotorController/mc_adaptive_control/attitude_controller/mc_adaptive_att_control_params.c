/**
 * @file mc_geometric_control_params.c
 * Parameters for the multicopter geometric controller
 *
 * @author Nathan Hughes <nathan.h.hughes@gmail.com>
 * @author Samuel Ubellacker <subella@mit.edu>
 *
 */

/**
 * Adaptive weight update enable
 *
 * Low-pass filter the angular velocity
 *
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_INT32(AMC_FILT_ANGV, 1);

/**
 * Adaptive weight update enable
 *
 * LPF cuttoff frequency
 *
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_FILT_FREQ, 120.0f);

/**
 * Adaptive term enable
 *
 * Use computed adaptive terms in control law
 *
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_INT32(AMC_ADAP_EN, 0);

/**
 * Moment scaling
 *
 * Scaling parameter to bring moments into mixer range
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_XY_MOM_SCL, 0.1f);

/**
 * Moment scaling
 *
 * Scaling parameter to bring moments into mixer range
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_Z_MOM_SCL, 0.1f);

/**
 * Thrust scaling
 *
 * Scaling parameter to bring thrust into mixer range
 *
 * Default puts max thrust at about 4.5g
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_THRUST_SCL, 0.05f);

/**
 * K_rot
 *
 * Rotation gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_KROT, 2.0f);

/**
 * K_omega
 *
 * Angular velocity gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_KOMEGA, 0.6f);

/**
 * C2
 *
 * Adaptive c2 gain
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_C2, 2.0f);

/**
 * gamma_r
 *
 * Adaptive learing rate
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_GAMMA_R, 0.1f);

/**
 * Adaptive Limit
 *
 * Limit of the adaptive term
 *
 * @decimal 4
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_ADAP_LIM, 0.5f);

/**
 * Mass
 *
 * Estimated mass of the multicopter in grams
 *
 * @unit kg
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_MASS, 1.3f);

/**
 * Moment of inertia: x-axis
 *
 * X-axis of moment of inertia
 *
 * @unit kg * m^2
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_JXX, 0.005f);

/**
 * Moment of inertia: y-axis
 *
 * y-axis of moment of inertia
 *
 * @unit kg * m^2
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_JYY, 0.005f);

/**
 * Moment of inertia: z-axis
 *
 * z-axis of moment of inertia
 *
 * @unit kg * m^2
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Geometric Control
 */
PARAM_DEFINE_FLOAT(AMC_JZZ, 0.01f);
