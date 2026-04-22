/**
 * @file xqpower_can_params.c
 * @brief XQPOWER CAN Servo Parameters
 */

/**
 * XQPOWER CAN Enable
 *
 * 0 = Disabled, 1 = Enabled
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 * @reboot_required true
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_ENABLE, 1);

/**
 * Servo 1 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE1, 1);

/**
 * Servo 2 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE2, 2);

/**
 * Servo 3 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE3, 3);

/**
 * Servo 4 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE4, 4);

/**
 * Servo angle limit (degrees)
 *
 * Maximum deflection angle in degrees. Servos will be clamped to ±limit.
 * XQPOWER supports ±100 deg.
 *
 * Must match the acados solver's delta_max (=20°) so that normalized
 * actuator_servos commands (fin / delta_max) scale back to the
 * intended physical angle. See m130_ocp_setup.py::delta_max.
 *
 * @min 1.0
 * @max 100.0
 * @unit deg
 * @decimal 1
 * @group XQPOWER CAN
 */
PARAM_DEFINE_FLOAT(XQCAN_LIMIT, 20.0f);

/**
 * Servo direction reverse bitmask
 *
 * Each bit reverses the corresponding servo direction.
 * bit0=servo0, bit1=servo1, bit2=servo2, bit3=servo3.
 * Example: 12 (0b1100) reverses servos 2 and 3.
 *
 * @min 0
 * @max 15
 * @bit 0 Reverse servo 0
 * @bit 1 Reverse servo 1
 * @bit 2 Reverse servo 2
 * @bit 3 Reverse servo 3
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_REV, 0);
