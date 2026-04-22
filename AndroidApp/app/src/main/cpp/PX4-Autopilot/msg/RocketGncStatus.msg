# Rocket GNC Status
# Published by rocket_mpc module for logging and QGroundControl telemetry.
#
# Field layout reflects the MPC+MHE pipeline.  The four legacy names
# wx_filter / du_roll / out_integ_roll / yaw_los_deg were retired in favour
# of the explicit mpc_solve_ms / mhe_quality / mpc_solve_count / blend_alpha
# fields below.  Consumers must migrate to the new names.

uint64 timestamp          # [us] Time since system start
uint64 timestamp_sample   # [us] Timestamp of input data

uint8 stage               # Flight stage (1=pre-launch, 2=in-flight)
float32 t_flight          # [s] Time since launch detection
float32 q_dyn             # [Pa] Dynamic pressure
float32 q_kgf             # [kgf/cm2] Dynamic pressure scaled

# --- Guidance references ---
float32 pitch_accel_cmd   # [rad] gamma_ref — flight-path angle reference from LOS guidance
float32 yaw_accel_cmd     # [rad] chi_ref — heading reference from LOS guidance

# --- Fin commands (channel-level, before X-mixer) ---
float32 delta_roll        # [rad] Aileron (roll channel) fin command (delta_a)
float32 delta_pitch       # [rad] Elevator (pitch channel) fin command (delta_e)
float32 delta_yaw         # [rad] Rudder (yaw channel) fin command (delta_r)

# --- Physical fin deflections (after X-mixer) ---
float32 fin1              # [rad] Fin 1 deflection
float32 fin2              # [rad] Fin 2 deflection
float32 fin3              # [rad] Fin 3 deflection
float32 fin4              # [rad] Fin 4 deflection

# --- State snapshot (published source follows EKF-MHE blend) ---
float32 altitude          # [m] Current altitude (MHE when authoritative, else EKF2)
float32 airspeed          # [m/s] Current airspeed (MHE when authoritative, else EKF2)
float32 rho               # [kg/m3] Air density

float32 phi               # [rad] Roll angle (MHE when authoritative, else EKF2)
float32 theta             # [rad] Pitch angle (MHE when authoritative, else EKF2)
float32 psi               # [rad] Yaw/heading angle (MHE when authoritative, else EKF2)
float32 alpha_est         # [rad] Estimated angle of attack
float32 gamma_rad         # [rad] Flight path angle (positive climbing)

# --- Position / velocity in guidance frame ---
float32 pos_downrange     # [m] Downrange position (Xm)
float32 pos_crossrange    # [m] Crossrange position (Zm)
float32 vel_downrange     # [m/s] Downrange velocity (Vxm)
float32 vel_down          # [m/s] Down velocity (Vym)
float32 vel_crossrange    # [m/s] Crossrange velocity (Vzm)

# --- Full MPC state vector (matches x0 passed to the acados solver) ---
#   [0]  V          (m/s)        airspeed
#   [1]  gamma      (rad)        flight-path angle
#   [2]  chi        (rad)        heading
#   [3..5]  p,q,r   (rad/s)      body angular rates (blended)
#   [6]  alpha      (rad)        angle of attack
#   [7]  beta       (rad)        sideslip
#   [8]  phi        (rad)        roll angle
#   [9]  h / H_SCALE              normalized altitude
#   [10] x / X_SCALE              normalized downrange
#   [11] y / Y_SCALE              normalized crossrange
#   [12..14] delta_e, delta_r, delta_a  (rad)   commanded fin deflections
#   [15..17] de_act, dr_act, da_act    (rad)   actual fin deflections
float32[18] x_mpc

# --- Diagnostics: EKF-MHE cooperation ---
float32 blend_alpha       # EKF-MHE blend factor [0..1] (0=pure EKF, 1=pure MHE)
float32 mhe_quality       # [0..1] MHE solver quality metric
uint8   mhe_valid         # 1 iff the last MHE solve produced finite states and met the quality gate
int32   mhe_status        # Last MHE solver return code (acados status)

# --- Diagnostics: solver performance ---
float32 mhe_solve_ms      # [ms] Last MHE solve wall-clock time
float32 mpc_solve_ms      # [ms] Last MPC solve wall-clock time
int32   mpc_solver_status # Last MPC solver return code (acados status)
uint32  mpc_sqp_iter      # Last MPC SQP iteration count
uint32  mpc_solve_count   # Cumulative successful MPC solve count

# --- Diagnostics: cross-validation (MHE vs EKF2) ---
float32 xval_gamma_err    # [rad] |gamma_mhe - gamma_ekf| last cycle
float32 xval_chi_err      # [rad] wrapped |chi_mhe - chi_ekf| last cycle
float32 xval_alt_err      # [m]   |alt_mhe - alt_ekf| last cycle
float32 xval_penalty      # [0..1] Cross-validation penalty applied to the blend

# --- Servo feedback ---
uint8 servo_online_mask   # Bitmask: bit0=servo0 .. bit3=servo3 (1=online, 0=offline)

# --- Navigation ---
float32 bearing_deg              # [deg] Bearing to target from arm heading
float32 target_range_remaining   # [m] Remaining range to target

# --- Event flags ---
bool launched                    # True after launch detection

# --- dt measurement ---
float32 dt_actual                # [s] Measured control loop dt this cycle
float32 dt_min                   # [s] Minimum dt seen since launch
float32 dt_max                   # [s] Maximum dt seen since launch
