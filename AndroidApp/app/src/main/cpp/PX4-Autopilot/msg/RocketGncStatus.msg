# Rocket GNC Status
# Published by rocket_mpc module for logging and QGroundControl telemetry
#
# NOTE: This message is shared between the legacy rocket_gnc and the newer
# rocket_mpc modules.  Several fields have been repurposed by rocket_mpc
# (documented below).  Legacy fields that are no longer written by any
# active module have been removed.

uint64 timestamp          # [us] Time since system start
uint64 timestamp_sample   # [us] Timestamp of input data

uint8 stage               # Flight stage (1=pre-launch, 2=in-flight)
float32 t_flight          # [s] Time since launch detection
float32 q_dyn             # [Pa] Dynamic pressure
float32 q_kgf             # [kgf/cm2] Dynamic pressure scaled

float32 pitch_accel_cmd   # [rad] gamma_ref — flight-path angle reference from LOS guidance
float32 yaw_accel_cmd     # [rad] chi_ref — heading reference from LOS guidance
float32 yaw_los_deg       # EKF-MHE blend factor [0..1] (0=pure EKF, 1=pure MHE)

float32 delta_roll        # [rad] Aileron (roll channel) fin command (delta_a)
float32 delta_pitch       # [rad] Elevator (pitch channel) fin command (delta_e)
float32 delta_yaw         # [rad] Rudder (yaw channel) fin command (delta_r)

float32 fin1              # [rad] Fin 1 deflection
float32 fin2              # [rad] Fin 2 deflection
float32 fin3              # [rad] Fin 3 deflection
float32 fin4              # [rad] Fin 4 deflection

float32 altitude          # [m] Current altitude (MHE when authoritative, else EKF2)
float32 airspeed          # [m/s] Current airspeed (MHE when authoritative, else EKF2)
float32 rho               # [kg/m3] Air density

float32 phi               # [rad] Roll angle (MHE when authoritative, else EKF2)
float32 wx_filter         # [ms] MPC solver wall-clock time
float32 du_roll           # MHE quality metric [0..1]
float32 out_integ_roll    # MPC cumulative solve count

uint8 servo_online_mask   # Bitmask: bit0=servo0 .. bit3=servo3 (1=online, 0=offline)

# --- Attitude angles ---
float32 theta                    # [rad] Pitch angle
float32 psi                      # [rad] Yaw/heading angle
float32 alpha_est                # [rad] Estimated angle of attack
float32 gamma_rad                # [rad] Flight path angle

# --- Position in guidance frame ---
float32 pos_downrange            # [m] Downrange position (Xm)
float32 pos_crossrange           # [m] Crossrange position (Zm)
float32 vel_downrange            # [m/s] Downrange velocity (Vxm)
float32 vel_down                 # [m/s] Down velocity (Vym)
float32 vel_crossrange           # [m/s] Crossrange velocity (Vzm)

# --- Navigation ---
float32 bearing_deg              # [deg] Bearing to target from arm heading
float32 target_range_remaining   # [m] Remaining range to target

# --- Event flags ---
bool launched                    # True after launch detection

# --- dt measurement ---
float32 dt_actual                # [s] Measured control loop dt this cycle
float32 dt_min                   # [s] Minimum dt seen since launch
float32 dt_max                   # [s] Maximum dt seen since launch
