#!/usr/bin/env python3
"""Servo dynamics with rate/deflection limits and optional backlash."""

import numpy as np


def _coerce_vector_param(value, n_actuators, param_name, default_value):
    """Convert value to array of length n_actuators (broadcasts scalars)."""
    if value is None:
        return np.full(n_actuators, default_value)

    arr = np.asarray(value)

    if arr.ndim == 0 or (arr.ndim == 1 and len(arr) == 1):
        scalar_val = float(arr) if arr.ndim == 0 else float(arr[0])
        return np.full(n_actuators, scalar_val)

    if arr.ndim > 1:
        raise ValueError(
            f"{param_name} must be a 1D array or scalar, got {arr.ndim}D array with shape {arr.shape}"
        )

    if len(arr) != n_actuators:
        raise ValueError(
            f"{param_name} length ({len(arr)}) must match n_actuators ({n_actuators}). "
            f"Got {param_name}={list(arr)}"
        )

    return arr.astype(float)


class ActuatorModel:
    """
    Servo/actuator dynamics model.
    Implements realistic actuator response with time constant and limits.
    """

    def __init__(self, config):
        """Initialize actuator model from config dict."""

        self.tau_servo = config.get('tau_servo', 0.05)  # s (tservo)
        if not np.isfinite(self.tau_servo):
            raise ValueError(f"tau_servo must be finite, got {self.tau_servo}")
        if self.tau_servo <= 0:
            raise ValueError(f"tau_servo must be positive, got {self.tau_servo}")

        self.delta_max = config.get('delta_max', np.deg2rad(15.0))  # rad (finlim)
        self.delta_min = config.get('delta_min', -np.deg2rad(15.0))  # rad

        if not np.isfinite(self.delta_max) or not np.isfinite(self.delta_min):
            raise ValueError(
                f"delta_min/delta_max must be finite, got delta_min={self.delta_min}, delta_max={self.delta_max}"
            )

        if self.delta_min >= self.delta_max:
            raise ValueError(
                f"delta_min ({self.delta_min:.4f} rad = {np.rad2deg(self.delta_min):.2f}°) "
                f"must be < delta_max ({self.delta_max:.4f} rad = {np.rad2deg(self.delta_max):.2f}°)"
            )

        self.rate_max = config.get('rate_max', np.deg2rad(300.0))  # rad/s

        if not np.isfinite(self.rate_max):
            raise ValueError(f"rate_max must be finite, got {self.rate_max}")
        if self.rate_max <= 0:
            raise ValueError(
                f"rate_max must be positive, got {self.rate_max:.4f} rad/s = {np.rad2deg(self.rate_max):.2f}°/s"
            )

        self.n_actuators = config.get('n_actuators', 4)

        if not isinstance(self.n_actuators, (int, np.integer)):
            raise ValueError(
                f"n_actuators must be an integer, got {type(self.n_actuators).__name__}: {self.n_actuators}"
            )
        if self.n_actuators <= 0:
            raise ValueError(
                f"n_actuators must be positive, got {self.n_actuators}"
            )

        self.delta_current = np.zeros(self.n_actuators)

        # Saturation flags for anti-windup feedback.
        # Set during update() to indicate which actuators are rate- or deflection-limited.
        self.rate_limited = np.zeros(self.n_actuators, dtype=bool)
        self.deflection_limited = np.zeros(self.n_actuators, dtype=bool)

        delta_trim_config = config.get('delta_trim', None)
        self.delta_trim = _coerce_vector_param(
            delta_trim_config, self.n_actuators, 'delta_trim', default_value=0.0
        )

        delta_scale_config = config.get('delta_scale', None)
        self.delta_scale = _coerce_vector_param(
            delta_scale_config, self.n_actuators, 'delta_scale', default_value=1.0
        )

        # config_type defines fin geometry (X or +) - used by ControlAllocator, not ActuatorModel
        self.config_type = config.get('config_type', 'X')
        allowed_config_types = ('X', '+')
        if self.config_type not in allowed_config_types:
            raise ValueError(
                f"config_type must be one of {allowed_config_types}, got '{self.config_type}'"
            )

    def update(self, delta_cmd, dt):
        """Update actuator positions with first-order dynamics and limits."""

        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        delta_cmd = np.asarray(delta_cmd)

        if delta_cmd.ndim != 1:
            raise ValueError(
                f"delta_cmd must be a 1D array of shape ({self.n_actuators},), got shape {delta_cmd.shape}"
            )

        if delta_cmd.shape[0] != self.n_actuators:
            raise ValueError(
                f"delta_cmd length {delta_cmd.shape[0]} != n_actuators {self.n_actuators}"
            )

        if not np.all(np.isfinite(delta_cmd)):
            raise ValueError(f"delta_cmd must contain only finite values, got {delta_cmd}")

        # Apply scale first, then add trim (correct servo calibration semantics)
        delta_cmd_adjusted = delta_cmd * self.delta_scale + self.delta_trim

        # Exact discrete-time solution for first-order lag: delta(t+dt) = delta(t) + (1 - exp(-dt/tau)) * error
        # Uses -expm1(-x) instead of 1-exp(-x) for better numerical precision at small dt/tau
        alpha = -np.expm1(-dt / self.tau_servo)
        delta_change = alpha * (delta_cmd_adjusted - self.delta_current)

        # Apply rate limiting
        max_change = self.rate_max * dt
        delta_change_raw = delta_change.copy()
        delta_change = np.clip(delta_change, -max_change, max_change)

        EPS_RATE = 1e-6
        self.rate_limited = np.abs(delta_change_raw) > (max_change + EPS_RATE)

        delta_new = self.delta_current + delta_change

        delta_before_clip = delta_new.copy()
        delta_new = np.clip(delta_new, self.delta_min, self.delta_max)
        EPS_DEFL = 1e-6
        self.deflection_limited = (
            (delta_before_clip > self.delta_max + EPS_DEFL) |
            (delta_before_clip < self.delta_min - EPS_DEFL)
        )

        self.delta_current = delta_new

        return self.delta_current.copy()


class ActuatorWithBacklash(ActuatorModel):
    """Actuator model with backlash (play state tracking)."""

    def __init__(self, config):
        """Initialize actuator with backlash from config dict."""
        super().__init__(config)

        backlash_raw = config.get('backlash', 0.0)
        units = config.get('units', 'rad')

        self.backlash = _coerce_vector_param(backlash_raw, self.n_actuators, 'backlash', default_value=0.0)

        if not np.all(np.isfinite(self.backlash)):
            raise ValueError(f"backlash must be finite for all actuators, got {self.backlash}")

        if units.lower() in ['deg', 'degrees', 'degree']:
            self.backlash = np.deg2rad(self.backlash)

        if np.any(self.backlash < 0):
            negative_indices = np.where(self.backlash < 0)[0]
            negative_values = self.backlash[negative_indices]
            raise ValueError(
                f"backlash must be non-negative for all actuators. "
                f"Got negative values at indices {list(negative_indices)}: "
                f"{[f'{v:.4f} rad ({np.rad2deg(v):.2f}°)' for v in negative_values]}"
            )

        # play_state: motor position within backlash zone relative to output
        # Range: [-backlash/2, +backlash/2] per actuator
        self.play_state = np.zeros(self.n_actuators)
        self.motor_position = np.zeros(self.n_actuators)

    def update(self, delta_cmd, dt):
        """Update actuator with backlash modeling."""

        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        delta_cmd = np.asarray(delta_cmd)

        if delta_cmd.ndim != 1:
            raise ValueError(
                f"delta_cmd must be a 1D array of shape ({self.n_actuators},), got shape {delta_cmd.shape}"
            )

        if delta_cmd.shape[0] != self.n_actuators:
            raise ValueError(
                f"delta_cmd length {delta_cmd.shape[0]} != n_actuators {self.n_actuators}"
            )

        if not np.all(np.isfinite(delta_cmd)):
            raise ValueError(f"delta_cmd must contain only finite values, got {delta_cmd}")

        # Apply scale first, then add trim (correct servo calibration semantics)
        delta_cmd_adjusted = delta_cmd * self.delta_scale + self.delta_trim

        # === Step 1: Apply servo dynamics to motor position ===
        # Uses -expm1(-x) instead of 1-exp(-x) for better numerical precision at small dt/tau
        alpha = -np.expm1(-dt / self.tau_servo)
        motor_change = alpha * (delta_cmd_adjusted - self.motor_position)

        # Apply rate limiting to motor movement
        max_change = self.rate_max * dt
        motor_change_raw = motor_change.copy()
        motor_change = np.clip(motor_change, -max_change, max_change)

        EPS_RATE = 1e-6
        self.rate_limited = np.abs(motor_change_raw) > (max_change + EPS_RATE)

        # Motor position before saturation - motor can "try" to go beyond limits
        new_motor_position = self.motor_position + motor_change

        # === Step 2: Update play state and calculate output ===
        delta_new = np.zeros(self.n_actuators)

        for i in range(self.n_actuators):
            half_backlash = self.backlash[i] / 2.0

            if half_backlash <= 1e-12:
                # No backlash - direct coupling
                delta_new[i] = new_motor_position[i]
            else:
                motor_delta = new_motor_position[i] - self.motor_position[i]
                new_play_state = self.play_state[i] + motor_delta

                if new_play_state > half_backlash:
                    # Motor engaged on positive side - output moves
                    output_delta = new_play_state - half_backlash
                    delta_new[i] = self.delta_current[i] + output_delta
                    self.play_state[i] = half_backlash
                elif new_play_state < -half_backlash:
                    # Motor engaged on negative side - output moves
                    output_delta = new_play_state + half_backlash
                    delta_new[i] = self.delta_current[i] + output_delta
                    self.play_state[i] = -half_backlash
                else:
                    # Motor is in the backlash zone - output doesn't move
                    delta_new[i] = self.delta_current[i]
                    self.play_state[i] = new_play_state

        self.motor_position = new_motor_position

        # === Step 3: Apply saturation limits to output ===
        delta_before_clip = delta_new.copy()
        delta_new = np.clip(delta_new, self.delta_min, self.delta_max)
        EPS_DEFL = 1e-6
        self.deflection_limited = (
            (delta_before_clip > self.delta_max + EPS_DEFL) |
            (delta_before_clip < self.delta_min - EPS_DEFL)
        )

        # Limit motor position to ±backlash/2 beyond output limits (consistent with play_state range)
        self.motor_position = np.clip(
            self.motor_position,
            self.delta_min - self.backlash / 2.0,
            self.delta_max + self.backlash / 2.0
        )

        # === Step 4: Reconcile play_state after saturation ===
        # Recompute play_state = motor_position - output_position to maintain mechanical
        # consistency and prevent windup when output is at a hard stop.
        half_backlash = self.backlash / 2.0
        self.play_state = np.clip(
            self.motor_position - delta_new,
            -half_backlash,
            half_backlash
        )

        self.delta_current = delta_new

        return self.delta_current.copy()


class SecondOrderActuatorModel:
    """Second-order servo dynamics with time delay."""

    def __init__(self, config):
        """Initialize second-order actuator model from config dict."""

        # Natural frequency (rad/s)
        if 'wn' not in config:
            raise ValueError("'wn' (natural frequency rad/s) is required in actuator config")
        self.wn = config['wn']
        if not np.isfinite(self.wn):
            raise ValueError(f"wn must be finite, got {self.wn}")
        if self.wn <= 0:
            raise ValueError(f"wn must be positive, got {self.wn}")

        # Damping ratio * natural frequency
        if 'zeta_wn' not in config:
            raise ValueError("'zeta_wn' (damping_ratio * natural_frequency) is required in actuator config")
        self.zeta_wn = config['zeta_wn']
        if not np.isfinite(self.zeta_wn):
            raise ValueError(f"zeta_wn must be finite, got {self.zeta_wn}")
        if self.zeta_wn < 0:
            raise ValueError(f"zeta_wn must be non-negative, got {self.zeta_wn}")

        # Deflection limits
        if 'delta_max' not in config:
            raise ValueError("'delta_max' (rad) is required in actuator config")
        if 'delta_min' not in config:
            raise ValueError("'delta_min' (rad) is required in actuator config")
        self.delta_max = config['delta_max']
        self.delta_min = config['delta_min']

        if not np.isfinite(self.delta_max) or not np.isfinite(self.delta_min):
            raise ValueError(
                f"delta_min/delta_max must be finite, got delta_min={self.delta_min}, delta_max={self.delta_max}"
            )

        if self.delta_min >= self.delta_max:
            raise ValueError(
                f"delta_min ({self.delta_min:.4f} rad = {np.rad2deg(self.delta_min):.2f} deg) "
                f"must be < delta_max ({self.delta_max:.4f} rad = {np.rad2deg(self.delta_max):.2f} deg)"
            )

        # Rate limit (rad/s)
        if 'rate_max' not in config:
            raise ValueError("'rate_max' (rad/s) is required in actuator config")
        self.rate_max = config['rate_max']

        if not np.isfinite(self.rate_max):
            raise ValueError(f"rate_max must be finite, got {self.rate_max}")
        if self.rate_max <= 0:
            raise ValueError(
                f"rate_max must be positive, got {self.rate_max:.4f} rad/s = {np.rad2deg(self.rate_max):.2f} deg/s"
            )

        # Number of actuators
        if 'n_actuators' not in config:
            raise ValueError("'n_actuators' is required in actuator config")
        self.n_actuators = config['n_actuators']

        if not isinstance(self.n_actuators, (int, np.integer)):
            raise ValueError(
                f"n_actuators must be an integer, got {type(self.n_actuators).__name__}: {self.n_actuators}"
            )
        if self.n_actuators <= 0:
            raise ValueError(f"n_actuators must be positive, got {self.n_actuators}")

        # Time delay configuration
        if 'delay_steps' not in config:
            raise ValueError("'delay_steps' is required in actuator config")
        if 'delay_buffer_size' not in config:
            raise ValueError("'delay_buffer_size' is required in actuator config")
        self.delay_steps = config['delay_steps']
        self.delay_buffer_size = config['delay_buffer_size']

        if not isinstance(self.delay_steps, (int, np.integer)):
            raise ValueError(f"delay_steps must be an integer, got {type(self.delay_steps).__name__}")
        if self.delay_steps < 0:
            raise ValueError(f"delay_steps must be non-negative, got {self.delay_steps}")
        if self.delay_steps >= self.delay_buffer_size:
            raise ValueError(
                f"delay_steps ({self.delay_steps}) must be < delay_buffer_size ({self.delay_buffer_size})"
            )

        # State variables
        self.delta_current = np.zeros(self.n_actuators)   # Position (actual fin deflection)
        self.velocity = np.zeros(self.n_actuators)         # Velocity (rate of change of deflection)
        self.delay_buffers = [np.zeros(self.delay_buffer_size) for _ in range(self.n_actuators)]

        # Saturation tracking flags
        self.rate_limited = np.zeros(self.n_actuators, dtype=bool)
        self.deflection_limited = np.zeros(self.n_actuators, dtype=bool)

        # Trim and scale (for compatibility with ActuatorModel interface)
        delta_trim_config = config.get('delta_trim', None)
        self.delta_trim = _coerce_vector_param(
            delta_trim_config, self.n_actuators, 'delta_trim', default_value=0.0
        )

        delta_scale_config = config.get('delta_scale', None)
        self.delta_scale = _coerce_vector_param(
            delta_scale_config, self.n_actuators, 'delta_scale', default_value=1.0
        )

        # Config type (for compatibility)
        self.config_type = config.get('config_type', 'X')
        allowed_config_types = ('X', '+')
        if self.config_type not in allowed_config_types:
            raise ValueError(
                f"config_type must be one of {allowed_config_types}, got '{self.config_type}'"
            )

        # For compatibility with ActuatorModel - tau_servo equivalent
        # For second-order system: tau ≈ 1/(zeta*wn) for overdamped, or 1/wn for underdamped
        zeta = self.zeta_wn / self.wn if self.wn > 0 else 0
        if zeta > 0:
            self.tau_servo = 1.0 / self.zeta_wn
        else:
            self.tau_servo = 1.0 / self.wn if self.wn > 0 else 0.05

    def update(self, delta_cmd, dt):
        """Update actuator positions with second-order dynamics and limits."""

        # Input validation
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        delta_cmd = np.asarray(delta_cmd)

        if delta_cmd.ndim != 1:
            raise ValueError(
                f"delta_cmd must be a 1D array of shape ({self.n_actuators},), got shape {delta_cmd.shape}"
            )

        if delta_cmd.shape[0] != self.n_actuators:
            raise ValueError(
                f"delta_cmd length {delta_cmd.shape[0]} != n_actuators {self.n_actuators}"
            )

        if not np.all(np.isfinite(delta_cmd)):
            raise ValueError(f"delta_cmd must contain only finite values, got {delta_cmd}")

        delta_cmd_adjusted = delta_cmd * self.delta_scale + self.delta_trim

        # Reset saturation flags
        self.rate_limited = np.zeros(self.n_actuators, dtype=bool)
        self.deflection_limited = np.zeros(self.n_actuators, dtype=bool)

        # Get delayed commands from shift registers
        delayed_cmd = np.array([
            self.delay_buffers[i][self.delay_steps]
            for i in range(self.n_actuators)
        ])

        # Standard second-order dynamics: δ̈ = -2ζωₙδ̇ - ωₙ²δ + ωₙ²u
        velocity_derivative = (
            -2.0 * self.zeta_wn * self.velocity
            - self.wn**2 * self.delta_current
            + self.wn**2 * delayed_cmd
        )

        # Integrate velocity (Euler method)
        new_velocity = self.velocity + velocity_derivative * dt

        # Apply rate limiting
        for i in range(self.n_actuators):
            if abs(new_velocity[i]) > self.rate_max:
                new_velocity[i] = self.rate_max * np.sign(new_velocity[i])
                self.rate_limited[i] = True

        # Integrate position
        new_position = self.delta_current + new_velocity * dt

        # Apply deflection limits
        position_before_limit = new_position.copy()
        new_position = np.clip(new_position, self.delta_min, self.delta_max)

        EPS_DEFL = 1e-6
        self.deflection_limited = (
            (position_before_limit > self.delta_max + EPS_DEFL) |
            (position_before_limit < self.delta_min - EPS_DEFL)
        )

        # Zero velocity when hitting mechanical hard stops
        for i in range(self.n_actuators):
            if new_position[i] >= self.delta_max and new_velocity[i] > 0:
                new_velocity[i] = 0.0
            elif new_position[i] <= self.delta_min and new_velocity[i] < 0:
                new_velocity[i] = 0.0

        self.velocity = new_velocity
        self.delta_current = new_position

        # Update shift registers (time delay buffers)
        for i in range(self.n_actuators):
            for j in range(self.delay_buffer_size - 1, 0, -1):
                self.delay_buffers[i][j] = self.delay_buffers[i][j - 1]
            self.delay_buffers[i][0] = delta_cmd_adjusted[i]

        return self.delta_current.copy()
