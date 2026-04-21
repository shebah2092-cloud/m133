#!/usr/bin/env python3
"""
Launch Initial Conditions Generator
مولد الشروط الابتدائية للإطلاق

This script calculates initial conditions for rocket/missile simulations
and outputs values ready for the 6DOF simulation configuration file.

هذا السكربت يحسب الشروط الابتدائية لمحاكاة الصواريخ
ويُخرج القيم الجاهزة لملف تكوين محاكاة 6DOF.

Supports / يدعم:
1. Ground Launch (الإطلاق الأرضي): V=0, specify launch angle
2. Air Launch / Post-Booster (الإطلاق الجوي / بعد البوستر): specify Mach, altitude, angles

Usage / الاستخدام:
    python launch_config_generator.py

"""

import numpy as np
import sys
from pathlib import Path
from datetime import datetime

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dynamics.quaternion_utils import degrees_to_quaternion

# Output config file path (same directory as script)
OUTPUT_CONFIG_FILE = Path(__file__).parent / "generated_initial_conditions.yaml"


def get_speed_of_sound(altitude_m: float, temperature_offset_K: float = 0.0) -> float:
    """
    Calculate speed of sound at given altitude using ISA atmosphere model.
    حساب سرعة الصوت عند ارتفاع معين باستخدام نموذج الغلاف الجوي القياسي.
    
    Args:
        altitude_m: Altitude in meters (الارتفاع بالمتر)
        temperature_offset_K: Temperature offset from ISA (انحراف درجة الحرارة عن ISA)
    
    Returns:
        Speed of sound in m/s (سرعة الصوت بالمتر/ثانية)
    """
    # ISA atmosphere constants
    T0 = 288.15  # Sea level temperature (K)
    L = 0.0065   # Temperature lapse rate (K/m) for troposphere
    R = 287.05   # Gas constant for air (J/kg·K)
    gamma = 1.4  # Ratio of specific heats for air
    
    # Calculate temperature at altitude
    if altitude_m <= 11000:
        # Troposphere
        T = T0 - L * altitude_m
    elif altitude_m <= 20000:
        # Tropopause (isothermal layer)
        T = 216.65
    elif altitude_m <= 32000:
        # Stratosphere (temperature increases)
        T = 216.65 + 0.001 * (altitude_m - 20000)
    else:
        # Upper stratosphere
        T = 228.65 + 0.0028 * (altitude_m - 32000)
    
    # Apply temperature offset
    T += temperature_offset_K
    
    # Speed of sound: a = sqrt(gamma * R * T)
    return np.sqrt(gamma * R * T)


def mach_to_speed(mach: float, altitude_m: float, temperature_offset_K: float = 0.0) -> float:
    """
    Convert Mach number to speed in m/s.
    تحويل رقم ماخ إلى سرعة بالمتر/ثانية.
    
    Args:
        mach: Mach number (رقم ماخ)
        altitude_m: Altitude in meters (الارتفاع بالمتر)
        temperature_offset_K: Temperature offset from ISA (انحراف درجة الحرارة عن ISA)
    
    Returns:
        Speed in m/s (السرعة بالمتر/ثانية)
    """
    a = get_speed_of_sound(altitude_m, temperature_offset_K)
    return mach * a


def speed_to_mach(speed_ms: float, altitude_m: float, temperature_offset_K: float = 0.0) -> float:
    """
    Convert speed in m/s to Mach number.
    تحويل السرعة بالمتر/ثانية إلى رقم ماخ.
    
    Args:
        speed_ms: Speed in m/s (السرعة بالمتر/ثانية)
        altitude_m: Altitude in meters (الارتفاع بالمتر)
        temperature_offset_K: Temperature offset from ISA (انحراف درجة الحرارة عن ISA)
    
    Returns:
        Mach number (رقم ماخ)
    """
    a = get_speed_of_sound(altitude_m, temperature_offset_K)
    return speed_ms / a


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list:
    """
    Convert Euler angles to quaternion (scalar first: [q0, q1, q2, q3]).
    تحويل زوايا أويلر إلى رباعي (العددي أولاً: [q0, q1, q2, q3]).
    
    This function uses the centralized degrees_to_quaternion from dynamics/quaternion_utils.py.
    هذه الدالة تستخدم degrees_to_quaternion المركزية من dynamics/quaternion_utils.py.
    
    Args:
        roll_deg: Roll angle in degrees (زاوية الدوران حول المحور الطولي)
        pitch_deg: Pitch angle in degrees (زاوية الميل - موجب = للأعلى)
        yaw_deg: Yaw/Heading angle in degrees (زاوية الاتجاه - 0=شمال، 90=شرق)
    
    Returns:
        Quaternion [q0, q1, q2, q3] (scalar first)
    """
    q = degrees_to_quaternion(roll_deg, pitch_deg, yaw_deg)
    return q.tolist()


def calculate_velocity_ned(speed_ms: float, pitch_deg: float, heading_deg: float) -> list:
    """
    Calculate velocity vector in NED frame.
    حساب متجه السرعة في إطار NED.
    
    Args:
        speed_ms: Total speed in m/s (السرعة الكلية بالمتر/ثانية)
        pitch_deg: Pitch angle in degrees (positive = climbing) (زاوية الميل - موجب = صعود)
        heading_deg: Heading in degrees (0=North, 90=East) (الاتجاه - 0=شمال، 90=شرق)
    
    Returns:
        Velocity vector [Vn, Ve, Vd] in m/s
    """
    if speed_ms == 0:
        return [0.0, 0.0, 0.0]
    
    pitch_rad = np.radians(pitch_deg)
    heading_rad = np.radians(heading_deg)
    
    # Horizontal speed component
    V_horizontal = speed_ms * np.cos(pitch_rad)
    
    # Vertical speed component (negative because Down is positive in NED)
    Vd = -speed_ms * np.sin(pitch_rad)
    
    # North and East components
    Vn = V_horizontal * np.cos(heading_rad)
    Ve = V_horizontal * np.sin(heading_rad)
    
    return [Vn, Ve, Vd]


def get_user_input(prompt: str, default=None, input_type: type = float):
    """Get user input with default value support."""
    if default is not None:
        prompt_with_default = f"{prompt} [{default}]: "
    else:
        prompt_with_default = f"{prompt}: "
    
    user_input = input(prompt_with_default).strip()
    
    if user_input == "" and default is not None:
        return default
    
    try:
        return input_type(user_input)
    except ValueError:
        print(f"  خطأ: قيمة غير صالحة. استخدام القيمة الافتراضية: {default}")
        return default if default is not None else 0.0


def get_user_choice(prompt: str, options: list, default: int = 1) -> int:
    """Get user choice from numbered options."""
    print(prompt)
    for i, option in enumerate(options, 1):
        marker = " *" if i == default else ""
        print(f"  {i}. {option}{marker}")
    
    choice = get_user_input("  اختيارك / Your choice", default, int)
    
    if 1 <= choice <= len(options):
        return choice
    else:
        print(f"  خطأ: اختيار غير صالح. استخدام الافتراضي: {default}")
        return default


def print_header():
    """Print script header."""
    print("=" * 70)
    print("مولد الشروط الابتدائية للإطلاق")
    print("Launch Initial Conditions Generator")
    print("=" * 70)
    print()


def print_section(title_ar: str, title_en: str):
    """Print section header."""
    print(f"【 {title_ar} / {title_en} 】")


def generate_ground_launch_config():
    """Generate configuration for ground launch (V=0)."""
    print()
    print("-" * 70)
    print("الإطلاق الأرضي - السرعة الابتدائية = صفر")
    print("Ground Launch - Initial velocity = 0")
    print("-" * 70)
    print()
    
    # Get inputs
    print_section("ارتفاع موقع الإطلاق", "Launch Site Altitude")
    altitude_m = get_user_input("  الارتفاع (متر) / Altitude (m)", 0.0)
    print()
    
    print_section("زاوية الإطلاق", "Launch Angle")
    pitch_deg = get_user_input("  زاوية الميل (درجة، موجب=للأعلى) / Pitch angle (deg, +ve=up)", 45.0)
    print()
    
    print_section("الاتجاه", "Heading")
    heading_deg = get_user_input("  الاتجاه (درجة، 0=شمال، 90=شرق) / Heading (deg, 0=N, 90=E)", 0.0)
    print()
    
    print_section("الدوران", "Roll")
    roll_deg = get_user_input("  زاوية الدوران (درجة) / Roll angle (deg)", 0.0)
    print()
    
    # Calculate quaternion
    quaternion = euler_to_quaternion(roll_deg, pitch_deg, heading_deg)
    
    # Velocity is zero for ground launch
    velocity_ned = [0.0, 0.0, 0.0]
    
    return {
        'launch_type': 'ground',
        'altitude_m': altitude_m,
        'speed_ms': 0.0,
        'mach_number': 0.0,
        'pitch_deg': pitch_deg,
        'heading_deg': heading_deg,
        'roll_deg': roll_deg,
        'velocity_ned': velocity_ned,
        'quaternion': quaternion
    }


def generate_air_launch_config():
    """Generate configuration for air launch / post-booster."""
    print()
    print("-" * 70)
    print("الإطلاق الجوي / بعد انتهاء البوستر")
    print("Air Launch / Post-Booster")
    print("-" * 70)
    print()
    
    # Get inputs
    print_section("الارتفاع", "Altitude")
    altitude_m = get_user_input("  الارتفاع (متر) / Altitude (m)", 5000.0)
    print()
    
    print_section("السرعة", "Speed")
    speed_unit = get_user_choice(
        "  وحدة السرعة / Speed unit:",
        ["ماخ / Mach", "م/ث / m/s", "كم/س / km/h"],
        default=1
    )
    print()
    
    if speed_unit == 1:  # Mach
        mach_number = get_user_input("  رقم ماخ / Mach number", 2.0)
        speed_ms = mach_to_speed(mach_number, altitude_m)
    elif speed_unit == 2:  # m/s
        speed_ms = get_user_input("  السرعة (م/ث) / Speed (m/s)", 600.0)
        mach_number = speed_to_mach(speed_ms, altitude_m)
    else:  # km/h
        speed_kmh = get_user_input("  السرعة (كم/س) / Speed (km/h)", 2000.0)
        speed_ms = speed_kmh / 3.6
        mach_number = speed_to_mach(speed_ms, altitude_m)
    print()
    
    print_section("زاوية الطيران", "Flight Angle")
    pitch_deg = get_user_input("  زاوية الميل (درجة، موجب=صعود، 0=أفقي) / Pitch (deg, +ve=climb, 0=level)", 0.0)
    print()
    
    print_section("الاتجاه", "Heading")
    heading_deg = get_user_input("  الاتجاه (درجة، 0=شمال، 90=شرق) / Heading (deg, 0=N, 90=E)", 0.0)
    print()
    
    print_section("الدوران", "Roll")
    roll_deg = get_user_input("  زاوية الدوران (درجة) / Roll angle (deg)", 0.0)
    print()
    
    # Calculate velocity and quaternion
    velocity_ned = calculate_velocity_ned(speed_ms, pitch_deg, heading_deg)
    quaternion = euler_to_quaternion(roll_deg, pitch_deg, heading_deg)
    
    return {
        'launch_type': 'air',
        'altitude_m': altitude_m,
        'speed_ms': speed_ms,
        'mach_number': mach_number,
        'pitch_deg': pitch_deg,
        'heading_deg': heading_deg,
        'roll_deg': roll_deg,
        'velocity_ned': velocity_ned,
        'quaternion': quaternion
    }


def save_config_to_file(config: dict, filepath: Path = None) -> str:
    """
    Save the generated configuration to a YAML file.
    حفظ التكوين المُولَّد في ملف YAML.
    
    Args:
        config: Configuration dictionary
        filepath: Output file path (default: generated_initial_conditions.yaml)
    
    Returns:
        Path to the saved file
    """
    if filepath is None:
        filepath = OUTPUT_CONFIG_FILE
    
    v = config['velocity_ned']
    q = config['quaternion']
    
    # Generate timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # Build YAML content
    content = f"""# Generated Initial Conditions / الشروط الابتدائية المُولَّدة
# Generated at / تم التوليد في: {timestamp}
# Launch type / نوع الإطلاق: {'Ground / أرضي' if config['launch_type'] == 'ground' else 'Air / جوي'}

launch:
  altitude: {config['altitude_m']:.1f}        # m - ارتفاع الإطلاق

initial_conditions:
  position: [0.0, 0.0, 0.0]   # m (NED frame)
"""
    
    if config['launch_type'] == 'ground':
        content += f"  velocity: [0.0, 0.0, 0.0]   # m/s (ground launch - إطلاق أرضي)\n"
    else:
        content += f"  velocity: [{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}]   # m/s (Mach {config['mach_number']:.2f})\n"
    
    content += f"""  attitude: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]  # quaternion (pitch={config['pitch_deg']:.1f}°, heading={config['heading_deg']:.1f}°)
  angular_velocity: [0.0, 0.0, 0.0]  # rad/s

# Input parameters / معاملات الإدخال:
#   Altitude / الارتفاع: {config['altitude_m']:.1f} m
#   Pitch / زاوية الميل: {config['pitch_deg']:.1f}°
#   Heading / الاتجاه: {config['heading_deg']:.1f}°
#   Roll / الدوران: {config['roll_deg']:.1f}°
"""
    
    if config['launch_type'] == 'air':
        content += f"""#   Speed / السرعة: {config['speed_ms']:.2f} m/s = Mach {config['mach_number']:.3f}
#   Vn (North): {v[0]:.2f} m/s
#   Ve (East): {v[1]:.2f} m/s
#   Vd (Down): {v[2]:.2f} m/s
"""
    
    # Write to file
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    
    return str(filepath)


def print_results(config: dict):
    """Print the generated configuration."""
    print()
    print("=" * 70)
    print("النتائج / Results")
    print("=" * 70)
    print()
    
    # Calculation info
    print_section("معلومات الحساب", "Calculation Info")
    
    if config['launch_type'] == 'ground':
        print("  نوع الإطلاق: أرضي (السرعة الابتدائية = 0)")
        print("  Launch type: Ground (initial velocity = 0)")
    else:
        print("  نوع الإطلاق: جوي / بعد البوستر")
        print("  Launch type: Air / Post-booster")
        
        speed_of_sound = get_speed_of_sound(config['altitude_m'])
        print(f"  سرعة الصوت عند الارتفاع {config['altitude_m']:.0f} م: {speed_of_sound:.2f} م/ث")
        print(f"  Speed of sound at {config['altitude_m']:.0f} m: {speed_of_sound:.2f} m/s")
        print(f"  السرعة: {config['speed_ms']:.2f} م/ث = ماخ {config['mach_number']:.3f}")
        print(f"  Speed: {config['speed_ms']:.2f} m/s = Mach {config['mach_number']:.3f}")
    
    print(f"  الارتفاع: {config['altitude_m']:.1f} م")
    print(f"  Altitude: {config['altitude_m']:.1f} m")
    print(f"  زاوية الميل: {config['pitch_deg']:.1f}°")
    print(f"  Pitch angle: {config['pitch_deg']:.1f}°")
    print(f"  الاتجاه: {config['heading_deg']:.1f}°")
    print(f"  Heading: {config['heading_deg']:.1f}°")
    print()
    
    # Config file values
    print_section("القيم لملف التكوين", "Values for Config File")
    print("-" * 70)
    print()
    print("# انسخ هذا القسم إلى ملف التكوين")
    print("# Copy this section to your config file")
    print()
    print("launch:")
    print(f"  altitude: {config['altitude_m']:.1f}        # m - ارتفاع الإطلاق")
    print()
    print("initial_conditions:")
    print(f"  position: [0.0, 0.0, 0.0]   # m (NED frame)")
    
    v = config['velocity_ned']
    q = config['quaternion']
    
    if config['launch_type'] == 'ground':
        print(f"  velocity: [0.0, 0.0, 0.0]   # m/s (ground launch)")
    else:
        print(f"  velocity: [{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}]   # m/s (Mach {config['mach_number']:.2f})")
    
    print(f"  attitude: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]  # quaternion (pitch={config['pitch_deg']:.1f}°, heading={config['heading_deg']:.1f}°)")
    print(f"  angular_velocity: [0.0, 0.0, 0.0]  # rad/s")
    print()
    print("-" * 70)
    print()
    
    # Additional info
    print_section("معلومات إضافية", "Additional Info")
    print(f"  مركبة السرعة شمال (Vn): {v[0]:.2f} م/ث")
    print(f"  مركبة السرعة شرق (Ve): {v[1]:.2f} م/ث")
    print(f"  مركبة السرعة عمودي (Vd): {v[2]:.2f} م/ث (سالب = صعود)")
    print()
    
    # Verify quaternion
    q_norm = np.sqrt(sum(qi**2 for qi in q))
    print(f"  تحقق من الرباعي (يجب أن يكون 1.0): {q_norm:.6f}")
    print()
    
    print("=" * 70)
    print("تم الانتهاء / Done")
    print("=" * 70)


def main():
    """Main function to generate launch configuration."""
    
    print_header()
    
    print("هذه الأداة تحسب الشروط الابتدائية لمحاكاة الصواريخ")
    print("This tool calculates initial conditions for rocket simulation")
    print()
    
    # Choose launch type
    launch_type = get_user_choice(
        "اختر نوع الإطلاق / Choose launch type:",
        [
            "إطلاق أرضي (V=0) / Ground launch (V=0)",
            "إطلاق جوي / بعد البوستر (V>0) / Air launch / Post-booster (V>0)"
        ],
        default=1
    )
    
    if launch_type == 1:
        config = generate_ground_launch_config()
    else:
        config = generate_air_launch_config()
    
    print_results(config)
    
    # Save to file automatically
    saved_path = save_config_to_file(config)
    print()
    print("=" * 70)
    print(f"تم حفظ التكوين في / Config saved to:")
    print(f"  {saved_path}")
    print("=" * 70)
    
    return config


if __name__ == "__main__":
    main()
