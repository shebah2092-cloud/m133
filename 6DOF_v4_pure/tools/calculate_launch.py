#!/usr/bin/env python3
"""
حاسبة اتجاه الإطلاق البسيطة - Simple Launch Direction Calculator

هذه الأداة تقرأ من ملف launch_input.yaml وتحسب quaternion الاتجاه

الاستخدام / Usage:
    1. عدّل ملف tools/launch_input.yaml بالقيم المطلوبة
       Edit tools/launch_input.yaml with your values
    2. شغّل هذا الملف:
       Run this file:
       python tools/calculate_launch.py
"""

import numpy as np
import yaml
from pathlib import Path


def euler_to_quaternion(pitch_deg: float, yaw_deg: float = 0.0, roll_deg: float = 0.0) -> list:
    """تحويل زوايا أويلر إلى quaternion"""
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    roll = np.radians(roll_deg)
    
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    
    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy
    
    return [round(q0, 4), round(q1, 4), round(q2, 4), round(q3, 4)]


def parse_direction(direction) -> float:
    """تحويل اسم الاتجاه أو الرقم إلى زاوية"""
    if isinstance(direction, (int, float)):
        return float(direction)
    
    direction_map = {
        'north': 0.0, 'n': 0.0, 'شمال': 0.0,
        'northeast': 45.0, 'ne': 45.0, 'شمال_شرق': 45.0,
        'east': 90.0, 'e': 90.0, 'شرق': 90.0,
        'southeast': 135.0, 'se': 135.0, 'جنوب_شرق': 135.0,
        'south': 180.0, 's': 180.0, 'جنوب': 180.0,
        'southwest': 225.0, 'sw': 225.0, 'جنوب_غرب': 225.0,
        'west': 270.0, 'w': 270.0, 'غرب': 270.0,
        'northwest': 315.0, 'nw': 315.0, 'شمال_غرب': 315.0,
    }
    
    direction_lower = str(direction).lower().strip()
    if direction_lower in direction_map:
        return direction_map[direction_lower]
    
    try:
        return float(direction)
    except ValueError:
        raise ValueError(f"اتجاه غير معروف / Unknown direction: {direction}")


def get_direction_name(yaw_deg: float) -> str:
    """الحصول على اسم الاتجاه"""
    yaw = yaw_deg % 360
    if yaw < 22.5 or yaw >= 337.5: return "شمال / North"
    elif yaw < 67.5: return "شمال شرق / NE"
    elif yaw < 112.5: return "شرق / East"
    elif yaw < 157.5: return "جنوب شرق / SE"
    elif yaw < 202.5: return "جنوب / South"
    elif yaw < 247.5: return "جنوب غرب / SW"
    elif yaw < 292.5: return "غرب / West"
    else: return "شمال غرب / NW"


def main():
    # قراءة ملف الإدخال
    input_file = Path(__file__).parent / "launch_input.yaml"
    
    if not input_file.exists():
        print("خطأ: ملف launch_input.yaml غير موجود!")
        print("Error: launch_input.yaml not found!")
        return
    
    with open(input_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # قراءة القيم
    pitch_deg = float(config.get('pitch_deg', 45))
    direction = config.get('direction', 0)
    roll_deg = float(config.get('roll_deg', 0))
    
    # تحويل الاتجاه
    yaw_deg = parse_direction(direction)
    
    # حساب quaternion
    quaternion = euler_to_quaternion(pitch_deg, yaw_deg, roll_deg)
    
    # عرض النتائج
    print("\n" + "=" * 60)
    print("نتيجة حساب اتجاه الإطلاق / Launch Direction Result")
    print("=" * 60)
    print(f"\nالمدخلات / Inputs:")
    print(f"  زاوية الإطلاق / Launch angle: {pitch_deg}°")
    print(f"  الاتجاه / Direction: {get_direction_name(yaw_deg)} ({yaw_deg}°)")
    if roll_deg != 0:
        print(f"  الدوران / Roll: {roll_deg}°")
    
    print(f"\n" + "-" * 60)
    print(f"النتيجة / Result:")
    print(f"-" * 60)
    print(f"\nQuaternion: [{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")
    
    print(f"\n" + "-" * 60)
    print(f"انسخ هذا السطر إلى ملف التكوين:")
    print(f"Copy this line to your config file:")
    print(f"-" * 60)
    print(f"\nattitude: [{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
