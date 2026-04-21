#!/usr/bin/env python3
"""
Interactive Simulation Viewer for 6-DOF Rocket Simulation
عارض المحاكاة التفاعلي لمحاكاة الصاروخ 6-DOF

This tool provides:
- Real-time visualization of simulation results
- Phase-based playback with adjustable speed
- Parameter editing with auto-save to main config
- Smart warnings for invalid parameter combinations

Usage:
    python tools/interactive_viewer.py                    # Interactive mode
    python tools/interactive_viewer.py --config FILE      # Use specific config
    python tools/interactive_viewer.py --phase APOGEE     # Start playback at specific flight phase

Author: Devin AI
Date: 2026-01-03
"""

import sys
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams, font_manager
from matplotlib.widgets import Button, Slider, RadioButtons, CheckButtons
from matplotlib.animation import FuncAnimation
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
import copy
import argparse
import tempfile

# ---------------------------------------------------------------------------
# Arabic font configuration for Matplotlib
# ضبط خط يدعم العربية في Matplotlib حتى تظهر النصوص العربية بشكل صحيح
# ---------------------------------------------------------------------------
# نحاول اختيار خط من عدة خطوط شائعة تدعم العربية (حسب المتوفر على النظام)
_PREFERRED_ARABIC_FONTS = [
    "Segoe UI",          # موجود غالباً على Windows ويدعم العربية
    "Tahoma",            # يدعم العربية
    "Arial",             # يدعم العربية
    "Noto Naskh Arabic", # من حزمة خطوط Noto (إذا كانت مثبتة)
    "Amiri",             # خط عربي شائع (إن وجد)
]

try:
    available_fonts = {f.name for f in font_manager.fontManager.ttflist}
    for _font_name in _PREFERRED_ARABIC_FONTS:
        if _font_name in available_fonts:
            rcParams["font.family"] = _font_name
            # تأكد من عرض علامة السالب بشكل صحيح مع Unicode
            rcParams["axes.unicode_minus"] = False
            break
except Exception:
    # إذا حدث خطأ، نستمر بالقيم الافتراضية بدون إيقاف البرنامج
    pass

# ---------------------------------------------------------------------------
# Arabic text shaping helper (optional, requires arabic_reshaper + python-bidi)
# أداة مساعدة لتشكيل النص العربي (اختياري، يتطلب arabic_reshaper + python-bidi)
# ---------------------------------------------------------------------------
try:
    import arabic_reshaper  # type: ignore
    from bidi.algorithm import get_display  # type: ignore
    _ARABIC_TEXT_SUPPORT = True
except Exception:
    _ARABIC_TEXT_SUPPORT = False


def _shape_arabic(text: str) -> str:
    """Return text with Arabic properly shaped when possible.

    تُعيد النص بعد تشكيل الحروف العربية واتجاه RTL إذا كانت المكتبات متوفرة.
    إذا لم تكن مكتبات التشكيل متاحة، تعيد النص كما هو بدون تغيير.
    """
    if not _ARABIC_TEXT_SUPPORT or not isinstance(text, str):
        return text

    try:
        import re

        # إذا لم يكن في النص أي حروف عربية، نعيده كما هو
        if not re.search(r"[\u0600-\u06FF]", text):
            return text

        # 1) تشكيل الحروف للاتصال الصحيح
        reshaped = arabic_reshaper.reshape(text)
        # 2) تطبيق خوارزمية bidi لعكس اتجاه السلسلة RTL داخل الشكل
        bidi_text = get_display(reshaped)
        return bidi_text
    except Exception:
        # في حالة أي خطأ، نعيد النص الأصلي حتى لا نتسبب في كسر الواجهة
        return text

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from rocket_6dof_sim import Rocket6DOFSimulation


def _flatten_dict(d: Dict, parent_key: str = '', sep: str = '.') -> Dict:
    """
    Flatten a nested dictionary into a flat dictionary with dot-separated keys.
    تحويل قاموس متداخل إلى قاموس مسطح بمفاتيح مفصولة بنقاط.
    
    Example:
        {'a': {'b': 1, 'c': 2}} -> {'a.b': 1, 'a.c': 2}
    
    Args:
        d: Nested dictionary to flatten
        parent_key: Prefix for keys (used in recursion)
        sep: Separator between nested keys
    
    Returns:
        Flat dictionary with dot-separated keys
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


class FlightPhaseNames:
    """Flight phase names for display and selection."""
    SAFE_ON_PAD = "SAFE_ON_PAD"
    PREFLIGHT = "PREFLIGHT"
    ARMED = "ARMED"
    LAUNCH = "LAUNCH"
    POWERED_ASCENT = "POWERED_ASCENT"
    BURNOUT = "BURNOUT"
    COAST_ASCENT = "COAST_ASCENT"
    NEAR_SPACE_ASCENT = "NEAR_SPACE_ASCENT"
    EXOATM_COAST = "EXOATM_COAST"
    APOGEE = "APOGEE"
    MIDCOURSE = "MIDCOURSE"
    TERMINAL = "TERMINAL"
    IMPACT_PREP = "IMPACT_PREP"
    IMPACT = "IMPACT"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    
    ALL_PHASES = [
        SAFE_ON_PAD, PREFLIGHT, ARMED, LAUNCH,
        POWERED_ASCENT, BURNOUT, COAST_ASCENT, NEAR_SPACE_ASCENT,
        EXOATM_COAST, APOGEE, MIDCOURSE, TERMINAL, IMPACT_PREP, IMPACT,
        MISSION_COMPLETE
    ]


# Phase-specific parameters that should be tuned for each flight phase
# المعاملات الخاصة بكل مرحلة طيران التي يجب ضبطها
PHASE_PARAMETERS = {
    'POWERED_ASCENT': {
        'name_ar': 'الصعود تحت الدفع',
        'description': 'Motor burning, initial pitch maneuver',
        'description_ar': 'المحرك يعمل، مناورة الميل الأولية',
        'parameters': [
            # Guidance pitch program parameters (used when guidance.mode includes pitch_program)
            # معاملات برنامج الميل للتوجيه (تُستخدم عندما يتضمن وضع التوجيه برنامج الميل)
            {'path': 'guidance.pitch_program.vertical_hold_s', 'name': 'Guidance Vertical Hold', 'name_ar': 'ثبات عمودي (توجيه)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.5, 'max': 5.0, 'step': 0.1},
            {'path': 'guidance.pitch_program.pitch_start_s', 'name': 'Guidance Pitch Start', 'name_ar': 'بدء الميل (توجيه)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 1.0, 'max': 10.0, 'step': 0.1},
            {'path': 'guidance.pitch_program.pitch_end_s', 'name': 'Guidance Pitch End', 'name_ar': 'انتهاء الميل (توجيه)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 10.0, 'max': 60.0, 'step': 1.0},
            {'path': 'guidance.pitch_program.pitch_initial_deg', 'name': 'Guidance Initial Pitch', 'name_ar': 'ميل أولي (توجيه)', 'unit': 'deg', 'unit_ar': 'درجة', 'min': 70.0, 'max': 90.0, 'step': 1.0},
            {'path': 'guidance.pitch_program.pitch_target_deg', 'name': 'Guidance Target Pitch', 'name_ar': 'ميل مستهدف (توجيه)', 'unit': 'deg', 'unit_ar': 'درجة', 'min': 20.0, 'max': 60.0, 'step': 1.0},
            # Simulation pitch program parameters (used when simulation.attitude_mode: "pitch_program")
            # معاملات برنامج الميل للمحاكاة (تُستخدم عندما يكون وضع الاتجاه "pitch_program")
            {'path': 'simulation.pitch_program.vertical_hold_s', 'name': 'Sim Vertical Hold', 'name_ar': 'ثبات عمودي (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.5, 'max': 5.0, 'step': 0.1},
            {'path': 'simulation.pitch_program.pitch_start_s', 'name': 'Sim Pitch Start', 'name_ar': 'بدء الميل (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 1.0, 'max': 10.0, 'step': 0.1},
            {'path': 'simulation.pitch_program.pitch_end_s', 'name': 'Sim Pitch End', 'name_ar': 'انتهاء الميل (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 10.0, 'max': 60.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.pitch_initial_deg', 'name': 'Sim Initial Pitch', 'name_ar': 'ميل أولي (محاكاة)', 'unit': 'deg', 'unit_ar': 'درجة', 'min': 70.0, 'max': 90.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.pitch_target_deg', 'name': 'Sim Target Pitch', 'name_ar': 'ميل مستهدف (محاكاة)', 'unit': 'deg', 'unit_ar': 'درجة', 'min': 20.0, 'max': 60.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.blend_start_s', 'name': 'Sim Blend Start', 'name_ar': 'بدء المزج (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 30.0, 'max': 100.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.blend_duration_s', 'name': 'Sim Blend Duration', 'name_ar': 'مدة المزج (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 1.0, 'max': 20.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.min_airspeed_m_s', 'name': 'Sim Min Airspeed', 'name_ar': 'سرعة جوية دنيا (محاكاة)', 'unit': 'm/s', 'unit_ar': 'م/ث', 'min': 10.0, 'max': 100.0, 'step': 5.0},
            # Autopilot gains
            # معاملات الطيار الآلي
            {'path': 'autopilot.gains.attitude.Kp_pitch', 'name': 'Pitch Kp', 'name_ar': 'معامل Kp للميل', 'unit': '-', 'unit_ar': '-', 'min': 0.1, 'max': 20.0, 'step': 0.1},
            {'path': 'autopilot.gains.attitude.Ki_pitch', 'name': 'Pitch Ki', 'name_ar': 'معامل Ki للميل', 'unit': '1/s', 'unit_ar': '1/ث', 'min': 0.0, 'max': 5.0, 'step': 0.1},
            {'path': 'autopilot.gains.attitude.Kd_pitch', 'name': 'Pitch Kd', 'name_ar': 'معامل Kd للميل', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.0, 'max': 10.0, 'step': 0.1},
        ]
    },
    'COAST_ASCENT': {
        'name_ar': 'الصعود بالقصور',
        'description': 'Coasting after burnout, velocity-aligned flight',
        'description_ar': 'الطيران بالقصور بعد انتهاء الوقود',
        'parameters': [
            # Guidance pitch program blend parameters
            # معاملات مزج برنامج الميل للتوجيه
            {'path': 'guidance.pitch_program.blend_start_s', 'name': 'Guidance Blend Start', 'name_ar': 'بدء المزج (توجيه)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 30.0, 'max': 100.0, 'step': 1.0},
            {'path': 'guidance.pitch_program.blend_duration_s', 'name': 'Guidance Blend Duration', 'name_ar': 'مدة المزج (توجيه)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 1.0, 'max': 20.0, 'step': 1.0},
            # Simulation pitch program blend parameters (used when simulation.attitude_mode: "pitch_program")
            # معاملات مزج برنامج الميل للمحاكاة
            {'path': 'simulation.pitch_program.blend_start_s', 'name': 'Sim Blend Start', 'name_ar': 'بدء المزج (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 30.0, 'max': 100.0, 'step': 1.0},
            {'path': 'simulation.pitch_program.blend_duration_s', 'name': 'Sim Blend Duration', 'name_ar': 'مدة المزج (محاكاة)', 'unit': 's', 'unit_ar': 'ثانية', 'min': 1.0, 'max': 20.0, 'step': 1.0},
            # Autopilot rate gains
            # معاملات معدل الطيار الآلي
            {'path': 'autopilot.gains.rate.Kp_pitch', 'name': 'Rate Pitch Kp', 'name_ar': 'معامل Kp لمعدل الميل', 'unit': '-', 'unit_ar': '-', 'min': 0.1, 'max': 50.0, 'step': 0.5},
            {'path': 'autopilot.gains.rate.Ki_pitch', 'name': 'Rate Pitch Ki', 'name_ar': 'معامل Ki لمعدل الميل', 'unit': '1/s', 'unit_ar': '1/ث', 'min': 0.0, 'max': 10.0, 'step': 0.1},
            {'path': 'autopilot.gains.rate.Kd_pitch', 'name': 'Rate Pitch Kd', 'name_ar': 'معامل Kd لمعدل الميل', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.0, 'max': 20.0, 'step': 0.1},
        ]
    },
    'APOGEE': {
        'name_ar': 'القمة',
        'description': 'Maximum altitude, transition to descent',
        'description_ar': 'أقصى ارتفاع، الانتقال للهبوط',
        'parameters': [
            {'path': 'guidance.composite.terminal_altitude_m', 'name': 'Terminal Switch Altitude', 'name_ar': 'ارتفاع التحول للمرحلة النهائية', 'unit': 'm', 'unit_ar': 'متر', 'min': 5000.0, 'max': 50000.0, 'step': 1000.0},
            {'path': 'guidance.composite.terminal_range_m', 'name': 'Terminal Switch Range', 'name_ar': 'مسافة التحول للمرحلة النهائية', 'unit': 'm', 'unit_ar': 'متر', 'min': 1000.0, 'max': 50000.0, 'step': 1000.0},
        ]
    },
    'MIDCOURSE': {
        'name_ar': 'المسار الأوسط',
        'description': 'Midcourse guidance phase',
        'description_ar': 'مرحلة التوجيه الأوسط',
        'parameters': [
            {'path': 'guidance.mode', 'name': 'Guidance Mode', 'name_ar': 'وضع التوجيه', 'unit': '-', 'unit_ar': '-', 'type': 'select', 'options': ['composite', 'velocity_aligned', 'gravity_turn', 'q_guidance']},
            {'path': 'autopilot.gains.attitude.Kp_yaw', 'name': 'Yaw Kp', 'name_ar': 'معامل Kp للانعراج', 'unit': '-', 'unit_ar': '-', 'min': 0.1, 'max': 20.0, 'step': 0.1},
            {'path': 'autopilot.gains.attitude.Ki_yaw', 'name': 'Yaw Ki', 'name_ar': 'معامل Ki للانعراج', 'unit': '1/s', 'unit_ar': '1/ث', 'min': 0.0, 'max': 5.0, 'step': 0.1},
            {'path': 'autopilot.gains.attitude.Kd_yaw', 'name': 'Yaw Kd', 'name_ar': 'معامل Kd للانعراج', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.0, 'max': 10.0, 'step': 0.1},
        ]
    },
    'TERMINAL': {
        'name_ar': 'المرحلة النهائية',
        'description': 'Terminal guidance phase',
        'description_ar': 'مرحلة التوجيه النهائي',
        'parameters': [
            {'path': 'guidance.proportional_navigation.use_true_pn', 'name': 'Use True PN', 'name_ar': 'استخدام True PN', 'unit': '-', 'unit_ar': '-', 'type': 'bool'},
            {'path': 'guidance.proportional_navigation.navigation_ratio', 'name': 'PN Gain (N)', 'name_ar': 'معامل التوجيه التناسبي', 'unit': '-', 'unit_ar': '-', 'min': 2.0, 'max': 6.0, 'step': 0.1},
            {'path': 'guidance.proportional_navigation.augmentation_enabled', 'name': 'Augmentation Enabled', 'name_ar': 'تفعيل التعزيز', 'unit': '-', 'unit_ar': '-', 'type': 'bool'},
        ]
    },
    'GENERAL': {
        'name_ar': 'إعدادات عامة',
        'description': 'General simulation settings',
        'description_ar': 'إعدادات المحاكاة العامة',
        'parameters': [
            {'path': 'simulation.duration', 'name': 'Duration', 'name_ar': 'مدة المحاكاة', 'unit': 's', 'unit_ar': 'ثانية', 'min': 10.0, 'max': 2000.0, 'step': 10.0},
            {'path': 'simulation.dt', 'name': 'Time Step', 'name_ar': 'خطوة الزمن', 'unit': 's', 'unit_ar': 'ثانية', 'min': 0.001, 'max': 0.1, 'step': 0.001},
            {'path': 'simulation.ballistic_mode', 'name': 'Ballistic Mode', 'name_ar': 'الوضع الباليستي', 'unit': '-', 'unit_ar': '-', 'type': 'bool'},
            {'path': 'guidance.enabled', 'name': 'Guidance Enabled', 'name_ar': 'تفعيل التوجيه', 'unit': '-', 'unit_ar': '-', 'type': 'bool'},
            {'path': 'autopilot.enabled', 'name': 'Autopilot Enabled', 'name_ar': 'تفعيل الطيار الآلي', 'unit': '-', 'unit_ar': '-', 'type': 'bool'},
            {'path': 'actuator.delta_max', 'name': 'Max Fin Deflection', 'name_ar': 'أقصى انحراف للزعانف', 'unit': 'deg', 'unit_ar': 'درجة', 'min': 5.0, 'max': 45.0, 'step': 1.0},
            {'path': 'actuator.rate_max', 'name': 'Max Fin Rate', 'name_ar': 'أقصى معدل للزعانف', 'unit': 'deg/s', 'unit_ar': 'درجة/ث', 'min': 50.0, 'max': 1000.0, 'step': 10.0},
        ]
    },
}


@dataclass
class Warning:
    """Warning message with severity and context."""
    severity: str  # 'info', 'warning', 'error'
    message: str
    message_ar: str
    time: Optional[float] = None
    phase: Optional[str] = None


@dataclass
class RunRecord:
    """
    Record of a simulation run for comparison.
    سجل تشغيل المحاكاة للمقارنة.
    """
    name: str  # Display name (e.g., "Ballistic", "Active Control")
    history: Dict  # Simulation history data
    config_snapshot: Dict  # Config used for this run
    max_range_km: float  # Maximum ground range achieved
    max_altitude_km: float  # Maximum altitude (apogee)
    flight_time_s: float  # Total flight time
    color: str = 'gray'  # Plot color
    linestyle: str = '--'  # Plot line style
    alpha: float = 0.6  # Plot transparency


class ConfigValidator:
    """
    Validates configuration parameters and generates warnings.
    مدقق الإعدادات وتوليد التحذيرات.
    """
    
    def __init__(self, config: Dict):
        self.config = config
        self.warnings: List[Warning] = []
    
    def validate_all(self) -> List[Warning]:
        """Run all validation checks."""
        self.warnings = []
        self._check_mode_consistency()
        self._check_pid_gains()
        self._check_actuator_limits()
        self._check_guidance_requirements()
        return self.warnings
    
    def _check_mode_consistency(self):
        """Check for contradictory mode settings."""
        sim_config = self.config.get('simulation', {})
        ballistic_mode = sim_config.get('ballistic_mode', False)
        
        guidance_enabled = self.config.get('guidance', {}).get('enabled', False)
        autopilot_enabled = self.config.get('autopilot', {}).get('enabled', False)
        
        if ballistic_mode and (guidance_enabled or autopilot_enabled):
            self.warnings.append(Warning(
                severity='error',
                message='Contradiction: ballistic_mode=true but guidance/autopilot enabled',
                message_ar='تناقض: وضع باليستي مفعل مع تفعيل التوجيه/الطيار الآلي'
            ))
        
        attitude_mode = sim_config.get('attitude_mode', 'none')
        if attitude_mode == 'closed_loop' and ballistic_mode:
            self.warnings.append(Warning(
                severity='error',
                message='Contradiction: closed_loop attitude with ballistic_mode',
                message_ar='تناقض: وضع التحكم المغلق مع الوضع الباليستي'
            ))
    
    def _check_pid_gains(self):
        """Check PID gain sanity."""
        autopilot = self.config.get('autopilot', {})
        gains = autopilot.get('gains', {})
        
        for loop_name in ['attitude', 'rate']:
            loop = gains.get(loop_name, {})
            for axis in ['pitch', 'yaw', 'roll']:
                kp = loop.get(f'Kp_{axis}', 0)
                ki = loop.get(f'Ki_{axis}', 0)
                kd = loop.get(f'Kd_{axis}', 0)
                
                if kp < 0 or ki < 0 or kd < 0:
                    self.warnings.append(Warning(
                        severity='error',
                        message=f'Negative PID gain in gains.{loop_name}.{axis}',
                        message_ar=f'معامل PID سالب في gains.{loop_name}.{axis}'
                    ))
                
                if kp > 100:
                    self.warnings.append(Warning(
                        severity='warning',
                        message=f'Very high Kp={kp} in gains.{loop_name}.{axis} may cause oscillation',
                        message_ar=f'معامل Kp={kp} مرتفع جداً في gains.{loop_name}.{axis} قد يسبب تذبذب'
                    ))
                
                if ki > 10:
                    self.warnings.append(Warning(
                        severity='warning',
                        message=f'High Ki={ki} in gains.{loop_name}.{axis} may cause integral windup',
                        message_ar=f'معامل Ki={ki} مرتفع في gains.{loop_name}.{axis} قد يسبب تراكم التكامل'
                    ))
    
    def _check_actuator_limits(self):
        """Check actuator configuration consistency."""
        actuator = self.config.get('actuator', {})
        max_deflection = actuator.get('delta_max', 25)
        max_rate = actuator.get('rate_max', 400)
        
        if max_deflection <= 0:
            self.warnings.append(Warning(
                severity='error',
                message=f'Invalid delta_max={max_deflection}',
                message_ar=f'قيمة غير صالحة delta_max={max_deflection}'
            ))
        
        if max_rate <= 0:
            self.warnings.append(Warning(
                severity='error',
                message=f'Invalid rate_max={max_rate}',
                message_ar=f'قيمة غير صالحة rate_max={max_rate}'
            ))
        
        if max_rate < 100:
            self.warnings.append(Warning(
                severity='warning',
                message=f'Low actuator rate {max_rate} deg/s may limit control authority',
                message_ar=f'معدل المشغل المنخفض {max_rate} درجة/ث قد يحد من سلطة التحكم'
            ))
    
    def _check_guidance_requirements(self):
        """Check guidance mode requirements."""
        guidance = self.config.get('guidance', {})
        if not guidance.get('enabled', False):
            return
        
        mode = guidance.get('mode', 'composite')
        
        # Check pitch program requirements
        if mode in ['pitch_program', 'composite']:
            pp = guidance.get('pitch_program', {})
            if not pp:
                self.warnings.append(Warning(
                    severity='warning',
                    message='pitch_program mode requires pitch_program settings',
                    message_ar='وضع pitch_program يتطلب إعدادات برنامج الميل'
                ))
            else:
                pitch_initial = pp.get('pitch_initial_deg', 90)
                pitch_target = pp.get('pitch_target_deg', 45)
                if pitch_initial < pitch_target:
                    self.warnings.append(Warning(
                        severity='warning',
                        message=f'Unusual pitch program: initial {pitch_initial}° < target {pitch_target}°',
                        message_ar=f'برنامج ميل غير عادي: البداية {pitch_initial}° < الهدف {pitch_target}°'
                    ))
        
        # Check PN guidance requirements
        if mode in ['pure_pn', 'true_pn', 'augmented_pn']:
            target = self.config.get('target', {})
            # Check for either position array or latitude/longitude coordinates
            has_position = target.get('position') is not None
            has_lat_lon = (target.get('latitude') is not None and 
                          target.get('longitude') is not None)
            if not has_position and not has_lat_lon:
                self.warnings.append(Warning(
                    severity='error',
                    message=f'{mode} guidance requires target position or lat/lon coordinates',
                    message_ar=f'توجيه {mode} يتطلب موقع الهدف أو إحداثيات خط العرض/الطول'
                ))
    


class RuntimeAnalyzer:
    """
    Analyzes simulation results for runtime warnings.
    محلل نتائج المحاكاة للتحذيرات أثناء التشغيل.
    """
    
    def __init__(self, history: Dict, config: Optional[Dict] = None):
        self.history = history
        self.config = config
        self.warnings: List[Warning] = []
    
    def analyze(self) -> List[Warning]:
        """Analyze simulation results for issues."""
        self.warnings = []
        self._check_actuator_saturation()
        self._check_control_oscillation()
        self._check_alpha_excursions()
        return self.warnings
    
    def _check_actuator_saturation(self):
        """Check for actuator saturation."""
        if 'autopilot_fin_deflections' not in self.history:
            return
        
        fin_deflections = np.array(self.history['autopilot_fin_deflections'])
        if fin_deflections.size == 0:
            return
        
        # Ensure 2D array shape for consistent processing
        # ضمان شكل مصفوفة ثنائية الأبعاد للمعالجة المتسقة
        if fin_deflections.ndim == 1:
            # Single fin or flattened data - reshape to (N, 1)
            fin_deflections = fin_deflections[:, np.newaxis]
        elif fin_deflections.ndim == 0:
            # Scalar value - skip analysis
            return
        
        # Read actuator limit from config, fallback to 25 deg if not found
        # قراءة حد الزعانف من الإعدادات، مع قيمة افتراضية 25 درجة
        default_limit_deg = 25.0
        if self.config is not None:
            max_limit_deg = self.config.get('actuator', {}).get('delta_max', default_limit_deg)
        else:
            max_limit_deg = default_limit_deg
        max_limit = np.radians(max_limit_deg)
        
        saturation_count = np.sum(np.abs(fin_deflections) > max_limit * 0.95)
        total_samples = fin_deflections.size
        
        if saturation_count > 0:
            saturation_pct = 100 * saturation_count / total_samples
            if saturation_pct > 10:
                self.warnings.append(Warning(
                    severity='warning',
                    message=f'Fin saturation detected: {saturation_pct:.1f}% of samples at limit ({max_limit_deg:.1f}°)',
                    message_ar=f'تشبع الزعانف: {saturation_pct:.1f}% من العينات عند الحد ({max_limit_deg:.1f}°)'
                ))
    
    def _check_control_oscillation(self):
        """Check for control oscillation by counting zero-crossings.
        
        فحص تذبذب التحكم عن طريق عد تقاطعات الصفر.
        """
        if 'autopilot_rate_error' not in self.history:
            return
        
        rate_errors = np.array(self.history['autopilot_rate_error'])
        if len(rate_errors) < 10:
            return
        
        # Normalize to 2D array: (N, M) where M is number of axes
        # توحيد الشكل إلى مصفوفة ثنائية الأبعاد
        if rate_errors.ndim == 1:
            # Single axis or scalar data - reshape to (N, 1)
            rate_errors = rate_errors[:, np.newaxis]
        
        n_samples = rate_errors.shape[0]
        n_axes = min(3, rate_errors.shape[1])
        
        # Axis names based on autopilot convention: [roll, pitch, yaw] = [p, q, r]
        # أسماء المحاور حسب اتفاقية الطيار الآلي
        if n_axes == 3:
            axis_names = ['roll', 'pitch', 'yaw']
        elif n_axes == 1:
            axis_names = ['rate']  # Generic name for single axis
        else:
            axis_names = [f'axis{i}' for i in range(n_axes)]
        
        # Check for zero-crossings (oscillation indicator)
        # فحص تقاطعات الصفر (مؤشر التذبذب)
        for axis in range(n_axes):
            axis_errors = rate_errors[:, axis]
            
            # Filter out NaN/Inf values
            valid_mask = np.isfinite(axis_errors)
            axis_errors = axis_errors[valid_mask]
            
            if len(axis_errors) < 10:
                continue
            
            # Count zero-crossings using product of consecutive samples
            # عد تقاطعات الصفر باستخدام حاصل ضرب العينات المتتالية
            # x[i] * x[i+1] < 0 indicates a sign change (zero crossing)
            zero_crossings = np.sum(axis_errors[:-1] * axis_errors[1:] < 0)
            
            # Calculate zero-crossing ratio (not frequency)
            # حساب نسبة تقاطعات الصفر (وليس التردد)
            zero_crossing_ratio = zero_crossings / (len(axis_errors) - 1)
            
            if zero_crossing_ratio > 0.3:  # More than 30% zero-crossings
                axis_name = axis_names[axis]
                self.warnings.append(Warning(
                    severity='warning',
                    message=f'High oscillation in {axis_name} rate control (zero-crossing ratio: {zero_crossing_ratio:.2f})',
                    message_ar=f'تذبذب عالي في تحكم معدل {axis_name} (نسبة تقاطع الصفر: {zero_crossing_ratio:.2f})'
                ))
    
    def _check_alpha_excursions(self):
        """Check for excessive angle of attack.
        
        Uses multiple heuristics to determine if alpha is in radians or degrees:
        1. Explicit 'alpha_unit' metadata in history (most reliable)
        2. Max value > π suggests degrees (simple threshold)
        3. Statistical analysis: radians typically have smaller variance
        
        يستخدم عدة طرق لتحديد ما إذا كانت زاوية الهجوم بالراديان أو الدرجات.
        """
        if 'alpha' not in self.history:
            return
        
        alpha = np.array(self.history['alpha'])
        if alpha.size == 0:
            return
        
        # Filter out NaN/Inf values for robust analysis
        # تصفية قيم NaN/Inf للتحليل القوي
        valid_mask = np.isfinite(alpha)
        alpha_valid = alpha[valid_mask]
        if len(alpha_valid) == 0:
            return
        
        # Determine if alpha is in radians or degrees
        # تحديد ما إذا كانت زاوية الهجوم بالراديان أو الدرجات
        is_radians = True  # Default assumption for scientific simulation
        
        # Method 1: Check for explicit unit metadata in history (most reliable)
        # الطريقة 1: التحقق من بيانات الوحدة الصريحة في السجل (الأكثر موثوقية)
        if 'alpha_unit' in self.history:
            unit = self.history['alpha_unit']
            if isinstance(unit, str):
                is_radians = unit.lower() in ('rad', 'radian', 'radians')
        else:
            # Method 2: Heuristic based on value range
            # الطريقة 2: استدلال بناءً على نطاق القيم
            max_val = np.max(np.abs(alpha_valid))
            
            # If max value > π (3.14), it's almost certainly degrees
            # إذا كانت القيمة القصوى أكبر من π، فهي بالتأكيد تقريباً درجات
            if max_val > np.pi:
                is_radians = False
            # If max value is very small (< 0.1 rad ≈ 5.7°), likely radians
            # إذا كانت القيمة القصوى صغيرة جداً، فمن المحتمل أنها راديان
            elif max_val < 0.1:
                is_radians = True
            # Ambiguous range (0.1 to π): use statistical heuristic
            # النطاق الغامض: استخدام استدلال إحصائي
            else:
                # Typical AoA in radians: std < 0.3 (≈17°)
                # Typical AoA in degrees: std > 5°
                std_val = np.std(alpha_valid)
                # If std is small relative to typical degree values, assume radians
                is_radians = std_val < 1.0
        
        if is_radians:
            alpha_deg = np.degrees(alpha_valid)
        else:
            alpha_deg = alpha_valid
        
        max_alpha = np.max(np.abs(alpha_deg))
        if max_alpha > 15:
            self.warnings.append(Warning(
                severity='warning',
                message=f'High angle of attack: max {max_alpha:.1f}°',
                message_ar=f'زاوية هجوم عالية: الحد الأقصى {max_alpha:.1f}°'
            ))
    
    pass
        # يدعم كلًّا من التمثيل أحادي البعد وثنائي/متعدد الأبعاد لأخطاء الموضع
class InteractiveViewer:
    """
    Interactive simulation viewer with matplotlib.
    عارض المحاكاة التفاعلي باستخدام matplotlib.
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """Initialize the interactive viewer."""
        # Set up paths
        self.base_dir = Path(__file__).parent.parent
        if config_file is None:
            self.config_file = self.base_dir / 'config' / '6dof_config_advanced.yaml'
        else:
            self.config_file = Path(config_file)
        
        # Load configuration
        self.config = self._load_config()
        self.original_config = copy.deepcopy(self.config)
        
        # Simulation state
        self.sim: Optional[Rocket6DOFSimulation] = None
        self.history: Optional[Dict] = None
        self.is_running = False
        self.current_index = 0
        self.playback_speed = 1.0
        self.is_paused = True
        
        # Animation
        self.anim: Optional[FuncAnimation] = None
        self.animation_interval = 50  # ms
        
        # Warnings
        self.config_warnings: List[Warning] = []
        self.runtime_warnings: List[Warning] = []
        
        # UI elements
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.markers = {}
        self.widgets = {}
        
        # Phase information
        self.phase_indices: Dict[str, Tuple[int, int]] = {}
        self.current_phase = None
        
        # Settings panel state
        self.settings_window = None
        self.selected_settings_phase = 'GENERAL'
        self.pending_changes: Dict[str, Any] = {}  # Track unsaved changes
        # Paths that were missing in the original configuration (dot-separated)
        self.missing_paths = set()
        
        # Trajectory comparison state
        self.reference_run: Optional[RunRecord] = None  # Reference trajectory for comparison
        self.show_reference: bool = True  # Toggle reference visibility
        self.reference_lines: Dict[str, Any] = {}  # Reference trajectory plot lines
        
        # Initial phase to jump to (set via --phase CLI argument)
        # المرحلة الأولية للانتقال إليها (تُضبط عبر وسيط --phase)
        self.initial_phase: Optional[str] = None
        
        # Validate parameter paths at startup
        self._validate_parameter_paths()
    
    def _load_config(self) -> Dict:
        """Load configuration from YAML file.
        
        Returns an empty dict if the file is empty, contains only comments,
        or if the root element is not a dict (e.g., list or string).
        This prevents NoneType/AttributeError when accessing config values later.
        
        يُرجع قاموساً فارغاً إذا كان الملف فارغاً أو يحتوي على تعليقات فقط
        أو إذا كان العنصر الجذر ليس قاموساً.
        """
        with open(self.config_file, 'r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f)
            # Ensure we always return a dict, even if YAML root is list/string/None
            # ضمان إرجاع قاموس دائماً، حتى لو كان جذر YAML قائمة/نص/None
            if not isinstance(loaded, dict):
                if loaded is not None:
                    print(f"[!] Warning: Config file root is not a dict (got {type(loaded).__name__}), using empty config")
                    print(f"[!] تحذير: جذر ملف الإعدادات ليس قاموساً (النوع: {type(loaded).__name__})، استخدام إعدادات فارغة")
                return {}
            return loaded
    
    def _validate_parameter_paths(self) -> List[Tuple[str, str, str]]:
        """Validate all parameter paths in PHASE_PARAMETERS exist in config.
        
        Returns list of (phase, path, status) tuples where status is 'OK' or 'NOT FOUND'.
        Prints warnings for missing paths at startup.

        Important:
            Distinguishes between "key missing" and "value is None".
            A parameter explicitly set to null in YAML is now treated as existing
            (status 'OK'), not as a missing path.
        """
        missing_paths = []
        valid_paths = []
        # Reset missing_paths tracking for this validation pass
        self.missing_paths = set()

        def _path_exists(path: str) -> Tuple[bool, Any]:
            """Return (exists, value) for a dot-separated path in self.config.

            This is similar to _get_parameter_value but preserves the distinction
            between a missing key and a key that exists with value None.
            """
            keys = path.split('.')
            obj: Any = self.config
            try:
                for key in keys:
                    obj = obj[key]
                return True, obj
            except (KeyError, TypeError):
                return False, None

        for phase, info in PHASE_PARAMETERS.items():
            for param in info['parameters']:
                path = param['path']
                exists, _ = _path_exists(path)
                if not exists:
                    missing_paths.append((phase, path, 'NOT FOUND'))
                    # Track this path so we can distinguish "missing" from "null" later
                    self.missing_paths.add(path)
                else:
                    valid_paths.append((phase, path, 'OK'))

        # Print warnings for missing paths
        if missing_paths:
            print("\n" + "="*60)
            print("[!] WARNING: Missing parameter paths / مسارات معاملات مفقودة")
            print("="*60)
            for phase, path, status in missing_paths:
                print(f"  [{phase}] {path} -> {status}")
            print("\nThese parameters will show as 'None' in the settings panel if their value is null.")
            print("هذه المعاملات ستظهر كـ 'None' في لوحة الإعدادات إذا كانت قيمتها null.")
            print("="*60 + "\n")

        return valid_paths + missing_paths
    
    def _save_config(self):
        """Save configuration to YAML file (WARNING: destroys comments)."""
        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
        print(f"Configuration saved to: {self.config_file}")
        print(f"تم حفظ الإعدادات في: {self.config_file}")
    
    def _save_config_overrides(self, changes: Dict[str, Any]) -> str:
        """Save only changed parameters to a separate override file.
        
        This preserves the main config file's comments and structure.
        Returns the path to the override file.
        """
        # Create override file path next to main config
        config_dir = os.path.dirname(self.config_file)
        override_file = os.path.join(config_dir, '6dof_config_overrides.yaml')
        
        # Load existing overrides if any
        existing_overrides = {}
        if os.path.exists(override_file):
            with open(override_file, 'r', encoding='utf-8') as f:
                existing_overrides = yaml.safe_load(f) or {}
        
        # Build nested structure from flat paths
        def set_nested(d: Dict, path: str, value: Any):
            keys = path.split('.')
            for key in keys[:-1]:
                d = d.setdefault(key, {})
            d[keys[-1]] = value
        
        # Merge new changes into existing overrides
        for path, value in changes.items():
            set_nested(existing_overrides, path, value)
        
        # Write override file with header comment
        with open(override_file, 'w', encoding='utf-8') as f:
            f.write("# ==========================================================================\n")
            f.write("# Configuration Overrides / إعدادات مخصصة\n")
            f.write("# ==========================================================================\n")
            f.write("# This file contains parameter overrides exported from interactive_viewer.py\n")
            f.write("# هذا الملف يحتوي على الإعدادات المصدرة من العارض التفاعلي\n")
            f.write("#\n")
            f.write("# To apply these overrides to the main config, either:\n")
            f.write("# لتطبيق هذه الإعدادات على الملف الرئيسي:\n")
            f.write("#   1. Manually copy values to 6dof_config_advanced.yaml\n")
            f.write("#      انسخ القيم يدوياً إلى الملف الرئيسي\n")
            f.write("#   2. Or run (passing this file as argument):\n")
            f.write("#      python tools/interactive_viewer.py --apply-overrides 6dof_config_overrides.yaml\n")
            f.write("#      أو استخدم الأمر أعلاه (مع مسار هذا الملف) لتطبيقها تلقائياً\n")
            f.write("# ==========================================================================\n\n")
            yaml.dump(existing_overrides, f, default_flow_style=False, allow_unicode=True)
        
        return override_file
    
    def _validate_config(self):
        """Validate configuration and generate warnings."""
        validator = ConfigValidator(self.config)
        self.config_warnings = validator.validate_all()
        return self.config_warnings
    
    def _analyze_results(self):
        """Analyze simulation results for runtime warnings."""
        if self.history is None:
            return []
        analyzer = RuntimeAnalyzer(self.history, self.config)
        self.runtime_warnings = analyzer.analyze()
        return self.runtime_warnings
    
    def _compute_phase_indices(self):
        """Compute start/end indices for each flight phase."""
        if self.history is None or 'flight_phase' not in self.history:
            return
        
        phases = self.history['flight_phase']
        self.phase_indices = {}
        
        current_phase = None
        start_idx = 0
        
        for i, phase in enumerate(phases):
            # Convert phase to string if it's an enum
            phase_str = str(phase) if not isinstance(phase, str) else phase
            # Extract phase name from enum string like "FlightPhase.POWERED_ASCENT"
            if '.' in phase_str:
                phase_str = phase_str.split('.')[-1]
            
            if phase_str != current_phase:
                if current_phase is not None:
                    self.phase_indices[current_phase] = (start_idx, i - 1)
                current_phase = phase_str
                start_idx = i
        
        # Add last phase
        if current_phase is not None:
            self.phase_indices[current_phase] = (start_idx, len(phases) - 1)
    
    def run_simulation(self):
        """Run the main simulation."""
        print("\n" + "="*60)
        print("Running simulation... / جاري تشغيل المحاكاة...")
        print("="*60)
        
        # Validate config first
        warnings = self._validate_config()
        if warnings:
            print("\nConfiguration warnings / تحذيرات الإعدادات:")
            for w in warnings:
                icon = {'error': '[X]', 'warning': '[!]', 'info': '[i]'}[w.severity]
                print(f"  {icon} {w.message}")
                print(f"      {w.message_ar}")
        
        # Check for errors
        errors = [w for w in warnings if w.severity == 'error']
        if errors:
            print("\nCannot run simulation due to configuration errors.")
            print("لا يمكن تشغيل المحاكاة بسبب أخطاء في الإعدادات.")
            return False
        
        # Track temp file for cleanup
        # تتبع الملف المؤقت للتنظيف
        temp_config_path = None
        
        try:
            # If there are pending changes, save to temp file and use that
            if self.pending_changes:
                fd, temp_config_path = tempfile.mkstemp(suffix='.yaml')
                os.chmod(temp_config_path, 0o600)
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
                print(f"  Using modified config with {len(self.pending_changes)} changes")
                print(f"  استخدام الإعدادات المعدلة مع {len(self.pending_changes)} تغييرات")
                self.sim = Rocket6DOFSimulation(config_file=temp_config_path)
            else:
                # Initialize simulation with original config file
                self.sim = Rocket6DOFSimulation(config_file=str(self.config_file))
            
            # Get simulation parameters
            sim_config = self.config.get('simulation', {})
            duration = sim_config.get('duration', 300.0)
            dt = sim_config.get('dt', 0.01)
            
            print(f"\nSimulation parameters / معاملات المحاكاة:")
            print(f"  Duration: {duration} s / المدة: {duration} ث")
            print(f"  Time step: {dt} s / خطوة الزمن: {dt} ث")
            
            # Define progress callback
            def progress_callback(info):
                print(f"\r  [{info['progress']:5.1f}%] t={info['time']:6.1f}s | "
                      f"alt={info['altitude']/1000:7.1f}km | "
                      f"phase={info['flight_phase']}", end="", flush=True)
            
            # Run simulation
            self.history = self.sim.simulate(
                duration=duration,
                dt=dt,
                on_step=progress_callback,
                callback_stride=100
            )
            print()  # New line after progress

            # Basic sanity check on history keys required by the viewer
            # التحقق من وجود المفاتيح الأساسية المطلوبة للعرض التفاعلي
            required_keys = ['time', 'position', 'velocity']
            missing_keys = [k for k in required_keys if k not in (self.history or {})]
            if missing_keys:
                print("\n[X] Simulation history is missing required keys for viewer.")
                print(f"    Missing keys / المفاتيح المفقودة: {missing_keys}")
                print("    لن يتم تشغيل العارض التفاعلي بسبب نقص بيانات المحاكاة.")
                # Clear history so that downstream code won't try to use it
                self.history = None
                return False
            
            # Compute phase indices
            self._compute_phase_indices()
            
            # Analyze results
            self._analyze_results()
            if self.runtime_warnings:
                print("\nRuntime warnings / تحذيرات التشغيل:")
                for w in self.runtime_warnings:
                    icon = {'error': '[X]', 'warning': '[!]', 'info': '[i]'}[w.severity]
                    print(f"  {icon} {w.message}")
                    print(f"      {w.message_ar}")
            
            # Print summary
            self._print_summary()
            
            return True
            
        except Exception as e:
            print(f"\nSimulation error: {e}")
            print(f"خطأ في المحاكاة: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # Clean up temp file if created
            # تنظيف الملف المؤقت إذا تم إنشاؤه
            if temp_config_path is not None:
                try:
                    os.remove(temp_config_path)
                except OSError:
                    pass  # Ignore cleanup errors
    
    def _print_summary(self):
        """Print simulation summary."""
        if self.history is None:
            return
        
        print("\n" + "="*60)
        print("Simulation Summary / ملخص المحاكاة")
        print("="*60)
        
        time = np.array(self.history['time'])
        position = np.array(self.history['position'])
        velocity = np.array(self.history['velocity'])
        
        # Calculate metrics
        if 'altitude_km' in self.history:
            altitude = np.array(self.history['altitude_km'])
            max_alt = np.max(altitude)
            print(f"  Max altitude: {max_alt:.2f} km / أقصى ارتفاع: {max_alt:.2f} كم")
        else:
            altitude = -position[:, 2] / 1000  # NED frame
            max_alt = np.max(altitude)
            print(f"  Max altitude: {max_alt:.2f} km / أقصى ارتفاع: {max_alt:.2f} كم")
        
        if 'ground_range_km' in self.history:
            ground_range = np.array(self.history['ground_range_km'])
            final_range = ground_range[-1]
            print(f"  Ground range: {final_range:.2f} km / المدى الأرضي: {final_range:.2f} كم")
        
        max_speed = np.max(np.linalg.norm(velocity, axis=1))
        print(f"  Max speed: {max_speed:.1f} m/s / أقصى سرعة: {max_speed:.1f} م/ث")
        
        flight_time = time[-1]
        print(f"  Flight time: {flight_time:.1f} s / زمن الطيران: {flight_time:.1f} ث")
        
        # Phase summary
        print("\n  Flight phases / مراحل الطيران:")
        for phase, (start, end) in self.phase_indices.items():
            t_start = time[start]
            t_end = time[end]
            print(f"    {phase}: {t_start:.1f}s - {t_end:.1f}s")
    
    def create_ui(self):
        """Create the matplotlib UI."""
        # Create figure with subplots
        self.fig = plt.figure(figsize=(18, 11))

        # Set window title when supported by the active backend
        # ضبط عنوان نافذة الرسم عندما يدعمه الـ backend الحالي لـ Matplotlib
        try:
            manager = getattr(self.fig.canvas, 'manager', None)
            if manager is not None and hasattr(manager, 'set_window_title'):
                manager.set_window_title(_shape_arabic('6-DOF Rocket Simulation Viewer / عارض محاكاة الصاروخ'))
        except Exception:
            # Silently ignore if backend does not support window titles
            # يتم تجاهل الخطأ بصمت إذا كان الـ backend لا يدعم عناوين النوافذ
            pass
        
        # Create grid for plots and controls (5 columns now to add settings panel)
        gs = self.fig.add_gridspec(4, 5, hspace=0.3, wspace=0.3,
                                   left=0.04, right=0.96, top=0.95, bottom=0.15)
        
        # Create axes for plots
        self.axes['trajectory'] = self.fig.add_subplot(gs[0:2, 0:2])
        self.axes['altitude'] = self.fig.add_subplot(gs[0, 2])
        self.axes['velocity'] = self.fig.add_subplot(gs[0, 3])
        self.axes['attitude'] = self.fig.add_subplot(gs[1, 2])
        self.axes['control'] = self.fig.add_subplot(gs[1, 3])
        self.axes['warnings'] = self.fig.add_subplot(gs[2:4, 0:2])
        self.axes['info'] = self.fig.add_subplot(gs[2:4, 2:3])
        # New settings panel on the right side
        self.axes['settings'] = self.fig.add_subplot(gs[0:4, 4])
        
        # Initialize plots
        self._init_plots()
        
        # Create control widgets
        self._create_widgets()
        
        # Set up animation
        self._setup_animation()
    
    def _init_plots(self):
        """Initialize all plots with data."""
        if self.history is None:
            return
        
        time = np.array(self.history['time'])
        position = np.array(self.history['position'])
        velocity = np.array(self.history['velocity'])
        
        # Trajectory plot (2D: range vs altitude)
        ax = self.axes['trajectory']
        ax.set_title(_shape_arabic('Trajectory / المسار'))
        ax.set_xlabel('Range (km) / المدى (كم)')
        ax.set_ylabel('Altitude (km) / الارتفاع (كم)')
        
        if 'ground_range_km' in self.history and 'altitude_km' in self.history:
            range_km = np.array(self.history['ground_range_km'])
            alt_km = np.array(self.history['altitude_km'])
        else:
            # NED frame: x=North, y=East, z=Down
            range_km = np.sqrt(position[:, 0]**2 + position[:, 1]**2) / 1000
            alt_km = -position[:, 2] / 1000
        
        self.lines['trajectory'], = ax.plot(range_km, alt_km, 'b-', linewidth=1.5, label='Current')
        self.markers['trajectory'], = ax.plot([], [], 'ro', markersize=10)
        # Initialize reference trajectory line (empty, will be populated when reference is set)
        self.reference_lines['trajectory'], = ax.plot([], [], 'gray', linestyle='--', 
                                                       linewidth=1, alpha=0.6, label='Reference')
        ax.grid(True, alpha=0.3)
        # Safe axis limits handling for empty arrays
        # معالجة آمنة لحدود المحاور للمصفوفات الفارغة
        max_range = np.max(range_km) if len(range_km) > 0 else 0
        max_alt = np.max(alt_km) if len(alt_km) > 0 else 0
        ax.set_xlim(0, max_range * 1.1 if max_range > 0 else 10)
        ax.set_ylim(0, max_alt * 1.1 if max_alt > 0 else 10)
        ax.legend(loc='upper right', fontsize=8)
        
        # Altitude plot
        ax = self.axes['altitude']
        ax.set_title(_shape_arabic('Altitude / الارتفاع'))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (km)')
        self.lines['altitude'], = ax.plot(time, alt_km, 'b-', linewidth=1)
        self.markers['altitude'], = ax.plot([], [], 'ro', markersize=8)
        # Reference altitude line
        self.reference_lines['altitude'], = ax.plot([], [], 'gray', linestyle='--', 
                                                     linewidth=1, alpha=0.6)
        ax.grid(True, alpha=0.3)
        
        # Velocity plot
        ax = self.axes['velocity']
        ax.set_title(_shape_arabic('Velocity / السرعة'))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        speed = np.linalg.norm(velocity, axis=1)
        self.lines['velocity'], = ax.plot(time, speed, 'g-', linewidth=1)
        self.markers['velocity'], = ax.plot([], [], 'ro', markersize=8)
        ax.grid(True, alpha=0.3)
        
        # Attitude plot
        ax = self.axes['attitude']
        ax.set_title(_shape_arabic('Attitude / الاتجاه'))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        
        if 'attitude' in self.history:
            attitude = np.array(self.history['attitude'])
            # Convert quaternion to Euler angles (simplified)
            # For visualization, just show pitch approximation
            if attitude.shape[1] == 4:
                # Quaternion: extract pitch from q
                q0, q1, q2, q3 = attitude[:, 0], attitude[:, 1], attitude[:, 2], attitude[:, 3]
                pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
                self.lines['attitude'], = ax.plot(time, pitch, 'm-', linewidth=1, label='Pitch')
            else:
                self.lines['attitude'], = ax.plot(time, attitude[:, 1], 'm-', linewidth=1, label='Pitch')
        else:
            self.lines['attitude'], = ax.plot([], [], 'm-', linewidth=1)
        self.markers['attitude'], = ax.plot([], [], 'ro', markersize=8)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        
        # Control plot (fin deflections)
        ax = self.axes['control']
        ax.set_title(_shape_arabic('Fin Deflections / انحرافات الزعانف'))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Deflection (deg)')
        
        if 'autopilot_fin_deflections' in self.history:
            fins = np.array(self.history['autopilot_fin_deflections'])
            if len(fins) > 0:
                fins_deg = np.degrees(fins)
                # Handle both 1D and 2D arrays
                if fins_deg.ndim == 2 and fins_deg.shape[1] > 0:
                    for i in range(min(4, fins_deg.shape[1])):
                        ax.plot(time[:len(fins_deg)], fins_deg[:, i], linewidth=0.5, alpha=0.7, label=f'Fin {i+1}')
                    ax.legend(loc='upper right', fontsize=6)
                elif fins_deg.ndim == 1:
                    ax.plot(time[:len(fins_deg)], fins_deg, linewidth=0.5, alpha=0.7, label='Fin 1')
                    ax.legend(loc='upper right', fontsize=6)
        self.markers['control'], = ax.plot([], [], 'ro', markersize=8)
        ax.grid(True, alpha=0.3)
        
        # Warnings panel
        ax = self.axes['warnings']
        ax.set_title(_shape_arabic('Warnings / التحذيرات'))
        ax.axis('off')
        self._update_warnings_display()
        
        # Info panel
        ax = self.axes['info']
        ax.set_title(_shape_arabic('Current State / الحالة الحالية'))
        ax.axis('off')
        self.info_text = ax.text(0.05, 0.95, '', transform=ax.transAxes,
                                  fontsize=10, verticalalignment='top',
                                  fontfamily='monospace')
        
        # Settings panel - initialize with default phase
        self._init_settings_panel()
    
    def _init_settings_panel(self):
        """Initialize the settings panel with phase-specific parameters."""
        ax = self.axes['settings']
        ax.clear()
        ax.set_title(_shape_arabic('Settings / الإعدادات'), fontsize=11, fontweight='bold')
        ax.axis('off')
        
        # Create text object for settings display
        self.settings_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                                      fontsize=8, verticalalignment='top',
                                      fontfamily='monospace', wrap=True)
        
        # Update with current phase settings
        self._update_settings_display()
    
    def _update_settings_display(self):
        """Update the settings panel with current phase parameters."""
        if 'settings' not in self.axes:
            return
        
        ax = self.axes['settings']
        phase = self.selected_settings_phase
        
        if phase not in PHASE_PARAMETERS:
            return
        
        phase_info = PHASE_PARAMETERS[phase]
        
        # Build settings text
        lines = []
        lines.append(f"Phase: {phase}")
        lines.append(f"{phase_info['name_ar']}")
        lines.append("-" * 30)
        lines.append(f"{phase_info['description']}")
        lines.append("")
        
        # Show unsaved changes indicator
        if self.pending_changes:
            lines.append(f"[*] Unsaved changes: {len(self.pending_changes)}")
            lines.append("[*] تغييرات غير محفوظة")
            lines.append("")
        
        lines.append("Parameters / المعاملات:")
        lines.append("")
        
        for i, param in enumerate(phase_info['parameters']):
            path = param['path']
            current_val = self._get_parameter_value(path)
            param_type = param.get('type', 'numeric')
            unit = param.get('unit', '')
            unit_ar = param.get('unit_ar', '')
            
            # Distinguish between a truly missing path and a present-but-null value
            if hasattr(self, 'missing_paths') and path in self.missing_paths:
                val_str = 'NOT FOUND / غير موجود'
            elif current_val is None:
                # Key exists but value is explicitly null in YAML
                val_str = 'null / فارغ'
            elif param_type == 'bool':
                val_str = 'true' if current_val else 'false'
            elif param_type == 'select':
                val_str = str(current_val)
            else:
                val_str = f"{current_val}"
            
            # Check if this parameter has pending changes
            is_modified = path in self.pending_changes
            modified_marker = " [*]" if is_modified else ""
            
            # Format with unit
            if unit and unit != '-':
                val_with_unit = f"{val_str} [{unit}]"
            else:
                val_with_unit = val_str
            
            lines.append(f"{i+1}. {param['name']}{modified_marker}")
            lines.append(f"   Value: {val_with_unit}")
            lines.append(f"   {param['name_ar']} ({unit_ar})")
            lines.append(f"   Path: {path}")
            lines.append("")
        
        lines.append("-" * 30)
        lines.append("Commands / الأوامر:")
        lines.append("  viewer.update_parameter(")
        lines.append("    'path', value)")
        lines.append("  viewer.undo_changes()")
        lines.append("  viewer.copy_path(n)")
        
        # Update text
        if hasattr(self, 'settings_text'):
            shaped_lines = [_shape_arabic(line) for line in lines]
            self.settings_text.set_text('\n'.join(shaped_lines))
        
        # Redraw
        if self.fig is not None:
            self.fig.canvas.draw_idle()
    
    def _update_warnings_display(self):
        """Update warnings display panel.

        Note:
            The panel shows a limited number of warnings for readability.
            If there are more warnings than can be displayed, a summary line
            is shown indicating that additional warnings exist.
        """
        if 'warnings' not in self.axes:
            return  # UI not initialized yet
        ax = self.axes['warnings']
        ax.clear()
        ax.set_title(_shape_arabic('Warnings / التحذيرات'))
        ax.axis('off')
        
        all_warnings = self.config_warnings + self.runtime_warnings
        if not all_warnings:
            ax.text(0.5, 0.5, _shape_arabic('No warnings / لا توجد تحذيرات'),
                   transform=ax.transAxes, ha='center', va='center',
                   fontsize=12, color='green')
            return
        
        max_display = 10
        total = len(all_warnings)
        y = 0.95
        
        for w in all_warnings[:max_display]:  # Show up to max_display warnings
            color = {'error': 'red', 'warning': 'orange', 'info': 'blue'}[w.severity]
            icon = {'error': '[X]', 'warning': '[!]', 'info': '[i]'}[w.severity]
            ax.text(0.02, y, _shape_arabic(f"{icon} {w.message}"), transform=ax.transAxes,
                   fontsize=9, color=color, verticalalignment='top')
            y -= 0.08
            ax.text(0.04, y, _shape_arabic(w.message_ar), transform=ax.transAxes,
                   fontsize=8, color=color, verticalalignment='top', alpha=0.8)
            y -= 0.1

        # If there are more warnings than displayed, show a small note at the bottom
        if total > max_display:
            ax.text(0.02, 0.02,
                    _shape_arabic(f"(+{total - max_display} more warnings not shown)\n(+{total - max_display} تحذيرات إضافية غير معروضة)"),
                    transform=ax.transAxes,
                    fontsize=8, color='gray', verticalalignment='bottom')
    
    def _create_widgets(self):
        """Create control widgets."""
        # Play/Pause button
        ax_play = self.fig.add_axes([0.02, 0.02, 0.05, 0.04])
        self.widgets['play'] = Button(ax_play, 'Play')
        self.widgets['play'].on_clicked(self._on_play_pause)
        
        # Reset button
        ax_reset = self.fig.add_axes([0.08, 0.02, 0.05, 0.04])
        self.widgets['reset'] = Button(ax_reset, 'Reset')
        self.widgets['reset'].on_clicked(self._on_reset)
        
        # Run Simulation button
        ax_run = self.fig.add_axes([0.14, 0.02, 0.07, 0.04])
        self.widgets['run'] = Button(ax_run, 'Run Sim')
        self.widgets['run'].on_clicked(self._on_run_simulation)
        
        # Save Reference button (saves current trajectory as reference)
        ax_save_ref = self.fig.add_axes([0.22, 0.02, 0.07, 0.04])
        self.widgets['save_ref'] = Button(ax_save_ref, 'Save Ref')
        self.widgets['save_ref'].on_clicked(self._on_save_reference)
        
        # Clear Reference button
        ax_clear_ref = self.fig.add_axes([0.30, 0.02, 0.07, 0.04])
        self.widgets['clear_ref'] = Button(ax_clear_ref, 'Clear Ref')
        self.widgets['clear_ref'].on_clicked(self._on_clear_reference)
        
        # Export button (saves to main config)
        ax_export = self.fig.add_axes([0.38, 0.02, 0.06, 0.04])
        self.widgets['export'] = Button(ax_export, 'Export')
        self.widgets['export'].on_clicked(self._on_export_settings)
        
        # Speed slider
        ax_speed = self.fig.add_axes([0.48, 0.02, 0.12, 0.03])
        self.widgets['speed'] = Slider(ax_speed, 'Speed', 0.1, 10.0,
                                        valinit=1.0, valstep=0.1)
        self.widgets['speed'].on_changed(self._on_speed_change)
        
        # Phase selector
        ax_phase = self.fig.add_axes([0.64, 0.01, 0.10, 0.08])
        phases = list(self.phase_indices.keys()) if self.phase_indices else ['No data']
        # Warn if there are more phases than can be displayed
        # تحذير إذا كان هناك مراحل أكثر مما يمكن عرضه
        max_display_phases = 6
        if len(phases) > max_display_phases:
            print(f"\n[!] Warning: {len(phases)} flight phases detected, showing first {max_display_phases} in selector.")
            print(f"    تحذير: تم اكتشاف {len(phases)} مراحل طيران، يتم عرض أول {max_display_phases} في المحدد.")
            print(f"    Hidden phases / المراحل المخفية: {phases[max_display_phases:]}")
            print(f"    Use time slider to navigate / استخدم شريط الوقت للتنقل")
        self.widgets['phase'] = RadioButtons(ax_phase, phases[:max_display_phases], active=0)
        self.widgets['phase'].on_clicked(self._on_phase_select)
        
        # Time slider
        ax_time = self.fig.add_axes([0.05, 0.08, 0.55, 0.03])

        # Robustly determine max_time, guarding against empty or missing time arrays
        # تحديد max_time بطريقة أكثر أماناً في حال كانت مصفوفة الزمن فارغة أو مفقودة
        if self.history and 'time' in self.history:
            times = self.history['time']
            try:
                # Handle list / numpy array / similar sequence types
                max_time = float(times[-1]) if len(times) > 0 else 100.0
            except Exception:
                max_time = 100.0
        else:
            max_time = 100.0

        self.widgets['time'] = Slider(ax_time, 'Time', 0, max_time,
                                       valinit=0, valstep=0.1)
        self.widgets['time'].on_changed(self._on_time_change)
        
        # Settings phase selector (in main window) - show ALL phases
        ax_settings_phase = self.fig.add_axes([0.76, 0.01, 0.10, 0.12])
        settings_phases = list(PHASE_PARAMETERS.keys())
        self.widgets['settings_phase'] = RadioButtons(ax_settings_phase, settings_phases, active=0)
        self.widgets['settings_phase'].on_clicked(self._on_settings_phase_change)
        
        # Edit Settings button - opens interactive settings editor
        ax_edit = self.fig.add_axes([0.88, 0.02, 0.06, 0.04])
        self.widgets['edit'] = Button(ax_edit, 'Edit')
        self.widgets['edit'].on_clicked(self._on_open_settings)
    
    def _setup_animation(self):
        """Set up the animation."""
        def update(frame):
            if self.history is None or self.is_paused:
                return []
            
            # Advance index based on speed
            stride = max(1, int(self.playback_speed * 10))
            self.current_index += stride
            
            if self.current_index >= len(self.history['time']):
                self.current_index = 0
                self.is_paused = True
                self.widgets['play'].label.set_text('Play')
            
            # Update time slider without triggering callback
            self.widgets['time'].eventson = False
            self.widgets['time'].set_val(self.history['time'][self.current_index])
            self.widgets['time'].eventson = True
            
            # Update plots
            self._update_plots()
            
            return list(self.markers.values())
        
        self.anim = FuncAnimation(self.fig, update, interval=self.animation_interval,
                                   blit=False, cache_frame_data=False)
    
    def _update_plots(self):
        """Update all plot markers to current index."""
        if self.history is None:
            return
        
        idx = self.current_index
        time = self.history['time'][idx]
        
        # Get current data
        position = np.array(self.history['position'])
        velocity = np.array(self.history['velocity'])
        
        if 'ground_range_km' in self.history and 'altitude_km' in self.history:
            range_km = self.history['ground_range_km'][idx]
            alt_km = self.history['altitude_km'][idx]
        else:
            range_km = np.sqrt(position[idx, 0]**2 + position[idx, 1]**2) / 1000
            alt_km = -position[idx, 2] / 1000
        
        speed = np.linalg.norm(velocity[idx])
        
        # Update trajectory marker
        self.markers['trajectory'].set_data([range_km], [alt_km])
        
        # Update altitude marker
        self.markers['altitude'].set_data([time], [alt_km])
        
        # Update velocity marker
        self.markers['velocity'].set_data([time], [speed])
        
        # Update attitude marker
        if 'attitude' in self.history:
            attitude = np.array(self.history['attitude'])
            if attitude.shape[1] == 4:
                q0, q1, q2, q3 = attitude[idx]
                pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
            else:
                pitch = attitude[idx, 1]
            self.markers['attitude'].set_data([time], [pitch])
        
        # Update info panel
        self._update_info_panel(idx)
        
        self.fig.canvas.draw_idle()
    
    def _update_info_panel(self, idx: int):
        """Update the info panel with current state."""
        if self.history is None:
            return
        
        time = self.history['time'][idx]
        position = np.array(self.history['position'])[idx]
        velocity = np.array(self.history['velocity'])[idx]
        
        # Get phase
        phase = "Unknown"
        if 'flight_phase' in self.history:
            phase_val = self.history['flight_phase'][idx]
            phase = str(phase_val).split('.')[-1] if '.' in str(phase_val) else str(phase_val)
        
        # Calculate derived values
        if 'altitude_km' in self.history:
            alt_km = self.history['altitude_km'][idx]
        else:
            alt_km = -position[2] / 1000
        
        if 'ground_range_km' in self.history:
            range_km = self.history['ground_range_km'][idx]
        else:
            range_km = np.sqrt(position[0]**2 + position[1]**2) / 1000
        
        speed = np.linalg.norm(velocity)
        
        # Get Mach if available
        mach_str = ""
        if 'mach' in self.history:
            mach = self.history['mach'][idx]
            mach_str = f"Mach:     {mach:.2f}\n"
        
        # Get dynamic pressure if available
        q_str = ""
        if 'q_dynamic' in self.history:
            q_dyn = self.history['q_dynamic'][idx]
            q_str = f"Q_dyn:    {q_dyn/1000:.1f} kPa\n"
        
        info = (
            f"Time:     {time:.2f} s\n"
            f"Phase:    {phase}\n"
            f"Altitude: {alt_km:.2f} km\n"
            f"Range:    {range_km:.2f} km\n"
            f"Speed:    {speed:.1f} m/s\n"
            f"{mach_str}"
            f"{q_str}"
        )
        
        self.info_text.set_text(info)
    
    def _on_play_pause(self, event):
        """Handle play/pause button click."""
        self.is_paused = not self.is_paused
        self.widgets['play'].label.set_text('Pause' if not self.is_paused else 'Play')
    
    def _on_reset(self, event):
        """Handle reset button click."""
        self.current_index = 0
        self.is_paused = True
        self.widgets['play'].label.set_text('Play')
        self.widgets['time'].set_val(0)
        self._update_plots()
    
    def _on_run_simulation(self, event):
        """Handle run simulation button click."""
        self.is_paused = True
        self.widgets['play'].label.set_text('Play')
        
        # Run simulation (updates self.history)
        if self.run_simulation():
            # Reset playback state
            self.current_index = 0
            
            # Refresh plots in place instead of closing/recreating figure
            # تحديث الرسوم البيانية في مكانها بدلاً من إغلاق/إعادة إنشاء النافذة
            self._refresh_plots()
            
            # Update reference trajectory overlay if exists
            self._update_reference_overlay()
            
            # Update warnings display
            self._update_warnings_display()
            
            # Redraw the canvas
            self.fig.canvas.draw_idle()
    
    def _refresh_plots(self):
        """Refresh all plots with new simulation data without recreating the figure.
        
        تحديث جميع الرسوم البيانية ببيانات المحاكاة الجديدة بدون إعادة إنشاء النافذة.
        This avoids calling plt.close()/plt.show() inside callbacks which can cause
        issues with some matplotlib backends.
        """
        if self.history is None:
            return
        
        time = np.array(self.history['time'])
        position = np.array(self.history['position'])
        velocity = np.array(self.history['velocity'])
        
        # Calculate range and altitude
        if 'ground_range_km' in self.history and 'altitude_km' in self.history:
            range_km = np.array(self.history['ground_range_km'])
            alt_km = np.array(self.history['altitude_km'])
        else:
            range_km = np.sqrt(position[:, 0]**2 + position[:, 1]**2) / 1000
            alt_km = -position[:, 2] / 1000
        
        # Update trajectory plot
        self.lines['trajectory'].set_data(range_km, alt_km)
        ax = self.axes['trajectory']
        # Safe axis limits handling for empty arrays
        # معالجة آمنة لحدود المحاور للمصفوفات الفارغة
        max_range = np.max(range_km) if len(range_km) > 0 else 0
        max_alt = np.max(alt_km) if len(alt_km) > 0 else 0
        ax.set_xlim(0, max_range * 1.1 if max_range > 0 else 10)
        ax.set_ylim(0, max_alt * 1.1 if max_alt > 0 else 10)
        
        # Update altitude plot
        self.lines['altitude'].set_data(time, alt_km)
        ax = self.axes['altitude']
        ax.relim()
        ax.autoscale_view()
        
        # Update velocity plot
        speed = np.linalg.norm(velocity, axis=1)
        self.lines['velocity'].set_data(time, speed)
        ax = self.axes['velocity']
        ax.relim()
        ax.autoscale_view()
        
        # Update attitude plot
        if 'attitude' in self.history:
            attitude = np.array(self.history['attitude'])
            if attitude.shape[1] == 4:
                # Quaternion to Euler pitch conversion with gimbal lock handling
                # تحويل الكواترنيون إلى زاوية الميل مع معالجة قفل الجيمبال
                q0, q1, q2, q3 = attitude[:, 0], attitude[:, 1], attitude[:, 2], attitude[:, 3]
                # Compute sin(pitch) with clipping to avoid numerical issues
                sin_pitch = np.clip(2*(q0*q2 - q3*q1), -1, 1)
                pitch = np.degrees(np.arcsin(sin_pitch))
                
                # Detect gimbal lock (pitch near ±90°) and warn
                # كشف قفل الجيمبال (الميل قريب من ±90°) وتحذير
                gimbal_lock_threshold = 0.99  # |sin(pitch)| > 0.99 means pitch > ~81.9°
                gimbal_lock_indices = np.abs(sin_pitch) > gimbal_lock_threshold
                if np.any(gimbal_lock_indices):
                    gimbal_lock_count = np.sum(gimbal_lock_indices)
                    print(f"[Warning] Gimbal lock detected at {gimbal_lock_count} samples "
                          f"(pitch near ±90°). Roll/yaw may be unreliable.")
                    print(f"[تحذير] تم اكتشاف قفل الجيمبال في {gimbal_lock_count} عينة "
                          f"(الميل قريب من ±90°). قد يكون الدوران/الانعراج غير موثوق.")
                
                self.lines['attitude'].set_data(time, pitch)
            else:
                self.lines['attitude'].set_data(time, attitude[:, 1])
            ax = self.axes['attitude']
            ax.relim()
            ax.autoscale_view()
        
        # Update control plot (fin deflections) - need to clear and redraw
        ax = self.axes['control']
        ax.clear()
        ax.set_title('Fin Deflections / انحرافات الزعانف')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Deflection (deg)')
        if 'autopilot_fin_deflections' in self.history:
            fins = np.array(self.history['autopilot_fin_deflections'])
            if fins.size > 0:
                fins_deg = np.degrees(fins)
                # Handle different array shapes
                # معالجة أشكال المصفوفات المختلفة
                if fins_deg.ndim == 1:
                    # Single fin or flattened data - plot as single line
                    ax.plot(time[:len(fins_deg)], fins_deg, linewidth=0.5, alpha=0.7, label='Fin 1')
                    ax.legend(loc='upper right', fontsize=6)
                elif fins_deg.ndim >= 2:
                    # Multiple fins - plot each column
                    n_fins = min(4, fins_deg.shape[1])
                    for i in range(n_fins):
                        ax.plot(time[:len(fins_deg)], fins_deg[:, i], linewidth=0.5, alpha=0.7, label=f'Fin {i+1}')
                    ax.legend(loc='upper right', fontsize=6)
        self.markers['control'], = ax.plot([], [], 'ro', markersize=8)
        ax.grid(True, alpha=0.3)
        
        # Update time slider range
        # تحديث نطاق شريط الوقت بطريقة آمنة
        max_time = float(time[-1]) if len(time) > 0 else 100.0
        time_slider = self.widgets['time']
        # Update slider axis limits for reliable behavior across matplotlib versions
        # تحديث حدود المحور للعمل بشكل موثوق عبر إصدارات matplotlib المختلفة
        time_slider.ax.set_xlim(0, max_time)
        time_slider.valmin = 0
        time_slider.valmax = max_time
        # Disable callbacks while setting value to prevent re-entrant updates
        # تعطيل الاستدعاءات أثناء تعيين القيمة لمنع التحديثات المتداخلة
        time_slider.eventson = False
        time_slider.set_val(0)
        time_slider.eventson = True
        
        # Update phase indices and phase selector
        self._compute_phase_indices()
        
        # Update settings display
        self._update_settings_display()
    
    def _on_save_reference(self, event):
        """Save current trajectory as reference for comparison."""
        if self.history is None:
            print("\nNo simulation data to save as reference / لا توجد بيانات محاكاة للحفظ كمرجع")
            return
        
        # Calculate metrics from current history
        position = np.array(self.history['position'])
        
        if 'ground_range_km' in self.history:
            range_km = np.array(self.history['ground_range_km'])
            max_range = np.max(range_km)
        else:
            range_km = np.sqrt(position[:, 0]**2 + position[:, 1]**2) / 1000
            max_range = np.max(range_km)
        
        if 'altitude_km' in self.history:
            alt_km = np.array(self.history['altitude_km'])
            max_alt = np.max(alt_km)
        else:
            alt_km = -position[:, 2] / 1000
            max_alt = np.max(alt_km)
        
        flight_time = self.history['time'][-1]
        
        # Determine run name based on config
        ballistic = self.config.get('simulation', {}).get('ballistic_mode', False)
        guidance_enabled = self.config.get('guidance', {}).get('enabled', False)
        if ballistic:
            name = "Ballistic"
        elif guidance_enabled:
            name = "Active Control"
        else:
            name = "Reference"
        
        # Create RunRecord
        self.reference_run = RunRecord(
            name=name,
            history=copy.deepcopy(self.history),
            config_snapshot=copy.deepcopy(self.config),
            max_range_km=max_range,
            max_altitude_km=max_alt,
            flight_time_s=flight_time,
            color='gray',
            linestyle='--',
            alpha=0.6
        )
        
        # Update the reference trajectory overlay
        self._update_reference_overlay()
        
        print(f"\n{'='*60}")
        print(f"Reference trajectory saved / تم حفظ المسار المرجعي")
        print(f"{'='*60}")
        print(f"  Name: {name}")
        print(f"  Max Range: {max_range:.2f} km / المدى الأقصى: {max_range:.2f} كم")
        print(f"  Max Altitude: {max_alt:.2f} km / أقصى ارتفاع: {max_alt:.2f} كم")
        print(f"  Flight Time: {flight_time:.1f} s / زمن الطيران: {flight_time:.1f} ث")
        print(f"\nRun a new simulation to compare / شغل محاكاة جديدة للمقارنة")
    
    def _on_clear_reference(self, event):
        """Clear the reference trajectory."""
        if self.reference_run is None:
            print("\nNo reference trajectory to clear / لا يوجد مسار مرجعي للمسح")
            return
        
        self.reference_run = None
        
        # Clear reference lines
        for key in self.reference_lines:
            self.reference_lines[key].set_data([], [])
        
        self.fig.canvas.draw_idle()
        
        print("\nReference trajectory cleared / تم مسح المسار المرجعي")
    
    def _update_reference_overlay(self):
        """Update the reference trajectory overlay on plots."""
        if self.reference_run is None:
            return
        
        # Also check that current history exists for comparison
        # التحقق أيضاً من وجود السجل الحالي للمقارنة
        if self.history is None:
            return
        
        ref_history = self.reference_run.history
        ref_position = np.array(ref_history['position'])
        ref_time = np.array(ref_history['time'])
        
        # Get reference trajectory data
        if 'ground_range_km' in ref_history and 'altitude_km' in ref_history:
            ref_range_km = np.array(ref_history['ground_range_km'])
            ref_alt_km = np.array(ref_history['altitude_km'])
        else:
            ref_range_km = np.sqrt(ref_position[:, 0]**2 + ref_position[:, 1]**2) / 1000
            ref_alt_km = -ref_position[:, 2] / 1000
        
        # Update trajectory overlay
        if 'trajectory' in self.reference_lines:
            self.reference_lines['trajectory'].set_data(ref_range_km, ref_alt_km)
            # Adjust axis limits to include both trajectories
            ax = self.axes['trajectory']
            if self.history is not None:
                if 'ground_range_km' in self.history and 'altitude_km' in self.history:
                    curr_range = np.array(self.history['ground_range_km'])
                    curr_alt = np.array(self.history['altitude_km'])
                else:
                    curr_pos = np.array(self.history['position'])
                    curr_range = np.sqrt(curr_pos[:, 0]**2 + curr_pos[:, 1]**2) / 1000
                    curr_alt = -curr_pos[:, 2] / 1000
                
                max_range = max(np.max(curr_range), np.max(ref_range_km))
                max_alt = max(np.max(curr_alt), np.max(ref_alt_km))
                ax.set_xlim(0, max_range * 1.1)
                ax.set_ylim(0, max_alt * 1.1)
        
        # Update altitude overlay
        if 'altitude' in self.reference_lines:
            self.reference_lines['altitude'].set_data(ref_time, ref_alt_km)
        
        # Update comparison info in info panel
        self._update_comparison_display()
        
        self.fig.canvas.draw_idle()
    
    def _analyze_trajectory_divergence(self) -> Optional[Dict]:
        """
        Analyze where trajectories diverge most and suggest which phase to tune.
        تحليل أين يتباعد المساران أكثر واقتراح المرحلة المطلوب ضبطها.
        
        Returns dict with:
            - worst_phase: phase with largest cumulative deviation
            - phase_deviations: dict of phase -> (mean_dev, max_dev, dev_type)
            - peak_idx: index of maximum instantaneous deviation
            - peak_phase: phase at peak deviation
            - suggested_params: list of parameters to tune
        """
        if self.reference_run is None or self.history is None:
            return None
        
        ref_history = self.reference_run.history
        
        # Get current trajectory data
        curr_time = np.array(self.history['time'])
        curr_pos = np.array(self.history['position'])
        if 'ground_range_km' in self.history and 'altitude_km' in self.history:
            curr_range = np.array(self.history['ground_range_km'])
            curr_alt = np.array(self.history['altitude_km'])
        else:
            curr_range = np.sqrt(curr_pos[:, 0]**2 + curr_pos[:, 1]**2) / 1000
            curr_alt = -curr_pos[:, 2] / 1000
        
        # Get reference trajectory data
        ref_time = np.array(ref_history['time'])
        ref_pos = np.array(ref_history['position'])
        if 'ground_range_km' in ref_history and 'altitude_km' in ref_history:
            ref_range = np.array(ref_history['ground_range_km'])
            ref_alt = np.array(ref_history['altitude_km'])
        else:
            ref_range = np.sqrt(ref_pos[:, 0]**2 + ref_pos[:, 1]**2) / 1000
            ref_alt = -ref_pos[:, 2] / 1000
        
        # Align by time - use current time as base, interpolate reference
        t_end = min(curr_time[-1], ref_time[-1])
        t_mask = curr_time <= t_end
        t_common = curr_time[t_mask]
        
        # Interpolate reference to match current time points
        ref_range_interp = np.interp(t_common, ref_time, ref_range)
        ref_alt_interp = np.interp(t_common, ref_time, ref_alt)
        
        # Compute deviations
        dr = curr_range[t_mask] - ref_range_interp  # range deviation
        dh = curr_alt[t_mask] - ref_alt_interp  # altitude deviation
        d_total = np.sqrt(dr**2 + dh**2)  # combined deviation in km
        
        # Get flight phases for current trajectory
        if 'flight_phase' not in self.history:
            return None
        
        phases = self.history['flight_phase'][:len(t_common)]
        
        # Compute deviation per phase
        phase_deviations = {}
        for i, phase in enumerate(phases):
            # Normalize phase name
            phase_str = str(phase) if not isinstance(phase, str) else phase
            if '.' in phase_str:
                phase_str = phase_str.split('.')[-1]
            
            if phase_str not in phase_deviations:
                phase_deviations[phase_str] = {
                    'range_devs': [],
                    'alt_devs': [],
                    'total_devs': []
                }
            
            phase_deviations[phase_str]['range_devs'].append(abs(dr[i]))
            phase_deviations[phase_str]['alt_devs'].append(abs(dh[i]))
            phase_deviations[phase_str]['total_devs'].append(d_total[i])
        
        # Calculate statistics per phase
        phase_stats = {}
        for phase, devs in phase_deviations.items():
            if len(devs['total_devs']) > 0:
                mean_dev = np.mean(devs['total_devs'])
                max_dev = np.max(devs['total_devs'])
                mean_range_dev = np.mean(devs['range_devs'])
                mean_alt_dev = np.mean(devs['alt_devs'])
                
                # Determine if deviation is mostly range or altitude
                if mean_range_dev > mean_alt_dev * 1.5:
                    dev_type = 'range'
                elif mean_alt_dev > mean_range_dev * 1.5:
                    dev_type = 'altitude'
                else:
                    dev_type = 'both'
                
                phase_stats[phase] = {
                    'mean_dev': mean_dev,
                    'max_dev': max_dev,
                    'dev_type': dev_type,
                    'mean_range_dev': mean_range_dev,
                    'mean_alt_dev': mean_alt_dev
                }
        
        # Find worst phase (highest mean deviation)
        worst_phase = max(phase_stats.keys(), key=lambda p: phase_stats[p]['mean_dev']) if phase_stats else None
        
        # Find peak deviation index and phase
        peak_idx = np.argmax(d_total)
        peak_phase_raw = phases[peak_idx]
        peak_phase = str(peak_phase_raw).split('.')[-1] if '.' in str(peak_phase_raw) else str(peak_phase_raw)
        
        # Get suggested parameters for worst phase
        suggested_params = []
        if worst_phase and worst_phase in PHASE_PARAMETERS:
            params = PHASE_PARAMETERS[worst_phase]['parameters']
            suggested_params = [p['name'] for p in params[:5]]  # Top 5 params
        elif worst_phase:
            # Map to closest PHASE_PARAMETERS key
            phase_mapping = {
                'POWERED_ASCENT': 'POWERED_ASCENT',
                'BURNOUT': 'POWERED_ASCENT',
                'COAST_ASCENT': 'COAST_ASCENT',
                'NEAR_SPACE_ASCENT': 'COAST_ASCENT',
                'EXOATM_COAST': 'COAST_ASCENT',
                'APOGEE': 'APOGEE',
                'MIDCOURSE': 'MIDCOURSE',
                'REENTRY': 'TERMINAL',
                'DESCENT': 'TERMINAL',
                'TERMINAL': 'TERMINAL',
                'IMPACT': 'TERMINAL',
            }
            mapped_phase = phase_mapping.get(worst_phase, 'GENERAL')
            if mapped_phase in PHASE_PARAMETERS:
                params = PHASE_PARAMETERS[mapped_phase]['parameters']
                suggested_params = [p['name'] for p in params[:5]]
        
        return {
            'worst_phase': worst_phase,
            'phase_stats': phase_stats,
            'peak_idx': peak_idx,
            'peak_phase': peak_phase,
            'peak_deviation_km': d_total[peak_idx],
            'suggested_params': suggested_params
        }
    
    def _update_comparison_display(self):
        """Update the comparison metrics display."""
        if self.reference_run is None or self.history is None:
            return
        
        # Calculate current metrics
        if 'ground_range_km' in self.history:
            curr_range = np.max(np.array(self.history['ground_range_km']))
        else:
            curr_pos = np.array(self.history['position'])
            curr_range = np.max(np.sqrt(curr_pos[:, 0]**2 + curr_pos[:, 1]**2) / 1000)
        
        if 'altitude_km' in self.history:
            curr_alt = np.max(np.array(self.history['altitude_km']))
        else:
            curr_pos = np.array(self.history['position'])
            curr_alt = np.max(-curr_pos[:, 2] / 1000)
        
        ref_range = self.reference_run.max_range_km
        ref_alt = self.reference_run.max_altitude_km
        
        # Calculate differences
        range_diff = curr_range - ref_range
        range_pct = (range_diff / ref_range * 100) if ref_range > 0 else 0
        alt_diff = curr_alt - ref_alt
        alt_pct = (alt_diff / ref_alt * 100) if ref_alt > 0 else 0
        
        # Print comparison
        print(f"\n{'='*60}")
        print("Trajectory Comparison / مقارنة المسارات")
        print(f"{'='*60}")
        print(f"                    Current    Reference    Diff")
        print(f"  Range (km):       {curr_range:7.2f}    {ref_range:7.2f}    {range_diff:+7.2f} ({range_pct:+.1f}%)")
        print(f"  Altitude (km):    {curr_alt:7.2f}    {ref_alt:7.2f}    {alt_diff:+7.2f} ({alt_pct:+.1f}%)")
        print(f"{'='*60}")
        
        # Analyze divergence and suggest tuning
        analysis = self._analyze_trajectory_divergence()
        if analysis:
            worst_phase = analysis['worst_phase']
            peak_phase = analysis['peak_phase']
            peak_dev = analysis['peak_deviation_km']
            suggested_params = analysis['suggested_params']
            phase_stats = analysis['phase_stats']
            
            print(f"\n{'='*60}")
            print("Tuning Suggestions / اقتراحات الضبط")
            print(f"{'='*60}")
            
            # Show phase with largest deviation
            if worst_phase and worst_phase in phase_stats:
                stats = phase_stats[worst_phase]
                dev_type_ar = {'range': 'المدى', 'altitude': 'الارتفاع', 'both': 'كلاهما'}
                print(f"\n  Largest deviation in phase / أكبر انحراف في مرحلة:")
                print(f"    {worst_phase}")
                print(f"    Mean deviation: {stats['mean_dev']:.2f} km / متوسط الانحراف: {stats['mean_dev']:.2f} كم")
                print(f"    Deviation type: {stats['dev_type']} / نوع الانحراف: {dev_type_ar.get(stats['dev_type'], stats['dev_type'])}")
            
            # Show peak deviation info
            if peak_phase != worst_phase:
                print(f"\n  Peak instantaneous deviation / أقصى انحراف لحظي:")
                print(f"    Phase: {peak_phase} / المرحلة: {peak_phase}")
                print(f"    Deviation: {peak_dev:.2f} km / الانحراف: {peak_dev:.2f} كم")
            
            # Show suggested parameters
            if suggested_params:
                print(f"\n  Suggested parameters to tune / المعاملات المقترح ضبطها:")
                for param in suggested_params:
                    print(f"    - {param}")
            
            # Auto-select the suggested phase in settings panel
            if worst_phase:
                # Map to PHASE_PARAMETERS key
                phase_mapping = {
                    'POWERED_ASCENT': 'POWERED_ASCENT',
                    'BURNOUT': 'POWERED_ASCENT',
                    'COAST_ASCENT': 'COAST_ASCENT',
                    'NEAR_SPACE_ASCENT': 'COAST_ASCENT',
                    'EXOATM_COAST': 'COAST_ASCENT',
                    'APOGEE': 'APOGEE',
                    'MIDCOURSE': 'MIDCOURSE',
                    'REENTRY': 'TERMINAL',
                    'DESCENT': 'TERMINAL',
                    'TERMINAL': 'TERMINAL',
                    'IMPACT': 'TERMINAL',
                }
                mapped_phase = phase_mapping.get(worst_phase, 'GENERAL')
                if mapped_phase in PHASE_PARAMETERS:
                    self.selected_settings_phase = mapped_phase
                    self._update_settings_display()
                    print(f"\n  Settings panel switched to: {mapped_phase}")
                    print(f"  تم تبديل لوحة الإعدادات إلى: {mapped_phase}")
            
            print(f"{'='*60}")
    
    def _on_speed_change(self, val):
        """Handle speed slider change."""
        self.playback_speed = val
    
    def _on_phase_select(self, label):
        """Handle phase selection."""
        self.jump_to_phase(label)
    
    def jump_to_phase(self, phase: str) -> bool:
        """Jump playback to the start of a specific flight phase.
        
        الانتقال إلى بداية مرحلة طيران محددة.
        
        Args:
            phase: Flight phase name (e.g., 'POWERED_ASCENT', 'APOGEE', 'TERMINAL')
                   اسم مرحلة الطيران
        
        Returns:
            True if jump was successful, False if phase not found
            True إذا نجح الانتقال، False إذا لم يتم العثور على المرحلة
        """
        if phase not in self.phase_indices:
            print(f"Phase '{phase}' not found in simulation results.")
            print(f"المرحلة '{phase}' غير موجودة في نتائج المحاكاة.")
            print(f"Available phases / المراحل المتاحة: {list(self.phase_indices.keys())}")
            return False
        
        start_idx, end_idx = self.phase_indices[phase]
        self.current_index = start_idx
        
        # Update time slider if widgets are initialized
        if 'time' in self.widgets and self.history is not None:
            self.widgets['time'].set_val(self.history['time'][start_idx])
        
        # Update plots if UI is initialized
        if self.fig is not None:
            self._update_plots()
        
        print(f"Jumped to phase: {phase} (index {start_idx})")
        print(f"تم الانتقال إلى المرحلة: {phase} (الفهرس {start_idx})")
        return True
    
    def _on_time_change(self, val):
        """Handle time slider change."""
        if self.history is None:
            return
        
        # Find closest index
        times = np.array(self.history['time'])
        self.current_index = np.argmin(np.abs(times - val))
        self._update_plots()
    
    def _on_settings_phase_change(self, label):
        """Handle settings phase selector change - updates GUI display."""
        self.selected_settings_phase = label
        # Update the settings panel in the GUI (non-blocking)
        self._update_settings_display()
    
    def _on_open_settings(self, event):
        """Open interactive settings editor window with sliders."""
        phase = self.selected_settings_phase
        if phase not in PHASE_PARAMETERS:
            print(f"Unknown phase: {phase}")
            return
        
        phase_info = PHASE_PARAMETERS[phase]
        params = phase_info['parameters']
        
        # Create a new figure for the settings editor
        self.settings_fig = plt.figure(figsize=(10, 8))
        self.settings_fig.suptitle(f'Settings Editor / محرر الإعدادات\n{phase} - {phase_info["name_ar"]}', 
                                    fontsize=14, fontweight='bold')
        
        # Store slider widgets
        self.param_sliders = {}
        self.param_textboxes = {}
        
        # Calculate layout
        n_params = len(params)
        height_per_param = 0.08
        start_y = 0.85
        
        for i, param in enumerate(params):
            current_val = self._get_parameter_value(param['path'])
            param_type = param.get('type', 'numeric')
            
            y_pos = start_y - i * height_per_param
            
            # Parameter label
            self.settings_fig.text(0.02, y_pos + 0.02, 
                                   f"{param['name']} / {param['name_ar']}", 
                                   fontsize=9, verticalalignment='top')
            
            if param_type == 'numeric':
                # Create slider for numeric parameters
                min_val = param.get('min', 0)
                max_val = param.get('max', 100)
                
                # Handle None values
                if current_val is None:
                    current_val = (min_val + max_val) / 2
                
                ax_slider = self.settings_fig.add_axes([0.35, y_pos - 0.02, 0.45, 0.025])
                slider = Slider(ax_slider, '', min_val, max_val, valinit=float(current_val))
                
                # Create callback for this parameter
                # Use skip_validation=True for better performance during slider drag
                # استخدام skip_validation=True لأداء أفضل أثناء سحب الشريط
                def make_slider_callback(path, slider_widget):
                    def callback(val):
                        self.update_parameter(path, val, skip_validation=True)
                    return callback
                
                slider.on_changed(make_slider_callback(param['path'], slider))
                self.param_sliders[param['path']] = slider
                
                # Show current value
                self.settings_fig.text(0.82, y_pos, f"{current_val}", fontsize=9)
                
            elif param_type == 'bool':
                # Create checkbox for boolean parameters
                ax_check = self.settings_fig.add_axes([0.35, y_pos - 0.02, 0.1, 0.03])
                check = CheckButtons(ax_check, [''], [bool(current_val) if current_val is not None else False])
                
                # Use skip_validation=True for better performance during toggle
                # استخدام skip_validation=True لأداء أفضل أثناء التبديل
                def make_check_callback(path):
                    def callback(label):
                        current = self._get_parameter_value(path)
                        # Handle None as False for proper toggle behavior
                        # معالجة None كـ False للحصول على سلوك تبديل صحيح
                        current_bool = bool(current) if current is not None else False
                        self.update_parameter(path, not current_bool, skip_validation=True)
                    return callback
                
                check.on_clicked(make_check_callback(param['path']))
                self.param_sliders[param['path']] = check
                
            elif param_type == 'select':
                # Show options as text for select parameters
                options = param.get('options', [])
                self.settings_fig.text(0.35, y_pos, 
                                       f"Current: {current_val} (Options: {', '.join(options)})", 
                                       fontsize=8)
            
            elif param_type == 'array':
                # Handle array parameters (e.g., noise_std vectors [x, y, z])
                # معالجة معاملات المصفوفات (مثل متجهات الضوضاء)
                is_array = isinstance(current_val, (list, tuple)) and len(current_val) > 0
                if is_array:
                    # Use the first element for the slider, apply to all elements
                    # استخدام العنصر الأول للشريط، وتطبيقه على جميع العناصر
                    first_val = float(current_val[0])
                    array_len = len(current_val)
                elif isinstance(current_val, (int, float)):
                    # Handle scalar values - keep as scalar
                    # معالجة القيم المفردة - الإبقاء عليها كقيمة مفردة
                    first_val = float(current_val)
                    array_len = None  # Indicates scalar value
                else:
                    first_val = param.get('min', 0)
                    array_len = param.get('array_len', 3)  # Use param-defined length or default to 3
                
                min_val = param.get('min', 0)
                max_val = param.get('max', 100)
                
                ax_slider = self.settings_fig.add_axes([0.35, y_pos - 0.02, 0.45, 0.025])
                slider = Slider(ax_slider, '', min_val, max_val, valinit=first_val)
                
                # Create callback that preserves the original type (array or scalar)
                # Use skip_validation=True for better performance during slider drag
                # استخدام skip_validation=True لأداء أفضل أثناء سحب الشريط
                def make_array_slider_callback(path, arr_len, was_array):
                    def callback(val):
                        if was_array and arr_len is not None:
                            # Create array with same value for all elements
                            new_array = [val] * arr_len
                            self.update_parameter(path, new_array, skip_validation=True)
                        else:
                            # Keep as scalar value
                            self.update_parameter(path, val, skip_validation=True)
                    return callback
                
                slider.on_changed(make_array_slider_callback(param['path'], array_len, is_array))
                self.param_sliders[param['path']] = slider
                
                # Show current array value
                if isinstance(current_val, (list, tuple)):
                    display_val = f"[{', '.join(f'{v:.4f}' for v in current_val)}]"
                else:
                    display_val = str(current_val)
                self.settings_fig.text(0.82, y_pos, display_val, fontsize=7)
        
        # Add Apply & Close button
        ax_close = self.settings_fig.add_axes([0.4, 0.02, 0.2, 0.05])
        btn_close = Button(ax_close, 'Close / إغلاق')
        
        def close_settings(event):
            plt.close(self.settings_fig)
            self._update_settings_display()
            self.fig.canvas.draw_idle()
        
        btn_close.on_clicked(close_settings)
        
        # Show the settings window
        plt.show(block=False)
        print(f"\nSettings editor opened for phase: {phase}")
        print(f"تم فتح محرر الإعدادات للمرحلة: {phase_info['name_ar']}")
    
    def _on_export_settings(self, event):
        """Export current settings to a separate override file (preserves main config comments)."""
        print("\n" + "="*60)
        print("Export Settings / تصدير الإعدادات")
        print("="*60)
        
        # Check for pending changes
        if not self.pending_changes:
            print("\nNo pending changes to export / لا توجد تغييرات معلقة للتصدير")
            print("Use viewer.update_parameter('path', value) to make changes first.")
            print("استخدم الأمر أعلاه لإجراء تغييرات أولاً.")
            return
        
        print(f"\nPending changes to export / التغييرات المعلقة للتصدير:")
        for path, value in self.pending_changes.items():
            print(f"  {path}: {value}")
        
        # Validate before export
        self._validate_config()
        if any(w.severity == 'error' for w in self.config_warnings):
            print("\n[!] Warning: Configuration has errors.")
            print("[!] تحذير: الإعدادات تحتوي على أخطاء.")
        
        # Save to separate override file (preserves main config comments)
        try:
            override_file = self._save_config_overrides(self.pending_changes)
            print(f"\nSettings exported successfully to / تم تصدير الإعدادات بنجاح إلى:")
            print(f"  {override_file}")
            print(f"\nNote: Main config file preserved / ملاحظة: الملف الرئيسي محفوظ:")
            print(f"  {self.config_file}")
            print("\nTo apply overrides manually, copy values from the override file.")
            print("لتطبيق الإعدادات يدوياً، انسخ القيم من ملف الإعدادات المخصصة.")
            
            # Clear pending changes
            self.pending_changes.clear()
            
            # Update warnings display
            self._update_warnings_display()
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            print(f"\n[X] Export failed / فشل التصدير: {e}")
    
    def _get_parameter_value(self, path: str) -> Any:
        """Get current value of a parameter from config."""
        keys = path.split('.')
        obj = self.config
        try:
            for key in keys:
                obj = obj[key]
            return obj
        except (KeyError, TypeError):
            return None
    
    def update_parameter(self, path: str, value: Any, skip_validation: bool = False) -> bool:
        """
        Update a parameter in the config (tracks as pending change until exported).
        
        Args:
            path: Dot-separated path to parameter (e.g., 'autopilot.gains.attitude.Kp_pitch')
            value: New value
            skip_validation: If True, skip validation and display updates for better
                           performance during continuous slider dragging.
                           تخطي التحقق وتحديث العرض لأداء أفضل أثناء سحب الشريط المستمر.
        
        Returns:
            True if update succeeded, False if path is invalid
        """
        # Navigate to the parameter
        keys = path.split('.')
        obj = self.config
        try:
            for key in keys[:-1]:
                if key not in obj:
                    obj[key] = {}
                elif not isinstance(obj[key], dict):
                    # Intermediate key exists but is not a dict - cannot navigate further
                    # المفتاح الوسيط موجود لكنه ليس قاموساً - لا يمكن المتابعة
                    print(f"[X] Invalid path: '{key}' in '{path}' is not a dict")
                    print(f"[X] مسار غير صالح: '{key}' في '{path}' ليس قاموساً")
                    return False
                obj = obj[key]
        except (KeyError, TypeError) as e:
            print(f"[X] Failed to navigate path '{path}': {e}")
            print(f"[X] فشل التنقل في المسار '{path}': {e}")
            return False
        
        # Get old value and set new value
        old_value = obj.get(keys[-1]) if isinstance(obj, dict) else None
        obj[keys[-1]] = value

        # If this path was previously marked as missing, clear that state
        if hasattr(self, 'missing_paths'):
            self.missing_paths.discard(path)
        
        # Track as pending change
        self.pending_changes[path] = value
        
        if not skip_validation:
            print(f"Parameter updated / تم تحديث المعامل:")
            print(f"  {path}: {old_value} -> {value}")
            print(f"  [*] Change pending - use Export to save / التغيير معلق - استخدم التصدير للحفظ")
            
            # Re-validate and update display (skip during continuous slider drag for performance)
            # إعادة التحقق وتحديث العرض (تخطي أثناء سحب الشريط المستمر للأداء)
            self._validate_config()
            self._update_warnings_display()
            self._update_settings_display()
        return True
    
    def undo_changes(self):
        """
        Undo all pending changes by reloading the original config.
        التراجع عن جميع التغييرات المعلقة بإعادة تحميل الإعدادات الأصلية.
        """
        if not self.pending_changes:
            print("No pending changes to undo / لا توجد تغييرات معلقة للتراجع عنها")
            return
        
        # Restore original config
        self.config = copy.deepcopy(self.original_config)
        
        # Clear pending changes
        num_changes = len(self.pending_changes)
        self.pending_changes.clear()
        
        print(f"Undone {num_changes} changes / تم التراجع عن {num_changes} تغييرات")
        print("Config restored to original state / تم استعادة الإعدادات الأصلية")
        
        # Re-validate and update display
        self._validate_config()
        self._update_warnings_display()
        self._update_settings_display()
    
    def copy_path(self, param_number: int):
        """
        Print the path for a parameter number in the current phase for easy copying.
        طباعة مسار المعامل حسب رقمه في المرحلة الحالية لسهولة النسخ.
        
        Args:
            param_number: 1-based parameter number from the settings display
        """
        phase = self.selected_settings_phase
        if phase not in PHASE_PARAMETERS:
            print(f"Invalid phase: {phase}")
            return
        
        params = PHASE_PARAMETERS[phase]['parameters']
        if param_number < 1 or param_number > len(params):
            print(f"Invalid parameter number. Valid range: 1-{len(params)}")
            print(f"رقم معامل غير صالح. النطاق الصالح: 1-{len(params)}")
            return
        
        param = params[param_number - 1]
        path = param['path']
        current_val = self._get_parameter_value(path)
        
        print(f"\nParameter {param_number}: {param['name']}")
        print(f"المعامل {param_number}: {param['name_ar']}")
        print(f"\nPath / المسار:")
        print(f"  {path}")
        print(f"\nCurrent value / القيمة الحالية: {current_val}")
        print(f"\nExample command / أمر مثال:")
        print(f"  viewer.update_parameter('{path}', {current_val})")
    
    def show_parameter_editor(self):
        """Show interactive parameter editor in console (CLI mode only).
        
        WARNING / تحذير:
            This function uses input() which blocks the matplotlib event loop.
            Only use this function BEFORE calling plt.show() (e.g., via --edit flag).
            During GUI operation, use the graphical settings editor instead.
            
            هذه الدالة تستخدم input() الذي يحجب حلقة أحداث matplotlib.
            استخدم هذه الدالة فقط قبل استدعاء plt.show() (مثلاً عبر وسيط --edit).
            أثناء تشغيل الواجهة الرسومية، استخدم محرر الإعدادات الرسومي بدلاً من ذلك.
        """
        # Check if GUI is active - warn user about blocking behavior
        # التحقق مما إذا كانت الواجهة الرسومية نشطة - تحذير المستخدم من السلوك الحاجب
        if self.fig is not None:
            print("\n" + "="*60)
            print("[!] WARNING: Console input blocks the GUI / تحذير: الإدخال يحجب الواجهة")
            print("    Use the graphical 'Edit' button instead for interactive editing.")
            print("    استخدم زر 'Edit' الرسومي بدلاً من ذلك للتعديل التفاعلي.")
            print("    Or use: viewer.update_parameter('path', value) in Python console.")
            print("    أو استخدم: viewer.update_parameter('path', value) في وحدة تحكم Python.")
            print("="*60)
            return
        
        print("\n" + "="*60)
        print("Parameter Editor / محرر المعاملات")
        print("="*60)
        print("\nCommon parameters / المعاملات الشائعة:")
        print("  1. simulation.duration - Simulation duration (s)")
        print("  2. guidance.pitch_program.pitch_target_deg - Target pitch angle")
        print("  3. autopilot.gains.attitude.Kp_pitch - Pitch Kp gain")
        print("  4. autopilot.gains.attitude.Ki_pitch - Pitch Ki gain")
        print("  5. autopilot.gains.attitude.Kd_pitch - Pitch Kd gain")
        print("  6. autopilot.gains.rate.Kp_pitch - Pitch rate Kp")
        print("  7. actuator.delta_max - Max fin deflection (deg)")
        print("\nEnter parameter path and value (e.g., 'autopilot.gains.attitude.Kp_pitch 5.0')")
        print("Or 'q' to quit editor / أو 'q' للخروج")
        
        while True:
            try:
                user_input = input("\n> ").strip()
                if user_input.lower() == 'q':
                    break
                
                parts = user_input.split()
                if len(parts) != 2:
                    print("Invalid input. Use: path value")
                    continue
                
                path = parts[0]
                try:
                    value = float(parts[1])
                except ValueError:
                    value = parts[1]
                
                self.update_parameter(path, value)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def run(self):
        """Run the interactive viewer."""
        print("\n" + "="*60)
        print("6-DOF Rocket Interactive Simulation Viewer")
        print("عارض المحاكاة التفاعلي للصاروخ 6-DOF")
        print("="*60)
        print(f"\nConfig file: {self.config_file}")
        print(f"ملف الإعدادات: {self.config_file}")
        
        # Run initial simulation
        if not self.run_simulation():
            print("\nFailed to run simulation. Please check configuration.")
            print("فشل تشغيل المحاكاة. يرجى التحقق من الإعدادات.")
            return
        
        # Create and show UI
        self.create_ui()
        
        # Jump to initial phase if specified via --phase CLI argument
        # الانتقال إلى المرحلة الأولية إذا تم تحديدها عبر وسيط --phase
        if self.initial_phase is not None:
            self.jump_to_phase(self.initial_phase)
        
        print("\n" + "-"*60)
        print("Controls / التحكم:")
        print("  - Play/Pause: Start/stop playback / بدء/إيقاف التشغيل")
        print("  - Reset: Return to start / العودة للبداية")
        print("  - Run Sim: Re-run simulation with current config / إعادة تشغيل المحاكاة")
        print("  - Edit: Open phase-specific parameter editor / فتح محرر الإعدادات الخاصة بالمرحلة")
        print("  - Export: Save overrides to separate YAML file / حفظ التغييرات في ملف إعدادات مخصص")
        print("  - Speed slider: Adjust playback speed (0.1x - 10x) / ضبط سرعة التشغيل")
        print("  - Time slider: Jump to specific time / الانتقال لوقت محدد")
        print("  - Phase buttons: Jump to flight phase / الانتقال لمرحلة الطيران")
        print("  - Settings Phase: Select parameter category / اختيار فئة المعاملات")
        print("-"*60)
        
        plt.show()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Interactive 6-DOF Rocket Simulation Viewer / عارض محاكاة الصاروخ التفاعلي'
    )
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='Path to configuration file')
    parser.add_argument('--phase', '-p', type=str, default=None,
                        help='Start playback at specific flight phase (e.g., APOGEE, TERMINAL)')
    parser.add_argument('--edit', '-e', action='store_true',
                        help='Open parameter editor before running')
    parser.add_argument('--apply-overrides', '-a', type=str, default=None,
                        help='Path to override YAML file to apply on startup')
    
    args = parser.parse_args()
    
    # Create viewer
    viewer = InteractiveViewer(config_file=args.config)
    
    # Apply overrides if specified
    if args.apply_overrides:
        try:
            with open(args.apply_overrides, 'r', encoding='utf-8') as f:
                overrides = yaml.safe_load(f)
            if overrides:
                # Flatten nested dict to get dot-separated paths
                # تحويل القاموس المتداخل إلى مسارات مفصولة بنقاط
                flat_overrides = _flatten_dict(overrides)
                for path, value in flat_overrides.items():
                    viewer.update_parameter(path, value)
                print(f"Applied {len(flat_overrides)} overrides from {args.apply_overrides}")
                print(f"تم تطبيق {len(flat_overrides)} إعدادات من {args.apply_overrides}")
        except Exception as e:
            print(f"Error loading overrides: {e}")
            print(f"خطأ في تحميل الإعدادات: {e}")
    
    # Set initial phase if specified (for both settings display and playback jump)
    # ضبط المرحلة الأولية إذا تم تحديدها (لعرض الإعدادات والانتقال في التشغيل)
    if args.phase:
        # Set settings phase if it's a valid settings category
        if args.phase in PHASE_PARAMETERS:
            viewer.selected_settings_phase = args.phase
            print(f"Starting with settings phase: {args.phase}")
            print(f"بدء مع فئة الإعدادات: {args.phase}")
        
        # Set initial playback phase (will jump after simulation runs)
        # This works for flight phases like POWERED_ASCENT, APOGEE, TERMINAL, etc.
        viewer.initial_phase = args.phase
        print(f"Will jump to flight phase: {args.phase}")
        print(f"سيتم الانتقال إلى مرحلة الطيران: {args.phase}")
    
    # Show parameter editor if requested
    if args.edit:
        viewer.show_parameter_editor()
    
    # Run viewer
    viewer.run()


if __name__ == '__main__':
    main()
