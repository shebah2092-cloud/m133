"""
failure_injector.py
===================
يحقن أعطالاً في محاكاة Python أثناء التشغيل لاختبار متانة النظام.

الاستخدام:
    injector = FailureInjector(scenario.failure_type, scenario.failure_params)
    sim.register_step_hook(injector.on_step)

الأعطال المدعومة:
    - nan_accel       : يحقن NaN في قراءة accelerometer
    - gps_dropout     : يوقف تحديث GPS لفترة محدّدة
    - attitude_freeze : يجمّد قراءة attitude (يقلّدها = stale data)
    - ekf2_stale     : مماثل لكن لـ EKF2 بالكامل (lpos/att)
    - mhe_force_fail  : يجعل MHE يُفشل N مرات متتالية
    - multi_arm       : دورات arm→fly→disarm متتالية (ليس hook عادي، يُعالَج في scenario_runner)

تنفيذ nan_accel داخلياً:
    يُعدَّل snapshot['forces'] قبل إرسالها إلى bridge أو MHE.

تنفيذ gps_dropout:
    يُعدَّل flag مُشترَك يقرأه SITL bridge لإيقاف HIL_GPS messages.

تنفيذ attitude_freeze:
    يُخزَّن آخر quat ويُعاد إرساله بدل الحقيقي.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple
import math
import numpy as np


@dataclass
class InjectorState:
    active: bool = False
    started_at: Optional[float] = None
    frozen_quat: Optional[Tuple[float, float, float, float]] = None
    last_lpos: Optional[Tuple[float, float, float]] = None
    mhe_fail_count: int = 0
    events: list = field(default_factory=list)


class FailureInjector:
    """
    Hook يُشغَّل كل خطوة محاكاة. يعدّل بيانات snapshot/sensors حسب failure_type.
    """

    def __init__(self, failure_type: Optional[str], params: Dict[str, Any]):
        self.type = failure_type
        self.p = params or {}
        self.state = InjectorState()
        self.t_start = float(self.p.get('t_start', 0.0))
        self.duration = float(self.p.get('duration', 1.0))
        self.n_fails_max = int(self.p.get('n_fails', 10))

    def is_window(self, t: float) -> bool:
        return self.t_start <= t <= (self.t_start + self.duration)

    # ─── hooks قابلة للاستدعاء من bridge / sim ──────────────────────────

    def mutate_sensors(self, t: float, sensors: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """يعدّل dict من أسماء إلى arrays. يُستدعى قبل إرسال HIL_SENSOR."""
        if self.type == 'nan_accel' and self.is_window(t):
            if 'accel' in sensors:
                s = sensors['accel'].copy()
                s[:] = np.nan
                sensors['accel'] = s
                self._log(t, 'nan_accel_applied', {})
        return sensors

    def should_send_gps(self, t: float) -> bool:
        if self.type == 'gps_dropout' and self.is_window(t):
            self._log_once(t, 'gps_dropout_window')
            return False
        return True

    def mutate_attitude(self, t: float, quat: np.ndarray) -> np.ndarray:
        if self.type == 'attitude_freeze' and self.is_window(t):
            if self.state.frozen_quat is None:
                self.state.frozen_quat = tuple(float(x) for x in quat)
                self._log(t, 'attitude_freeze_start', {'q': self.state.frozen_quat})
            return np.asarray(self.state.frozen_quat)
        else:
            if self.state.frozen_quat is not None and t > (self.t_start + self.duration):
                self._log(t, 'attitude_freeze_end', {})
                self.state.frozen_quat = None
        return quat

    def should_publish_ekf2(self, t: float) -> bool:
        """استخدم هذا كـ gate للـ PX4 lpos/att topics عند ekf2_stale."""
        if self.type == 'ekf2_stale' and self.is_window(t):
            return False
        return True

    def mhe_should_fail(self, t: float) -> bool:
        if self.type != 'mhe_force_fail':
            return False
        if not self.is_window(t):
            return False
        if self.state.mhe_fail_count >= self.n_fails_max:
            return False
        self.state.mhe_fail_count += 1
        self._log(t, 'mhe_force_fail', {'count': self.state.mhe_fail_count})
        return True

    # ─── logging داخلي ─────────────────────────────────────────────────

    def _log(self, t: float, event: str, extra: Dict[str, Any]) -> None:
        self.state.events.append({'t': float(t), 'event': event, **extra})

    def _log_once(self, t: float, event: str) -> None:
        if not any(e['event'] == event for e in self.state.events):
            self._log(t, event, {})

    def report(self) -> Dict[str, Any]:
        return {
            'failure_type': self.type,
            'params': self.p,
            'n_events': len(self.state.events),
            'mhe_fail_count': self.state.mhe_fail_count,
            'frozen_quat_applied': self.state.frozen_quat is not None,
            'events': self.state.events[:20],
        }
