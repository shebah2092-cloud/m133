"""
config_matrix.py
================
يولّد مصفوفة السيناريوهات التي ستُختَبر.
كل سيناريو = dict من overrides للـ YAML الأساسي.

الأبعاد المُغطّاة لكشف الـ 13 bug:
  - bearing: 0/45/90/135/180/225/270/315      → BUG-A (frame mixing)
  - pitch:   15/30/60/88                       → BUG-X2 (launch threshold)
  - range:   500/1000/3000/8000                → generalization
  - mass:    -10% / 0 / +10%                   → robustness
  - attitude_mode: normal / near-vertical      → edge cases
  - scenarios خاصّة: multi-arm, chi-wrap, NaN, GPS-loss, EKF-freeze
"""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional
import itertools


@dataclass
class Scenario:
    name: str
    overrides: Dict[str, Any] = field(default_factory=dict)
    failure_type: Optional[str] = None
    failure_params: Dict[str, Any] = field(default_factory=dict)
    duration_override: Optional[float] = None
    tags: List[str] = field(default_factory=list)
    expected_detects: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _nested_set(d: Dict[str, Any], path: str, value: Any) -> None:
    keys = path.split('.')
    cur = d
    for k in keys[:-1]:
        cur = cur.setdefault(k, {})
    cur[keys[-1]] = value


def apply_overrides(base_cfg: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    import copy
    cfg = copy.deepcopy(base_cfg)
    for path, val in overrides.items():
        _nested_set(cfg, path, val)
    return cfg


# ─── المصفوفات الفرعية ──────────────────────────────────────────────────────

def bearing_sweep(pitches=(15, 30), distances=(3000,), bearings=(0, 45, 90, 135, 180, 225, 270, 315)) -> List[Scenario]:
    out: List[Scenario] = []
    for b, p, r in itertools.product(bearings, pitches, distances):
        out.append(Scenario(
            name=f"sweep_bearing{b:03d}_pitch{p:02d}_range{r}",
            overrides={
                'target.bearing_deg': float(b),
                'target.range_m': float(r),
                'initial_conditions.attitude_degrees': [0.0, float(p), 0.0],
            },
            tags=['bearing_sweep'],
            expected_detects=['BUG-A'] if b != 0 else [],
        ))
    return out


def near_vertical_launch() -> List[Scenario]:
    """pitch >= 88° يُخفق شرط |a_z|>g في C++ (BUG-X2)."""
    out: List[Scenario] = []
    for p in (80.0, 85.0, 88.0, 89.5):
        out.append(Scenario(
            name=f"nearvert_pitch{int(p*10):03d}",
            overrides={
                'initial_conditions.attitude_degrees': [0.0, float(p), 0.0],
                'target.range_m': 200.0,
            },
            tags=['near_vertical'],
            expected_detects=['BUG-X2'] if p >= 88 else [],
        ))
    return out


def mass_robustness(base_mass_key='rocket_data.mass_override_kg',
                    variations=(-0.10, -0.05, 0.0, 0.05, 0.10)) -> List[Scenario]:
    out: List[Scenario] = []
    for v in variations:
        pct = int(round(v * 100))
        out.append(Scenario(
            name=f"mass_err_{pct:+03d}pct",
            overrides={
                'error_injection.sig_mass_fraction': float(v),
            },
            tags=['mass_robustness'],
        ))
    return out


def failure_injection() -> List[Scenario]:
    """حقن أعطال يجب أن تُكشَف."""
    return [
        Scenario(
            name="inject_nan_accel_at_2s5",
            failure_type='nan_accel',
            failure_params={'t_start': 2.5, 'duration': 0.5},
            tags=['failure'],
            expected_detects=['BUG-3'],
        ),
        Scenario(
            name="inject_gps_loss_mid_flight",
            failure_type='gps_dropout',
            failure_params={'t_start': 3.0, 'duration': 2.0},
            tags=['failure'],
            expected_detects=['BUG-F'],
        ),
        Scenario(
            name="inject_attitude_freeze",
            failure_type='attitude_freeze',
            failure_params={'t_start': 4.0, 'duration': 1.0},
            tags=['failure'],
            expected_detects=['BUG-F'],
        ),
        Scenario(
            name="inject_ekf2_stale",
            failure_type='ekf2_stale',
            failure_params={'t_start': 5.0, 'duration': 1.5},
            tags=['failure'],
            expected_detects=['BUG-F'],
        ),
        Scenario(
            name="inject_mhe_consecutive_fails",
            failure_type='mhe_force_fail',
            failure_params={'t_start': 2.0, 'n_fails': 12},
            tags=['failure'],
            expected_detects=['BUG-3', 'BUG-D'],
        ),
    ]


def multi_arm_cycles() -> List[Scenario]:
    return [
        Scenario(
            name="multi_arm_3cycles",
            failure_type='multi_arm',
            failure_params={'n_cycles': 3, 'disarm_alt_m': 5.0},
            tags=['multi_arm'],
            expected_detects=['BUG-C'],
        ),
    ]


def chi_wrap() -> List[Scenario]:
    """هدف خلف الصاروخ → chi tracking cross ±180° → يكشف BUG-H."""
    out: List[Scenario] = []
    for b in (175, 180, 185):
        out.append(Scenario(
            name=f"chi_wrap_bearing{b}",
            overrides={
                'target.bearing_deg': float(b),
                'target.range_m': 500.0,
                'initial_conditions.attitude_degrees': [0.0, 20.0, 0.0],
            },
            tags=['chi_wrap'],
            expected_detects=['BUG-H'],
        ))
    return out


def control_authority() -> List[Scenario]:
    """هدف بعيد + pitch خاطئ → يدفع solver لـ max deflection (BUG-E)."""
    return [
        Scenario(
            name="authority_saturation",
            overrides={
                'target.range_m': 10000.0,
                'target.bearing_deg': 45.0,
                'initial_conditions.attitude_degrees': [0.0, 5.0, 0.0],
            },
            tags=['authority'],
            expected_detects=['BUG-E'],
        ),
    ]


def parity_subset() -> List[Scenario]:
    """
    سيناريوهات مختارة ستُنفَّذ مع SITL (PX4) لإنتاج parity check.
    هذه مكلفة لذا نُبقيها قليلة لكن دالّة.
    """
    out: List[Scenario] = []
    for b, p in [(0, 15), (90, 15), (180, 30), (270, 45)]:
        out.append(Scenario(
            name=f"parity_b{b:03d}_p{p:02d}",
            overrides={
                'target.bearing_deg': float(b),
                'target.range_m': 3000.0,
                'initial_conditions.attitude_degrees': [0.0, float(p), 0.0],
            },
            tags=['parity', 'require_sitl'],
            expected_detects=['BUG-1', 'BUG-2', 'BUG-A', 'BUG-X1'],
        ))
    return out


# ─── المجموعة الكاملة ───────────────────────────────────────────────────────

def build_full_matrix(include_sitl_parity: bool = False,
                      quick: bool = False) -> List[Scenario]:
    """
    quick=True → مصفوفة مُصغَّرة (~10 سيناريوهات) للتطوير
    quick=False → ~50+ سيناريو (التغطية الكاملة)
    """
    scenarios: List[Scenario] = []
    if quick:
        scenarios += bearing_sweep(pitches=(15,), bearings=(0, 90, 180, 270))
        scenarios += near_vertical_launch()[:2]
        scenarios += failure_injection()[:2]
        scenarios += chi_wrap()[:1]
    else:
        scenarios += bearing_sweep()
        scenarios += near_vertical_launch()
        scenarios += mass_robustness()
        scenarios += failure_injection()
        scenarios += multi_arm_cycles()
        scenarios += chi_wrap()
        scenarios += control_authority()

    if include_sitl_parity:
        scenarios += parity_subset()

    return scenarios


def summarise_matrix(scenarios: List[Scenario]) -> Dict[str, Any]:
    tags: Dict[str, int] = {}
    bugs: Dict[str, int] = {}
    for s in scenarios:
        for t in s.tags:
            tags[t] = tags.get(t, 0) + 1
        for b in s.expected_detects:
            bugs[b] = bugs.get(b, 0) + 1
    return {
        'total_scenarios': len(scenarios),
        'by_tag': tags,
        'by_expected_detection': bugs,
    }


if __name__ == '__main__':
    import json
    mx = build_full_matrix(include_sitl_parity=True)
    print(json.dumps(summarise_matrix(mx), indent=2))
    for s in mx:
        print(f"  - {s.name:45s} tags={s.tags} detects={s.expected_detects}")
