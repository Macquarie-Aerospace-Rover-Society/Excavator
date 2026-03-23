from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class ExcavatorConfig:
    s1_mm: float = 505.0
    c_mm: float = 125.0
    b_mm: float = 125.0
    actuator_min_mm: float = 380.0
    actuator_max_mm: float = 630.0
    warn_thresh_mm: float = 5.0
    v_stroke_mmps: float = 5.0
    gravity_mps2: float = 9.81


@dataclass
class ExcavatorState:
    re_deg: float
    rt_deg: float
    mass_kg: float
    l1_mm: float
    l2_mm: float
    l1_pct: float
    l2_pct: float
    re_dot: float
    rt_dot: float
    f_l1_n: float
    f_l2_n: float
    l1_warn: bool
    l2_warn: bool
    j1_mm: Tuple[float, float]
    j2_mm: Tuple[float, float]
    j3_mm: Tuple[float, float]
    j4_mm: Tuple[float, float]
    tip_mm: Tuple[float, float]


class ExcavatorMechanism:
    def __init__(self, config: ExcavatorConfig) -> None:
        self.cfg = config
        self._eps = 1.0e-9

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _deg_sin(deg: float) -> float:
        return math.sin(math.radians(deg))

    @staticmethod
    def _deg_cos(deg: float) -> float:
        return math.cos(math.radians(deg))

    def _l1_from_angles(self, re_deg: float, c_mm: float) -> float:
        return math.sqrt(
            self.cfg.s1_mm ** 2 + c_mm ** 2 - 2.0 * c_mm * self.cfg.s1_mm * self._deg_sin(re_deg)
        )

    def _l2_from_angles(self, re_deg: float, rt_deg: float, b_mm: float) -> float:
        return math.sqrt(
            self.cfg.s1_mm ** 2
            + b_mm ** 2
            + 2.0 * self.cfg.s1_mm * b_mm * self._deg_cos(re_deg - rt_deg)
        )

    def compute(self, re_deg: float, rt_deg: float, mass_kg: float, c_mm: float, b_mm: float) -> ExcavatorState:
        l1_mm = self._l1_from_angles(re_deg, c_mm)
        if l1_mm > self.cfg.actuator_max_mm or l1_mm < self.cfg.actuator_min_mm:
            l1_mm = self._clamp(l1_mm, self.cfg.actuator_min_mm, self.cfg.actuator_max_mm)
            ratio = (self.cfg.s1_mm ** 2 + c_mm ** 2 - l1_mm ** 2) / (2.0 * c_mm * self.cfg.s1_mm + self._eps)
            ratio = self._clamp(ratio, -1.0, 1.0)
            re_deg = math.degrees(math.asin(ratio))

        l2_mm = self._l2_from_angles(re_deg, rt_deg, b_mm)
        if l2_mm > self.cfg.actuator_max_mm or l2_mm < self.cfg.actuator_min_mm:
            l2_mm = self._clamp(l2_mm, self.cfg.actuator_min_mm, self.cfg.actuator_max_mm)
            ratio = (l2_mm ** 2 - self.cfg.s1_mm ** 2 - b_mm ** 2) / (2.0 * self.cfg.s1_mm * b_mm + self._eps)
            ratio = self._clamp(ratio, -1.0, 1.0)
            rt_deg = re_deg - math.degrees(math.acos(ratio))

        l1_pct = ((l1_mm - self.cfg.actuator_min_mm) / (self.cfg.actuator_max_mm - self.cfg.actuator_min_mm)) * 100.0
        l2_pct = ((l2_mm - self.cfg.actuator_min_mm) / (self.cfg.actuator_max_mm - self.cfg.actuator_min_mm)) * 100.0

        j3 = (0.0, 0.0)
        j1 = (0.0, c_mm)
        j2 = (self.cfg.s1_mm * self._deg_cos(re_deg), self.cfg.s1_mm * self._deg_sin(re_deg))
        j4 = (j2[0] + b_mm * self._deg_cos(rt_deg), j2[1] + b_mm * self._deg_sin(rt_deg))

        vec_j2_j4 = (j4[0] - j2[0], j4[1] - j2[1])
        len_j2_j4 = math.sqrt(vec_j2_j4[0] ** 2 + vec_j2_j4[1] ** 2) + self._eps
        dir_b = (vec_j2_j4[0] / len_j2_j4, vec_j2_j4[1] / len_j2_j4)
        perp_b = (-dir_b[1], dir_b[0])
        tip = (
            j2[0] + 1.2 * vec_j2_j4[0] + (b_mm * 1.2) * perp_b[0],
            j2[1] + 1.2 * vec_j2_j4[1] + (b_mm * 1.2) * perp_b[1],
        )

        re_dot = abs(
            (l1_mm * self.cfg.v_stroke_mmps)
            / (c_mm * self.cfg.s1_mm * self._deg_cos(re_deg) + self._eps)
        )
        rt_dot = re_dot + (
            (l2_mm * self.cfg.v_stroke_mmps)
            / (self.cfg.s1_mm * b_mm * self._deg_sin(re_deg - rt_deg) + self._eps)
        )

        f_gravity = (0.0, -mass_kg * self.cfg.gravity_mps2)
        len_j3_j4 = math.sqrt(j4[0] ** 2 + j4[1] ** 2) + self._eps
        u_l2 = (j4[0] / len_j3_j4, j4[1] / len_j3_j4)

        m_grav_j2 = (j4[0] - j2[0]) * f_gravity[1] - (j4[1] - j2[1]) * f_gravity[0]
        lever_l2 = (j4[0] - j2[0]) * u_l2[1] - (j4[1] - j2[1]) * u_l2[0]
        f_l2_mag = -m_grav_j2 / (lever_l2 + self._eps)

        len_j1_j2 = math.sqrt((j2[0] - j1[0]) ** 2 + (j2[1] - j1[1]) ** 2) + self._eps
        u_l1 = ((j2[0] - j1[0]) / len_j1_j2, (j2[1] - j1[1]) / len_j1_j2)

        m_grav_j3 = j4[0] * f_gravity[1] - j4[1] * f_gravity[0]
        lever_l1 = j2[0] * u_l1[1] - j2[1] * u_l1[0]
        f_l1_mag = -m_grav_j3 / (lever_l1 + self._eps)

        l1_warn = (l1_mm < self.cfg.actuator_min_mm + self.cfg.warn_thresh_mm) or (
            l1_mm > self.cfg.actuator_max_mm - self.cfg.warn_thresh_mm
        )
        l2_warn = (l2_mm < self.cfg.actuator_min_mm + self.cfg.warn_thresh_mm) or (
            l2_mm > self.cfg.actuator_max_mm - self.cfg.warn_thresh_mm
        )

        return ExcavatorState(
            re_deg=re_deg,
            rt_deg=rt_deg,
            mass_kg=mass_kg,
            l1_mm=l1_mm,
            l2_mm=l2_mm,
            l1_pct=l1_pct,
            l2_pct=l2_pct,
            re_dot=re_dot,
            rt_dot=rt_dot,
            f_l1_n=abs(f_l1_mag),
            f_l2_n=abs(f_l2_mag),
            l1_warn=l1_warn,
            l2_warn=l2_warn,
            j1_mm=j1,
            j2_mm=j2,
            j3_mm=j3,
            j4_mm=j4,
            tip_mm=tip,
        )
