from excavator_kinematics.kinematics_model import ExcavatorConfig, ExcavatorMechanism


def test_nominal_state_has_valid_outputs() -> None:
    model = ExcavatorMechanism(ExcavatorConfig())
    state = model.compute(re_deg=30.0, rt_deg=-45.0, mass_kg=10.0, c_mm=125.0, b_mm=125.0)

    assert state.l1_mm > 0.0
    assert state.l2_mm > 0.0
    assert state.f_l1_n >= 0.0
    assert state.f_l2_n >= 0.0


def test_clamping_keeps_stroke_within_limits() -> None:
    cfg = ExcavatorConfig(actuator_min_mm=380.0, actuator_max_mm=630.0)
    model = ExcavatorMechanism(cfg)
    state = model.compute(re_deg=85.0, rt_deg=60.0, mass_kg=15.0, c_mm=125.0, b_mm=125.0)

    assert cfg.actuator_min_mm <= state.l1_mm <= cfg.actuator_max_mm
    assert cfg.actuator_min_mm <= state.l2_mm <= cfg.actuator_max_mm
