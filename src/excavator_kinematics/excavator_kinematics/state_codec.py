from typing import Tuple

from std_msgs.msg import Float32MultiArray


# Encoded state array layout:
# 0 re_deg, 1 rt_deg, 2 mass_kg,
# 3 l1_mm, 4 l2_mm, 5 l1_pct, 6 l2_pct,
# 7 re_dot, 8 rt_dot, 9 f_l1_n, 10 f_l2_n,
# 11 l1_warn, 12 l2_warn,
# 13-14 j1(x,y), 15-16 j2(x,y), 17-18 j3(x,y), 19-20 j4(x,y), 21-22 tip(x,y)
STATE_LEN = 23


class DecodedState:
    def __init__(self, data):
        self.re_deg = data[0]
        self.rt_deg = data[1]
        self.mass_kg = data[2]
        self.l1_mm = data[3]
        self.l2_mm = data[4]
        self.l1_pct = data[5]
        self.l2_pct = data[6]
        self.re_dot = data[7]
        self.rt_dot = data[8]
        self.f_l1_n = data[9]
        self.f_l2_n = data[10]
        self.l1_warn = data[11] >= 0.5
        self.l2_warn = data[12] >= 0.5
        self.j1_mm = (data[13], data[14])
        self.j2_mm = (data[15], data[16])
        self.j3_mm = (data[17], data[18])
        self.j4_mm = (data[19], data[20])
        self.tip_mm = (data[21], data[22])


def encode_state(state) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [
        float(state.re_deg),
        float(state.rt_deg),
        float(state.mass_kg),
        float(state.l1_mm),
        float(state.l2_mm),
        float(state.l1_pct),
        float(state.l2_pct),
        float(state.re_dot),
        float(state.rt_dot),
        float(state.f_l1_n),
        float(state.f_l2_n),
        1.0 if state.l1_warn else 0.0,
        1.0 if state.l2_warn else 0.0,
        float(state.j1_mm[0]),
        float(state.j1_mm[1]),
        float(state.j2_mm[0]),
        float(state.j2_mm[1]),
        float(state.j3_mm[0]),
        float(state.j3_mm[1]),
        float(state.j4_mm[0]),
        float(state.j4_mm[1]),
        float(state.tip_mm[0]),
        float(state.tip_mm[1]),
    ]
    return msg


def decode_state(msg: Float32MultiArray) -> DecodedState:
    if len(msg.data) < STATE_LEN:
        raise ValueError(f"state message length {len(msg.data)} < expected {STATE_LEN}")
    return DecodedState(msg.data)


def mm_xy_to_point_xyz(xy_mm: Tuple[float, float]) -> Tuple[float, float, float]:
    # MATLAB model is planar in (x, y); map y to ROS z for world vertical axis.
    return (xy_mm[0] * 0.001, 0.0, xy_mm[1] * 0.001)
