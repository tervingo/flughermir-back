"""
Minimal flight physics for MVP: 4 forces (lift, weight, thrust, drag) + Euler integration.
SI units. Body frame: X forward, Y right, Z down (NED).
"""

from dataclasses import dataclass
from math import sin, cos, sqrt


@dataclass
class AircraftState:
    x: float   # position E (m)
    y: float   # position N (m)
    z: float   # position, NED z (m) — altitude = -z
    u: float   # velocity body-x (m/s)
    v: float
    w: float
    phi: float   # roll (rad)
    theta: float # pitch (rad)
    psi: float   # heading (rad)
    p: float
    q: float
    r: float
    throttle: float
    elevator: float
    aileron: float
    rudder: float


MASS = 1000.0
WING_AREA = 16.2
CL_ALPHA = 5.0
CD0 = 0.03
K = 0.04
THRUST_MAX = 3500.0
G = 9.81


def update_physics(state: AircraftState, dt: float) -> AircraftState:
    v_mag = sqrt(state.u * state.u + state.v * state.v + state.w * state.w)
    if v_mag < 0.1:
        v_mag = 0.1
    alpha = state.theta - (state.w / v_mag if v_mag > 0.5 else 0)
    q_bar = 0.5 * 1.225 * v_mag * v_mag
    cl = CL_ALPHA * alpha
    cd = CD0 + K * cl * cl
    lift = q_bar * WING_AREA * cl
    drag = q_bar * WING_AREA * cd
    thrust = THRUST_MAX * state.throttle
    weight = MASS * G

    # Body frame: X fwd, Z down. Lift opposes weight (along -Z), drag along -velocity (approx -X)
    fx = thrust - drag * (state.u / v_mag)
    fz = -lift * (state.u / v_mag) + weight * cos(state.theta)  # Z down: -lift component + weight
    ax = fx / MASS
    az = fz / MASS
    ay = 0.0

    # Body angular rates from controls (simplified)
    p_new = state.p + (state.aileron * 0.6 - state.p * 0.4) * dt
    q_new = state.q + (state.elevator * 0.6 - state.q * 0.4) * dt
    r_new = state.r + (state.rudder * 0.4 - state.r * 0.4) * dt
    phi_new = state.phi + p_new * dt
    theta_new = state.theta + q_new * dt
    psi_new = state.psi + r_new * dt

    # Body velocity integration
    u_new = state.u + ax * dt
    v_new = state.v + ay * dt
    w_new = state.w + az * dt

    # Body to NED (world) rotation
    cphi, sphi = cos(state.phi), sin(state.phi)
    cth, sth = cos(state.theta), sin(state.theta)
    cpsi, spsi = cos(state.psi), sin(state.psi)
    u_w = state.u * (cth * cpsi) + state.v * (sphi * sth * cpsi - cphi * spsi) + state.w * (cphi * sth * cpsi + sphi * spsi)
    v_w = state.u * (cth * spsi) + state.v * (sphi * sth * spsi + cphi * cpsi) + state.w * (cphi * sth * spsi - sphi * cpsi)
    w_w = state.u * (-sth) + state.v * (sphi * cth) + state.w * (cphi * cth)

    x_new = state.x + u_w * dt
    y_new = state.y + v_w * dt
    z_new = state.z + w_w * dt

    return AircraftState(
        x=x_new, y=y_new, z=z_new,
        u=u_new, v=v_new, w=w_new,
        phi=phi_new, theta=theta_new, psi=psi_new,
        p=p_new, q=q_new, r=r_new,
        throttle=state.throttle, elevator=state.elevator,
        aileron=state.aileron, rudder=state.rudder,
    )


def state_to_telemetry(state: AircraftState) -> dict:
    v_mag = sqrt(state.u**2 + state.v**2 + state.w**2)
    return {
        "x": state.x,
        "y": state.y,
        "altitude": -state.z,
        "phi_deg": round(state.phi * 180 / 3.14159265, 4),
        "theta_deg": round(state.theta * 180 / 3.14159265, 4),
        "psi_deg": round(state.psi * 180 / 3.14159265, 4),
        "airspeed": round(v_mag, 2),
        "throttle": state.throttle,
    }
