"""
Minimal flight physics for MVP: 4 forces (lift, weight, thrust, drag) + Euler integration.
SI units. Body frame: X forward, Y right, Z down (NED).
State is clamped to sane limits so the sim never overflows or diverges.
"""

from dataclasses import dataclass
from math import sin, cos, sqrt, isfinite, pi


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


MASS = 1000.0       # kg (light aircraft)
G = 9.81            # m/s^2
WEIGHT = MASS * G   # N — downward force; lift must balance this in level flight
WING_AREA = 16.2
CL0 = 1.1          # lift at zero alpha: ~weight at 30 m/s
CL_ALPHA = 4.0     # per radian
CD0 = 0.035         # parasitic drag
K = 0.03            # induced drag factor
THRUST_MAX = 2800.0 # N — max thrust; 50% throttle ≈ 1400 N should balance ~45-50 m/s (162-180 km/h)
ROLLING_MU = 0.02   # rolling friction coefficient on tarmac
V_LOF      = 28.0   # lift-off speed (m/s) ≈ 100 km/h — aircraft cannot leave ground below this

# Clamps to prevent divergence and overflow
MAX_SPEED = 400.0          # m/s (hard ceiling for numerics)
MAX_AIRSPEED_MS = 83.33    # 300 km/h — light aircraft never exceeds this
MAX_ANGLE = pi
MAX_ANGULAR_RATE = 4.0     # rad/s
POS_LIMIT = 1e6            # m


def update_physics(state: AircraftState, dt: float) -> AircraftState:
    v_mag = sqrt(state.u * state.u + state.v * state.v + state.w * state.w)
    if v_mag < 0.1:
        v_mag = 0.1
    alpha = state.theta - (state.w / v_mag if v_mag > 0.5 else 0)
    q_bar = 0.5 * 1.225 * v_mag * v_mag
    cl = CL0 + CL_ALPHA * alpha
    cd = CD0 + K * cl * cl
    lift = q_bar * WING_AREA * cl
    drag = q_bar * WING_AREA * cd
    thrust = THRUST_MAX * state.throttle

    # Body frame: X fwd, Z down. Drag opposes velocity vector (mostly along X when level)
    # Drag force magnitude = drag, direction = opposite to velocity in body frame
    # For small angles, drag_x ≈ -drag * (u/v_mag), drag_z ≈ -drag * (w/v_mag)
    drag_x = drag * (state.u / v_mag) if v_mag > 0.1 else 0.0
    drag_z = drag * (state.w / v_mag) if v_mag > 0.1 else 0.0
    fx = thrust - drag_x
    # Lift acts perpendicular to velocity (approx along -Z when level), weight always down
    lift_z = lift * (state.u / v_mag) if v_mag > 0.1 else 0.0  # lift component along body Z (up = -Z)
    fz = -lift_z - drag_z + WEIGHT * cos(state.theta)  # Z down: -lift - drag_z + weight

    # Ground roll friction: opposes forward motion while wheels are on the ground.
    # Normal force decreases as lift builds, so friction fades naturally near take-off.
    on_ground = (state.z >= 0)
    if on_ground and v_mag > 0.1:
        normal = max(0.0, WEIGHT * cos(state.theta) - lift)
        fx -= ROLLING_MU * normal * (state.u / v_mag)

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

    # Body velocity integration (still in current/old body frame)
    u_int = state.u + ax * dt
    v_int = state.v + ay * dt
    w_int = state.w + az * dt

    # Current (old) body to NED: world velocity after integration step
    cphi, sphi = cos(state.phi), sin(state.phi)
    cth, sth = cos(state.theta), sin(state.theta)
    cpsi, spsi = cos(state.psi), sin(state.psi)
    u_w = u_int * (cth * cpsi) + v_int * (sphi * sth * cpsi - cphi * spsi) + w_int * (cphi * sth * cpsi + sphi * spsi)
    v_w = u_int * (cth * spsi) + v_int * (sphi * sth * spsi + cphi * cpsi) + w_int * (cphi * sth * spsi - sphi * cpsi)
    w_w = u_int * (-sth) + v_int * (sphi * cth) + w_int * (cphi * cth)

    # Ground: do not go below z=0. Zero vertical velocity when on ground.
    if state.z >= 0 and w_w > 0:
        w_w = 0.0
    # Minimum lift-off speed: wheels stay on the ground below V_LOF regardless of
    # pitch/AoA so the aircraft must accelerate down the runway first.
    if state.z >= 0 and v_mag < V_LOF and w_w < 0:
        w_w = 0.0
    z_new = state.z + w_w * dt
    if z_new > 0:
        z_new = 0.0
        w_w = 0.0
    x_new = state.x + u_w * dt
    y_new = state.y + v_w * dt

    # Express world velocity (u_w, v_w, w_w) in NEW body frame so (u,v,w) stays consistent with orientation
    cphi_n, sphi_n = cos(phi_new), sin(phi_new)
    cth_n, sth_n = cos(theta_new), sin(theta_new)
    cpsi_n, spsi_n = cos(psi_new), sin(psi_new)
    u_new = u_w * (cth_n * cpsi_n) + v_w * (cth_n * spsi_n) + w_w * (-sth_n)
    v_new = u_w * (sphi_n * sth_n * cpsi_n - cphi_n * spsi_n) + v_w * (sphi_n * sth_n * spsi_n + cphi_n * cpsi_n) + w_w * (sphi_n * cth_n)
    w_new = u_w * (cphi_n * sth_n * cpsi_n + sphi_n * spsi_n) + v_w * (cphi_n * sth_n * spsi_n - sphi_n * cpsi_n) + w_w * (cphi_n * cth_n)

    # Cap airspeed to light-aircraft max (300 km/h): scale velocity vector, preserve direction
    v_mag_new = sqrt(u_new * u_new + v_new * v_new + w_new * w_new)
    if v_mag_new > MAX_AIRSPEED_MS and v_mag_new > 0.01:
        scale = MAX_AIRSPEED_MS / v_mag_new
        u_new *= scale
        v_new *= scale
        w_new *= scale
    # Per-component clamp for numerics
    u_new = max(-MAX_SPEED, min(MAX_SPEED, u_new))
    v_new = max(-MAX_SPEED, min(MAX_SPEED, v_new))
    w_new = max(-MAX_SPEED, min(MAX_SPEED, w_new))
    phi_new = max(-MAX_ANGLE, min(MAX_ANGLE, phi_new))
    theta_new = max(-MAX_ANGLE, min(MAX_ANGLE, theta_new))
    psi_new = psi_new % (2 * pi) if isfinite(psi_new) else 0.0
    p_new = max(-MAX_ANGULAR_RATE, min(MAX_ANGULAR_RATE, p_new))
    q_new = max(-MAX_ANGULAR_RATE, min(MAX_ANGULAR_RATE, q_new))
    r_new = max(-MAX_ANGULAR_RATE, min(MAX_ANGULAR_RATE, r_new))
    x_new = max(-POS_LIMIT, min(POS_LIMIT, x_new))
    y_new = max(-POS_LIMIT, min(POS_LIMIT, y_new))
    z_new = max(-POS_LIMIT, min(POS_LIMIT, z_new))

    return AircraftState(
        x=x_new, y=y_new, z=z_new,
        u=u_new, v=v_new, w=w_new,
        phi=phi_new, theta=theta_new, psi=psi_new,
        p=p_new, q=q_new, r=r_new,
        throttle=state.throttle, elevator=state.elevator,
        aileron=state.aileron, rudder=state.rudder,
    )


def state_to_telemetry(state: AircraftState) -> dict:
    # Safe airspeed: avoid OverflowError when state has blown up (no squaring if too large)
    u, v, w = state.u, state.v, state.w
    if not all(isfinite(x) for x in (u, v, w)):
        v_mag = 0.0
    elif max(abs(u), abs(v), abs(w)) > 1e100:
        v_mag = MAX_AIRSPEED_MS
    else:
        v_mag = sqrt(u * u + v * v + w * w)
        if not isfinite(v_mag):
            v_mag = MAX_AIRSPEED_MS
        v_mag = min(v_mag, MAX_AIRSPEED_MS)
    
    # World vertical velocity (NED: z down, so vertical speed up = -w_world)
    cphi, sphi = cos(state.phi), sin(state.phi)
    cth, sth = cos(state.theta), sin(state.theta)
    cpsi, spsi = cos(state.psi), sin(state.psi)
    w_world = u * (-sth) + v * (sphi * cth) + w * (cphi * cth)
    vertical_speed = -w_world  # m/s, positive = climbing
    
    return {
        "x": state.x,
        "y": state.y,
        "altitude": -state.z,
        "phi_deg": round(state.phi * 180 / 3.14159265, 4),
        "theta_deg": round(state.theta * 180 / 3.14159265, 4),
        "psi_deg": round(state.psi * 180 / 3.14159265, 4),
        "airspeed": round(v_mag, 2),
        "vertical_speed": round(vertical_speed, 2),  # m/s, positive = up
        "p_deg_s": round(state.p * 180 / 3.14159265, 2),  # roll rate deg/s
        "q_deg_s": round(state.q * 180 / 3.14159265, 2),  # pitch rate deg/s
        "r_deg_s": round(state.r * 180 / 3.14159265, 2),  # yaw rate deg/s
        "throttle": state.throttle,
        "physics_engine": "manual",
    }
