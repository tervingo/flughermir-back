"""
JSBSim wrapper: interfaces with JSBSim flight dynamics model.
Uses JSBSim Python bindings to run physics simulation.
"""

import os
from jsbsim import FGFDMExec


class JSBSimWrapper:
    """Wrapper around JSBSim FDM for aircraft simulation."""

    def __init__(self, aircraft: str = "c172x"):
        """
        Initialize JSBSim with an aircraft model.
        Common models: c172x, f15, f16, etc.
        """
        self.fdm = FGFDMExec()
        self.fdm.load_model(aircraft)
        self.dt = 1.0 / 60.0  # 60 Hz
        self.fdm.set_dt(self.dt)  # Set timestep
        self.initialized = False

    def initialize(self, altitude_m: float = 0.0, heading_deg: float = 0.0, airspeed_ms: float = 0.0):
        """Initialize aircraft state (on runway, ready for takeoff)."""
        # Set initial conditions
        self.fdm["ic/h-sl-ft"] = altitude_m * 3.28084  # m to ft
        self.fdm["ic/long-gc-deg"] = 0.0
        self.fdm["ic/lat-gc-deg"] = 0.0
        self.fdm["ic/psi-true-deg"] = heading_deg
        self.fdm["ic/u-fps"] = airspeed_ms * 3.28084  # m/s to ft/s
        self.fdm["ic/v-fps"] = 0.0
        self.fdm["ic/w-fps"] = 0.0
        self.fdm["ic/phi-deg"] = 0.0
        self.fdm["ic/theta-deg"] = 0.0
        self.fdm["ic/alpha-deg"] = 0.0
        self.fdm["ic/beta-deg"] = 0.0
        self.fdm.run_ic()
        self.initialized = True

    def set_controls(self, throttle: float, elevator: float, aileron: float, rudder: float):
        """Set control surfaces (all normalized -1..1, throttle 0..1)."""
        self.fdm["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, throttle))
        self.fdm["fcs/elevator-cmd-norm"] = max(-1.0, min(1.0, elevator))
        self.fdm["fcs/aileron-cmd-norm"] = max(-1.0, min(1.0, aileron))
        self.fdm["fcs/rudder-cmd-norm"] = max(-1.0, min(1.0, rudder))

    def step(self) -> bool:
        """Advance simulation by one timestep. Returns True if successful."""
        if not self.initialized:
            return False
        try:
            return self.fdm.run()
        except Exception:
            return False

    def get_state(self) -> dict:
        """Extract current aircraft state for telemetry."""
        # JSBSim uses NED frame, feet, degrees
        # Convert to SI units (meters, m/s, degrees)
        try:
            # Try common property names
            x_ft = self.fdm["position/local-x-ft"]
            y_ft = self.fdm["position/local-y-ft"]
            z_ft = self.fdm["position/local-z-ft"]
            alt_ft = self.fdm["position/altitude-agl-ft"]
            phi = self.fdm["attitude/phi-deg"]
            theta = self.fdm["attitude/theta-deg"]
            psi = self.fdm["attitude/psi-deg"]
            u_fps = self.fdm["velocities/u-fps"]
            v_fps = self.fdm["velocities/v-fps"]
            w_fps = self.fdm["velocities/w-fps"]
            h_dot_fps = self.fdm["velocities/h-dot-fps"]
            p_rad = self.fdm["velocities/p-rad_sec"]
            q_rad = self.fdm["velocities/q-rad_sec"]
            r_rad = self.fdm["velocities/r-rad_sec"]
            # True airspeed (better than just u)
            tas_fps = self.fdm["velocities/vt-fps"]
            throttle = self.fdm["fcs/throttle-cmd-norm"]
        except (KeyError, AttributeError) as e:
            # Fallback if properties don't exist
            return {
                "x": 0.0, "y": 0.0, "z": 0.0,
                "altitude": 0.0,
                "phi_deg": 0.0, "theta_deg": 0.0, "psi_deg": 0.0,
                "airspeed": 0.0,
                "vertical_speed": 0.0,
                "p_deg_s": 0.0, "q_deg_s": 0.0, "r_deg_s": 0.0,
                "throttle": throttle if 'throttle' in locals() else 0.0,
            }
        
        return {
            "x": x_ft * 0.3048,  # ft to m
            "y": y_ft * 0.3048,
            "z": z_ft * 0.3048,
            "altitude": alt_ft * 0.3048,  # AGL in meters
            "phi_deg": phi,
            "theta_deg": theta,
            "psi_deg": psi,
            "airspeed": tas_fps * 0.3048,  # true airspeed in m/s
            "vertical_speed": -h_dot_fps * 0.3048,  # m/s, positive = climbing (h-dot is down)
            "p_deg_s": p_rad * 57.2958,  # rad/s to deg/s
            "q_deg_s": q_rad * 57.2958,
            "r_deg_s": r_rad * 57.2958,
            "throttle": throttle,
        }

    def reset(self):
        """Reset to initial state."""
        self.initialize(altitude_m=0.0, heading_deg=0.0, airspeed_ms=0.0)
