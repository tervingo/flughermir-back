"""
JSBSim wrapper: interfaces with JSBSim flight dynamics model.
Uses JSBSim Python bindings to run physics simulation.
"""

import logging
import os
from jsbsim import FGFDMExec

logger = logging.getLogger(__name__)


class JSBSimWrapper:
    """Wrapper around JSBSim FDM for aircraft simulation."""

    def __init__(self, aircraft: str = "c172x"):
        """
        Initialize JSBSim with an aircraft model.
        Common models: c172x, f15, f16, etc.
        """
        try:
            # FGFDMExec requires root directory path (empty string uses default)
            root_dir = os.environ.get("JSBSIM_ROOT", "")
            self.fdm = FGFDMExec(root_dir) if root_dir else FGFDMExec("")
            logger.info(f"Loading JSBSim aircraft model: {aircraft}")
            self.fdm.load_model(aircraft)
            self.dt = 1.0 / 60.0  # 60 Hz
            self.fdm.set_dt(self.dt)  # Set timestep
            self.initialized = False
            logger.info(f"JSBSim initialized successfully with aircraft {aircraft}")
        except Exception as e:
            logger.error(f"Failed to initialize JSBSim with aircraft {aircraft}: {e}", exc_info=True)
            raise

    def initialize(self, altitude_m: float = 0.0, heading_deg: float = 0.0, airspeed_ms: float = 0.0):
        """Initialize aircraft state (on runway, ready for takeoff)."""
        try:
            # Set initial conditions - ensure aircraft is on ground
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
            
            # Ensure gear is down and locked
            try:
                self.fdm["gear/gear-cmd-norm"] = 1.0  # Gear down
            except (KeyError, AttributeError):
                pass  # Some aircraft models may not have this property
            
            self.fdm.run_ic()
            
            # Stabilize on ground by running multiple steps with zero controls
            # This allows the aircraft to settle on the ground and stop bouncing
            self.fdm["fcs/throttle-cmd-norm"] = 0.0
            self.fdm["fcs/elevator-cmd-norm"] = 0.0
            self.fdm["fcs/aileron-cmd-norm"] = 0.0
            self.fdm["fcs/rudder-cmd-norm"] = 0.0
            
            # Run multiple steps to stabilize - check if on ground
            settled = False
            for i in range(30):  # More steps to ensure stability
                self.fdm.run()
                # Check if aircraft is on ground (altitude AGL should be near zero)
                try:
                    alt_agl_ft = self.fdm["position/altitude-agl-ft"]
                    if alt_agl_ft < 0.1:  # Very close to ground
                        settled = True
                        break  # Aircraft has settled
                except (KeyError, AttributeError):
                    pass  # Property may not exist, continue stabilizing
            
            # Log final state
            try:
                final_alt = self._get_property("position/altitude-agl-ft", 0.0)
                final_vt = self._get_property("velocities/vt-fps", 0.0)
                logger.info(f"JSBSim initialized: altitude AGL={final_alt*0.3048:.2f}m, speed={final_vt*0.3048:.2f}m/s, settled={settled}")
            except:
                pass
            
            self.initialized = True
            logger.info("JSBSim aircraft state initialized and stabilized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize aircraft state: {e}", exc_info=True)
            self.initialized = False
            raise

    def set_controls(self, throttle: float, elevator: float, aileron: float, rudder: float):
        """Set control surfaces (all normalized -1..1, throttle 0..1)."""
        try:
            self.fdm["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, throttle))
            self.fdm["fcs/elevator-cmd-norm"] = max(-1.0, min(1.0, elevator))
            self.fdm["fcs/aileron-cmd-norm"] = max(-1.0, min(1.0, aileron))
            self.fdm["fcs/rudder-cmd-norm"] = max(-1.0, min(1.0, rudder))
        except (KeyError, AttributeError) as e:
            logger.warning(f"Failed to set controls: {e}")

    def step(self) -> bool:
        """Advance simulation by one timestep. Returns True if successful."""
        if not self.initialized:
            return False
        try:
            return self.fdm.run()
        except Exception as e:
            logger.error(f"JSBSim step failed: {e}")
            return False

    def _get_property(self, prop_name: str, default=None):
        """Safely get a JSBSim property with fallback."""
        try:
            value = self.fdm[prop_name]
            return value
        except (KeyError, AttributeError, TypeError) as e:
            # Only log first few failures to avoid spam
            if not hasattr(self, '_logged_props'):
                self._logged_props = set()
            if prop_name not in self._logged_props and len(self._logged_props) < 5:
                logger.debug(f"Property {prop_name} not available: {e}")
                self._logged_props.add(prop_name)
            return default
    
    def get_state(self) -> dict:
        """Extract current aircraft state for telemetry."""
        # JSBSim uses NED frame, feet, degrees
        # Convert to SI units (meters, m/s, degrees)
        
        # Check if we can access basic properties - if not, JSBSim may not be properly initialized
        test_prop = self._get_property("position/h-sl-ft")
        if test_prop is None:
            # Try alternative property names
            test_prop = self._get_property("position/altitude-agl-ft")
            if test_prop is None:
                logger.warning("JSBSim properties not accessible - model may not be loaded correctly")
                # Return zero state to trigger fallback
                return {
                    "x": 0.0, "y": 0.0, "z": 0.0,
                    "altitude": 0.0,
                    "phi_deg": 0.0, "theta_deg": 0.0, "psi_deg": 0.0,
                    "airspeed": 0.0,
                    "vertical_speed": 0.0,
                    "p_deg_s": 0.0, "q_deg_s": 0.0, "r_deg_s": 0.0,
                    "throttle": 0.0,
                    "physics_engine": "manual",  # Indicates JSBSim failed
                }
        
        # Try multiple property name variations
        alt_ft = self._get_property("position/h-sl-ft") or self._get_property("position/altitude-agl-ft") or 0.0
        long_deg = self._get_property("position/long-gc-deg") or 0.0
        lat_deg = self._get_property("position/lat-gc-deg") or 0.0
        
        phi = self._get_property("attitude/phi-deg") or 0.0
        theta = self._get_property("attitude/theta-deg") or 0.0
        psi = self._get_property("attitude/psi-deg") or 0.0
        
        u_fps = self._get_property("velocities/u-fps") or 0.0
        v_fps = self._get_property("velocities/v-fps") or 0.0
        w_fps = self._get_property("velocities/w-fps") or 0.0
        h_dot_fps = self._get_property("velocities/h-dot-fps") or 0.0
        
        p_rad = self._get_property("velocities/p-rad_sec") or self._get_property("velocities/p-rad-sec") or 0.0
        q_rad = self._get_property("velocities/q-rad_sec") or self._get_property("velocities/q-rad-sec") or 0.0
        r_rad = self._get_property("velocities/r-rad_sec") or self._get_property("velocities/r-rad-sec") or 0.0
        
        tas_fps = self._get_property("velocities/vt-fps") or 0.0
        if tas_fps == 0.0:
            # Fallback: calculate from body velocities
            tas_fps = (u_fps**2 + v_fps**2 + w_fps**2)**0.5
        
        throttle = self._get_property("fcs/throttle-cmd-norm") or 0.0
        
        # Approximate local position from geodetic (simplified - use as relative to origin)
        # For MVP, we can use a simple approximation or track relative to start
        x_m = long_deg * 111320.0  # rough conversion: 1 deg longitude ≈ 111 km at equator
        y_m = lat_deg * 111320.0
        z_m = -alt_ft * 0.3048  # NED: z down, so altitude = -z
        
        return {
            "x": x_m,
            "y": y_m,
            "z": z_m,
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
            "physics_engine": "jsbsim",
        }

    def reset(self):
        """Reset to initial state."""
        try:
            # Stop the simulation first
            self.initialized = False
            # Reinitialize
            self.initialize(altitude_m=0.0, heading_deg=0.0, airspeed_ms=0.0)
        except Exception as e:
            logger.error(f"Failed to reset JSBSim: {e}", exc_info=True)
            raise
