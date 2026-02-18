"""
JSBSim wrapper: interfaces with JSBSim flight dynamics model.
Uses JSBSim Python bindings to run physics simulation.
"""

import logging
import os
import tempfile
import jsbsim as _jsbsim_pkg
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
            # FGFDMExec requires the root directory containing aircraft/engine data.
            # Fall back to the path bundled with the jsbsim pip package when the
            # environment variable is not set.
            root_dir = os.environ.get("JSBSIM_ROOT") or _jsbsim_pkg.get_default_root_dir()
            self._root_dir = root_dir
            self._aircraft = aircraft
            self.fdm = FGFDMExec(root_dir)
            
            # Try to create the output file JSBSim expects (JSBout172B.csv)
            # This prevents the "unable to open file" error
            # JSBSim will write to this file but we don't need to read it
            try:
                # Try current directory first (where JSBSim will look)
                output_file = "JSBout172B.csv"
                # Get the current working directory
                cwd = os.getcwd()
                output_path = os.path.join(cwd, output_file)
                with open(output_path, 'w') as f:
                    pass  # Create empty file
                logger.debug(f"Created JSBSim output file: {output_path}")
            except (PermissionError, OSError) as e:
                try:
                    # If that fails, try temp directory and create a symlink or change directory
                    temp_dir = tempfile.gettempdir()
                    output_file = os.path.join(temp_dir, "JSBout172B.csv")
                    with open(output_file, 'w') as f:
                        pass
                    logger.debug(f"Created JSBSim output file in temp: {output_file}")
                except Exception as e2:
                    logger.debug(f"Could not create JSBSim output file: {e}, {e2}")
            
            # Try to disable CSV output (if possible)
            try:
                self.fdm["simulation/output/log-rate-hz"] = 0
            except (KeyError, AttributeError):
                pass
            
            logger.info(f"Loading JSBSim aircraft model: {aircraft}")
            self.fdm.load_model(aircraft)
            
            self.dt = 1.0 / 60.0  # 60 Hz
            self.fdm.set_dt(self.dt)  # Set timestep
            self.initialized = False
            self._bounce_count = 0  # Track bouncing behavior
            self._last_altitude = None
            
            # Log available properties for debugging
            try:
                logger.info("Checking available JSBSim properties...")
                fcs_props = self.list_available_properties("fcs/")
                prop_props = self.list_available_properties("propulsion/")
                logger.info(f"Available FCS properties: {[p[0] for p in fcs_props]}")
                logger.info(f"Available propulsion properties: {[p[0] for p in prop_props]}")
            except Exception as e:
                logger.debug(f"Could not list properties: {e}")
            
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
            
            # Ensure engine is running (may be needed for some aircraft)
            engine_running_set = False
            for prop_name in ["propulsion/engine/set-running", "propulsion/engine[0]/set-running", 
                             "propulsion/engine/engine-running", "propulsion/engine[0]/engine-running"]:
                try:
                    self.fdm[prop_name] = 1  # Engine running
                    logger.info(f"Set engine running via property: {prop_name}")
                    engine_running_set = True
                    break
                except (KeyError, AttributeError):
                    continue
            
            if not engine_running_set:
                logger.warning("Could not set engine running - engine may not start")
            
            # Log engine state before run_ic
            try:
                engine_running = self._get_property("propulsion/engine/engine-running", None)
                engine_rpm = self._get_property("propulsion/engine/engine-rpm", None)
                logger.info(f"Before run_ic: engine_running={engine_running}, engine_rpm={engine_rpm}")
            except:
                pass
            
            self.fdm.run_ic()
            
            # Log engine state after run_ic
            try:
                engine_running = self._get_property("propulsion/engine/engine-running", None)
                engine_rpm = self._get_property("propulsion/engine/engine-rpm", None)
                throttle_cmd = self._get_property("fcs/throttle-cmd-norm", None)
                logger.info(f"After run_ic: engine_running={engine_running}, engine_rpm={engine_rpm}, throttle_cmd={throttle_cmd}")
            except:
                pass
            
            # Zero all controls before settling.
            self.fdm["fcs/throttle-cmd-norm"] = 0.0
            self.fdm["fcs/elevator-cmd-norm"] = 0.0
            self.fdm["fcs/aileron-cmd-norm"] = 0.0
            self.fdm["fcs/rudder-cmd-norm"] = 0.0

            # Let the landing-gear spring/damper model fully settle.
            # 120 steps = 2 seconds at 60 Hz.  Do NOT break early: the aircraft
            # starts at 0 ft AGL so alt_agl < 0.1 is true from step 1, which is
            # exactly why the old early-break caused only 1 stabilisation step to
            # run and left the gear oscillating.
            for _ in range(120):
                self.fdm.run()

            # Log final resting state for diagnostics.
            try:
                final_alt = self._get_property("position/altitude-agl-ft", 0.0)
                final_vt  = self._get_property("velocities/vt-fps", 0.0)
                logger.info(
                    f"JSBSim settled: altitude AGL={final_alt * 0.3048:.2f} m, "
                    f"speed={final_vt * 0.3048:.2f} m/s"
                )
            except Exception:
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
            throttle_val = max(0.0, min(1.0, throttle))
            elevator_val = max(-1.0, min(1.0, elevator))
            aileron_val = max(-1.0, min(1.0, aileron))
            rudder_val = max(-1.0, min(1.0, rudder))
            
            # Try multiple throttle property names
            throttle_set = False
            throttle_prop_used = None
            for prop_name in ["fcs/throttle-cmd-norm", "fcs/throttle-pos-norm", "propulsion/engine/throttle", "fcs/throttle-cmd"]:
                try:
                    self.fdm[prop_name] = throttle_val
                    throttle_set = True
                    throttle_prop_used = prop_name
                    break
                except (KeyError, AttributeError) as e:
                    continue
            
            if not throttle_set:
                # Fallback to standard property
                try:
                    self.fdm["fcs/throttle-cmd-norm"] = throttle_val
                    throttle_prop_used = "fcs/throttle-cmd-norm"
                    throttle_set = True
                except (KeyError, AttributeError) as e:
                    logger.warning(f"Could not set throttle - property not found: {e}")
            
            # Set other controls
            try:
                self.fdm["fcs/elevator-cmd-norm"] = elevator_val
            except (KeyError, AttributeError) as e:
                logger.debug(f"Could not set elevator: {e}")
            try:
                self.fdm["fcs/aileron-cmd-norm"] = aileron_val
            except (KeyError, AttributeError) as e:
                logger.debug(f"Could not set aileron: {e}")
            try:
                self.fdm["fcs/rudder-cmd-norm"] = rudder_val
            except (KeyError, AttributeError) as e:
                logger.debug(f"Could not set rudder: {e}")
            
            # Log throttle changes for debugging (only when throttle > 0)
            if throttle_val > 0.01:
                if not hasattr(self, '_last_logged_throttle'):
                    self._last_logged_throttle = 0.0
                if abs(throttle_val - getattr(self, '_last_logged_throttle', 0.0)) > 0.05:  # Log more frequently
                    try:
                        # Try multiple throttle property names to see what's actually set
                        throttle_reads = {}
                        for prop in ["fcs/throttle-cmd-norm", "fcs/throttle-pos-norm", "propulsion/engine/throttle"]:
                            try:
                                throttle_reads[prop] = self.fdm[prop]
                            except:
                                throttle_reads[prop] = None
                        
                        speed_fps = self._get_property("velocities/vt-fps", 0.0)
                        u_fps = self._get_property("velocities/u-fps", 0.0)
                        thrust_lbf = self._get_property("propulsion/engine/thrust-lbs", None)
                        rpm = self._get_property("propulsion/engine/engine-rpm", None)
                        engine_running = self._get_property("propulsion/engine/engine-running", None)
                        
                        logger.info(f"Controls SET: throttle_cmd={throttle_val:.2f} (used prop: {throttle_prop_used})")
                        logger.info(f"Controls READ: throttle_props={throttle_reads}, engine_running={engine_running}, rpm={rpm}, thrust={thrust_lbf}")
                        logger.info(f"State: speed={speed_fps*0.3048:.2f}m/s (vt), u={u_fps*0.3048:.2f}m/s (body)")
                        self._last_logged_throttle = throttle_val
                    except Exception as e:
                        logger.debug(f"Could not log control state: {e}", exc_info=True)
        except Exception as e:
            logger.warning(f"Failed to set controls: {e}", exc_info=True)

    def step(self) -> bool:
        """Advance simulation by one timestep. Returns True if successful."""
        if not self.initialized:
            return False
        try:
            # Log state before step (only occasionally to avoid spam)
            if not hasattr(self, '_step_count'):
                self._step_count = 0
            self._step_count += 1
            
            if self._step_count % 60 == 0:  # Log every second
                try:
                    throttle = self._get_property("fcs/throttle-cmd-norm", None)
                    speed = self._get_property("velocities/vt-fps", 0.0)
                    rpm = self._get_property("propulsion/engine/engine-rpm", None)
                    engine_running = self._get_property("propulsion/engine/engine-running", None)
                    thrust = self._get_property("propulsion/engine/thrust-lbs", None)
                    logger.debug(f"Step {self._step_count}: throttle={throttle}, speed={speed*0.3048:.2f}m/s, rpm={rpm}, engine={engine_running}, thrust={thrust}")
                except:
                    pass
            
            result = self.fdm.run()
            
            # Detect excessive bouncing (rapid altitude changes near ground)
            try:
                alt_agl_ft = self._get_property("position/altitude-agl-ft", None)
                if alt_agl_ft is not None and alt_agl_ft < 1.0:  # Near ground
                    if self._last_altitude is not None:
                        alt_change = abs(alt_agl_ft - self._last_altitude)
                        if alt_change > 0.1:  # Significant change
                            self._bounce_count += 1
                        else:
                            self._bounce_count = max(0, self._bounce_count - 1)
                    self._last_altitude = alt_agl_ft
                    
                    # If bouncing excessively, try to stabilize
                    if self._bounce_count > 50:
                        logger.warning("Excessive bouncing detected, attempting stabilization")
                        # Reset controls to zero and run a few steps
                        self.fdm["fcs/throttle-cmd-norm"] = 0.0
                        self.fdm["fcs/elevator-cmd-norm"] = 0.0
                        for _ in range(5):
                            self.fdm.run()
                        self._bounce_count = 0
            except:
                pass
            
            return result
        except Exception as e:
            logger.error(f"JSBSim step failed: {e}", exc_info=True)
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
    
    def _get_prop(self, *names, default=0.0):
        """Return the value of the first property name that exists.

        Unlike chaining with ``or``, this correctly handles a property whose
        value happens to be 0.0 (which is falsy in Python but perfectly valid
        for, e.g., altitude on the ground or a centred control surface).
        """
        for name in names:
            val = self._get_property(name)
            if val is not None:
                return val
        return default

    def list_available_properties(self, prefix: str = "fcs/") -> list:
        """List available properties with a given prefix (for debugging)."""
        # This is a helper for debugging - JSBSim doesn't have a direct way to list properties
        # but we can try common ones
        common_props = [
            "fcs/throttle-cmd-norm", "fcs/throttle-pos-norm", "fcs/throttle-cmd",
            "fcs/elevator-cmd-norm", "fcs/aileron-cmd-norm", "fcs/rudder-cmd-norm",
            "propulsion/engine/throttle", "propulsion/engine/thrust-lbs",
            "propulsion/engine/engine-rpm", "propulsion/engine/engine-running",
            "velocities/vt-fps", "velocities/u-fps", "velocities/v-fps", "velocities/w-fps",
            "position/h-sl-ft", "position/altitude-agl-ft",
            "attitude/phi-deg", "attitude/theta-deg", "attitude/psi-deg",
        ]
        available = []
        for prop in common_props:
            if prop.startswith(prefix):
                try:
                    value = self.fdm[prop]
                    available.append((prop, value))
                except (KeyError, AttributeError):
                    pass
        return available
    
    def get_state(self) -> dict:
        """Extract current aircraft state for telemetry."""
        # JSBSim uses NED frame, feet, degrees — convert to SI (metres, m/s, degrees).

        # Accessibility check: use explicit None comparison so a valid 0.0 (e.g.
        # altitude on the ground) is not misread as "property missing".
        if (self._get_property("position/h-sl-ft") is None
                and self._get_property("position/altitude-agl-ft") is None):
            logger.warning("JSBSim properties not accessible - model may not be loaded correctly")
            return {
                "x": 0.0, "y": 0.0, "z": 0.0,
                "altitude": 0.0,
                "phi_deg": 0.0, "theta_deg": 0.0, "psi_deg": 0.0,
                "airspeed": 0.0,
                "vertical_speed": 0.0,
                "p_deg_s": 0.0, "q_deg_s": 0.0, "r_deg_s": 0.0,
                "throttle": 0.0,
                "physics_engine": "manual",  # signals fallback to caller
            }

        # All reads below use _get_prop so that a property value of exactly 0.0
        # is returned as-is instead of falling through to the next candidate.
        alt_ft   = self._get_prop("position/h-sl-ft", "position/altitude-agl-ft")
        long_deg = self._get_prop("position/long-gc-deg")
        lat_deg  = self._get_prop("position/lat-gc-deg")

        phi   = self._get_prop("attitude/phi-deg")
        theta = self._get_prop("attitude/theta-deg")
        psi   = self._get_prop("attitude/psi-deg")

        u_fps     = self._get_prop("velocities/u-fps")
        v_fps     = self._get_prop("velocities/v-fps")
        w_fps     = self._get_prop("velocities/w-fps")
        h_dot_fps = self._get_prop("velocities/h-dot-fps")

        # JSBSim property name uses underscore (p-rad_sec), not hyphen.
        p_rad = self._get_prop("velocities/p-rad_sec", "velocities/p-rad-sec")
        q_rad = self._get_prop("velocities/q-rad_sec", "velocities/q-rad-sec")
        r_rad = self._get_prop("velocities/r-rad_sec", "velocities/r-rad-sec")

        tas_fps  = self._get_prop("velocities/vt-fps")
        if tas_fps == 0.0:
            tas_fps = (u_fps**2 + v_fps**2 + w_fps**2)**0.5

        throttle = self._get_prop("fcs/throttle-cmd-norm")

        # Approximate local position from geodetic coords relative to origin.
        x_m = long_deg * 111320.0  # 1 deg lon ≈ 111 km at equator
        y_m = lat_deg  * 111320.0
        z_m = -alt_ft  * 0.3048    # NED: z down → altitude = -z

        return {
            "x": x_m,
            "y": y_m,
            "z": z_m,
            "altitude": alt_ft * 0.3048,        # MSL in metres
            "phi_deg": phi,
            "theta_deg": theta,
            "psi_deg": psi,
            "airspeed": tas_fps * 0.3048,        # true airspeed m/s
            "vertical_speed": h_dot_fps * 0.3048,  # h-dot > 0 means climbing
            "p_deg_s": p_rad * 57.2958,
            "q_deg_s": q_rad * 57.2958,
            "r_deg_s": r_rad * 57.2958,
            "throttle": throttle,
            "physics_engine": "jsbsim",
        }

    def reset(self):
        """Hard reset: recreate the FDM for a guaranteed clean state.

        Calling run_ic() on an active simulation does not fully clear internal
        JSBSim state (engine RPM, gear contact forces, accumulated integrators).
        Recreating FGFDMExec is the only reliable way to get a blank slate.
        """
        try:
            self.initialized = False
            self.fdm = FGFDMExec(self._root_dir)
            self.fdm.load_model(self._aircraft)
            self.fdm.set_dt(self.dt)
            # Reset per-instance tracking state
            self._bounce_count = 0
            self._last_altitude = None
            self.initialize(altitude_m=0.0, heading_deg=0.0, airspeed_ms=0.0)
        except Exception as e:
            logger.error(f"Failed to reset JSBSim: {e}", exc_info=True)
            raise
