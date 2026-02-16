"""
FastAPI server: WebSocket for telemetry + control. Physics loop at ~60 Hz.
Uses JSBSim for realistic flight dynamics.
"""

import asyncio
import json
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

import os

# Always import manual physics as fallback
from physics import AircraftState, update_physics, state_to_telemetry

# Allow forcing manual physics via environment variable
FORCE_MANUAL_PHYSICS = os.environ.get("FORCE_MANUAL_PHYSICS", "false").lower() == "true"

try:
    if FORCE_MANUAL_PHYSICS:
        raise ImportError("Manual physics forced via FORCE_MANUAL_PHYSICS environment variable")
    from jsbsim_wrapper import JSBSimWrapper
    JSBSIM_AVAILABLE = True
except ImportError as e:
    import logging
    if FORCE_MANUAL_PHYSICS:
        logging.info("Using manual physics (forced via environment variable)")
    else:
        logging.warning(f"JSBSim not available, falling back to manual physics: {e}")
    JSBSIM_AVAILABLE = False

# Global state
_jsbsim = None
_state = None  # For manual physics fallback
_control = {"throttle": 0.0, "elevator": 0.0, "aileron": 0.0, "rudder": 0.0}
_ws_connections: list[WebSocket] = []

# Auto-reset when outside these bounds (altitude in m, position in m)
ALTITUDE_MIN, ALTITUDE_MAX = -200.0, 50000.0
POSITION_ABS_MAX = 100_000.0


def get_initial_state():
    """Get initial state for manual physics."""
    if JSBSIM_AVAILABLE:
        return None
    return AircraftState(
        x=0.0, y=0.0, z=0.0,
        u=0.0, v=0.0, w=0.0,
        phi=0.0, theta=0.0, psi=0.0,
        p=0.0, q=0.0, r=0.0,
        throttle=0.0, elevator=0.0, aileron=0.0, rudder=0.0,
    )


def reset_sim() -> None:
    global _jsbsim, _state
    # Reset controls first
    _control["throttle"] = 0.0
    _control["elevator"] = 0.0
    _control["aileron"] = 0.0
    _control["rudder"] = 0.0
    if JSBSIM_AVAILABLE:
        try:
            if _jsbsim is None:
                _jsbsim = JSBSimWrapper("c172x")
            # Ensure controls are zero before reset
            if _jsbsim.initialized:
                _jsbsim.set_controls(0.0, 0.0, 0.0, 0.0)
            _jsbsim.reset()
        except Exception as e:
            import logging
            logging.error(f"JSBSim reset failed: {e}")
    else:
        _state = get_initial_state()


def _state_out_of_bounds(state: dict) -> bool:
    if not state:
        return True
    alt = state.get("altitude", 0.0)
    if alt < ALTITUDE_MIN or alt > ALTITUDE_MAX:
        return True
    x, y = state.get("x", 0.0), state.get("y", 0.0)
    if abs(x) > POSITION_ABS_MAX or abs(y) > POSITION_ABS_MAX:
        return True
    return False


async def physics_loop():
    global _jsbsim, _state
    dt = 1.0 / 60.0
    jsbsim_failed_count = 0
    MAX_JSBSIM_FAILURES = 10  # After 10 consecutive failures, switch to manual physics
    
    # Initialize based on available physics engine
    if JSBSIM_AVAILABLE:
        try:
            _jsbsim = JSBSimWrapper("c172x")
            _jsbsim.initialize(altitude_m=0.0, heading_deg=0.0, airspeed_ms=0.0)
            # Test if we can actually read properties
            test_state = _jsbsim.get_state()
            if test_state.get("altitude") == 0.0 and test_state.get("airspeed") == 0.0:
                # Check if this is because properties aren't accessible
                import logging
                logging.warning("JSBSim initialized but properties not accessible, falling back to manual physics")
                _jsbsim = None
            else:
                # Test JSBSim: set throttle to 50% and run a few steps, check if speed increases
                import logging
                logging.info("Testing JSBSim functionality...")
                _jsbsim.set_controls(0.5, 0.0, 0.0, 0.0)
                initial_state = _jsbsim.get_state()
                initial_speed = initial_state.get("airspeed", 0.0)
                logging.info(f"Initial state: speed={initial_speed:.2f} m/s")
                
                for i in range(60):  # 1 second at 60Hz
                    if not _jsbsim.step():
                        logging.error(f"JSBSim step failed during test at step {i}")
                        break
                    if i % 20 == 0:  # Log every 1/3 second
                        test_state = _jsbsim.get_state()
                        test_speed = test_state.get("airspeed", 0.0)
                        test_throttle = test_state.get("throttle", 0.0)
                        logging.info(f"Test step {i}: speed={test_speed:.2f} m/s, throttle={test_throttle:.2f}")
                
                test_state_after = _jsbsim.get_state()
                test_speed = test_state_after.get("airspeed", 0.0)
                speed_increase = test_speed - initial_speed
                logging.info(f"JSBSim test result: initial={initial_speed:.2f} m/s, final={test_speed:.2f} m/s, increase={speed_increase:.2f} m/s")
                
                if test_speed < 2.0 or speed_increase < 0.5:  # Should have some speed increase after 1 second at 50% throttle
                    logging.error(f"JSBSim FAILED TEST: throttle 50% but speed={test_speed:.2f} m/s (increase={speed_increase:.2f}) after 1s, switching to manual physics")
                    _jsbsim = None
                else:
                    # Reset after test
                    logging.info("JSBSim test passed, resetting to initial state")
                    _jsbsim.reset()
        except Exception as e:
            import logging
            logging.error(f"JSBSim initialization failed: {e}")
            # Fall through to manual physics
            _jsbsim = None
    
    if not JSBSIM_AVAILABLE or _jsbsim is None:
        _state = get_initial_state()
    
    while True:
        await asyncio.sleep(dt)
        try:
            if JSBSIM_AVAILABLE and _jsbsim is not None:
                # JSBSim path
                _jsbsim.set_controls(
                    _control["throttle"],
                    _control["elevator"],
                    _control["aileron"],
                    _control["rudder"]
                )
                if not _jsbsim.step():
                    jsbsim_failed_count += 1
                    if jsbsim_failed_count >= MAX_JSBSIM_FAILURES:
                        import logging
                        logging.error("JSBSim step failed repeatedly, switching to manual physics")
                        _jsbsim = None
                        _state = get_initial_state()
                    else:
                        reset_sim()
                    continue
                state = _jsbsim.get_state()
                
                # Check if state is valid (not all zeros due to property access failure)
                if state.get("altitude") == 0.0 and state.get("airspeed") == 0.0 and state.get("phi_deg") == 0.0:
                    jsbsim_failed_count += 1
                    if jsbsim_failed_count >= MAX_JSBSIM_FAILURES:
                        import logging
                        logging.error("JSBSim returning invalid state, switching to manual physics")
                        _jsbsim = None
                        _state = get_initial_state()
                    else:
                        continue
                
                # Detect problematic JSBSim behavior: throttle high but speed zero/low
                throttle = _control.get("throttle", 0.0)
                airspeed = state.get("airspeed", 0.0)
                altitude = state.get("altitude", 0.0)
                
                # More aggressive detection: if throttle > 30% for more than 1 second but speed < 3 m/s, switch to manual
                if throttle > 0.3 and airspeed < 3.0:
                    if not hasattr(physics_loop, '_jsbsim_slow_speed_count'):
                        physics_loop._jsbsim_slow_speed_count = 0
                    physics_loop._jsbsim_slow_speed_count += 1
                    if physics_loop._jsbsim_slow_speed_count > 60:  # ~1 second at 60Hz
                        import logging
                        logging.error(f"JSBSim FAILURE DETECTED: throttle={throttle:.2f} but speed={airspeed:.2f} m/s after 1s, switching to manual physics")
                        _jsbsim = None
                        _state = get_initial_state()
                        physics_loop._jsbsim_slow_speed_count = 0
                        # Force state to indicate manual physics
                        state["physics_engine"] = "manual"
                        continue
                else:
                    physics_loop._jsbsim_slow_speed_count = 0
                
                jsbsim_failed_count = 0  # Reset failure count on success
            else:
                # Manual physics fallback
                if _state is None:
                    _state = get_initial_state()
                _state = AircraftState(
                    **{**vars(_state), "throttle": _control["throttle"], "elevator": _control["elevator"],
                       "aileron": _control["aileron"], "rudder": _control["rudder"]}
                )
                _state = update_physics(_state, dt)
                state = state_to_telemetry(_state)
            
            # Check bounds
            if _state_out_of_bounds(state):
                reset_sim()
                if JSBSIM_AVAILABLE and _jsbsim:
                    state = _jsbsim.get_state()
                elif _state:
                    state = state_to_telemetry(_state)
            
            # Send telemetry
            msg = json.dumps(state)
            dead = []
            for ws in _ws_connections:
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                _ws_connections.remove(ws)
        except Exception as e:
            import logging
            logging.error(f"Physics loop error: {e}", exc_info=True)
            reset_sim()


@asynccontextmanager
async def lifespan(app: FastAPI):
    asyncio.create_task(physics_loop())
    yield


app = FastAPI(title="Flughermir Sim API", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/debug/jsbsim")
def debug_jsbsim():
    """Debug endpoint to check JSBSim status and properties."""
    import logging
    logger = logging.getLogger(__name__)
    
    if not JSBSIM_AVAILABLE:
        return {
            "jsbsim_available": False,
            "reason": "JSBSim not imported",
            "using_manual_physics": True
        }
    
    if _jsbsim is None:
        return {
            "jsbsim_available": True,
            "jsbsim_initialized": False,
            "reason": "JSBSim wrapper not created",
            "using_manual_physics": True
        }
    
    try:
        state = _jsbsim.get_state()
        # Try to read various properties
        diagnostics = {
            "jsbsim_available": True,
            "jsbsim_initialized": _jsbsim.initialized,
            "physics_engine": state.get("physics_engine", "unknown"),
            "current_state": {
                "altitude": state.get("altitude", 0.0),
                "airspeed": state.get("airspeed", 0.0),
                "throttle": state.get("throttle", 0.0),
            },
            "controls": _control.copy(),
        }
        
        # Try to read engine properties
        try:
            diagnostics["engine"] = {
                "running": _jsbsim._get_property("propulsion/engine/engine-running", None),
                "rpm": _jsbsim._get_property("propulsion/engine/engine-rpm", None),
                "thrust": _jsbsim._get_property("propulsion/engine/thrust-lbs", None),
            }
        except:
            diagnostics["engine"] = "Could not read engine properties"
        
        # Try to read throttle properties
        try:
            diagnostics["throttle_properties"] = {}
            for prop in ["fcs/throttle-cmd-norm", "fcs/throttle-pos-norm", "propulsion/engine/throttle"]:
                try:
                    diagnostics["throttle_properties"][prop] = _jsbsim._get_property(prop, None)
                except:
                    diagnostics["throttle_properties"][prop] = "not available"
        except:
            diagnostics["throttle_properties"] = "Could not read throttle properties"
        
        return diagnostics
    except Exception as e:
        logger.error(f"Error in debug endpoint: {e}", exc_info=True)
        return {
            "jsbsim_available": True,
            "error": str(e),
            "using_manual_physics": True
        }


@app.post("/reset")
def reset():
    """Reset simulation to initial state (e.g. after crash or runaway)."""
    reset_sim()
    return {"status": "reset"}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_connections.append(ws)
    # Send current state immediately so client gets first frame without waiting for next tick
    try:
        if JSBSIM_AVAILABLE and _jsbsim:
            state = _jsbsim.get_state()
        elif _state:
            state = state_to_telemetry(_state)
        else:
            reset_sim()
            if JSBSIM_AVAILABLE and _jsbsim:
                state = _jsbsim.get_state()
            elif _state:
                state = state_to_telemetry(_state)
            else:
                state = {}
        await ws.send_text(json.dumps(state))
    except Exception:
        pass
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
                if "throttle" in data:
                    _control["throttle"] = max(0.0, min(1.0, float(data["throttle"])))
                if "elevator" in data:
                    _control["elevator"] = max(-1.0, min(1.0, float(data["elevator"])))
                if "aileron" in data:
                    _control["aileron"] = max(-1.0, min(1.0, float(data["aileron"])))
                if "rudder" in data:
                    _control["rudder"] = max(-1.0, min(1.0, float(data["rudder"])))
            except (json.JSONDecodeError, TypeError, ValueError):
                pass
    except WebSocketDisconnect:
        pass
    finally:
        if ws in _ws_connections:
            _ws_connections.remove(ws)
