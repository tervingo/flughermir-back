"""
FastAPI server: WebSocket for telemetry + control. Physics loop at ~60 Hz.
Uses JSBSim for realistic flight dynamics.
"""

import asyncio
import json
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

try:
    from jsbsim_wrapper import JSBSimWrapper
    JSBSIM_AVAILABLE = True
except ImportError as e:
    import logging
    logging.warning(f"JSBSim not available, falling back to manual physics: {e}")
    JSBSIM_AVAILABLE = False
    from physics import AircraftState, update_physics, state_to_telemetry

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
    _control["throttle"] = 0.0
    _control["elevator"] = 0.0
    _control["aileron"] = 0.0
    _control["rudder"] = 0.0
    if JSBSIM_AVAILABLE:
        try:
            if _jsbsim is None:
                _jsbsim = JSBSimWrapper("c172x")
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
    
    # Initialize based on available physics engine
    if JSBSIM_AVAILABLE:
        try:
            _jsbsim = JSBSimWrapper("c172x")
            _jsbsim.initialize(altitude_m=0.0, heading_deg=0.0, airspeed_ms=0.0)
        except Exception as e:
            import logging
            logging.error(f"JSBSim initialization failed: {e}")
            # Fall through to manual physics
            pass
    
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
                    reset_sim()
                    continue
                state = _jsbsim.get_state()
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
