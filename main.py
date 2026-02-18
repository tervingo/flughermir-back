"""
FastAPI server: WebSocket for telemetry + control. Physics loop at ~60 Hz.
"""

import asyncio
import json
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from physics import AircraftState, update_physics, state_to_telemetry

# Global state
_state: AircraftState | None = None
_control = {"throttle": 0.0, "elevator": 0.0, "aileron": 0.0, "rudder": 0.0}
_ws_connections: list[WebSocket] = []

# Auto-reset bounds
ALTITUDE_MIN, ALTITUDE_MAX = -200.0, 50000.0
POSITION_ABS_MAX = 100_000.0


def _make_initial_state() -> AircraftState:
    return AircraftState(
        x=0.0, y=0.0, z=0.0,
        u=0.0, v=0.0, w=0.0,
        phi=0.0, theta=0.0, psi=0.0,
        p=0.0, q=0.0, r=0.0,
        throttle=0.0, elevator=0.0, aileron=0.0, rudder=0.0,
    )


def reset_sim() -> None:
    global _state
    _control["throttle"] = 0.0
    _control["elevator"] = 0.0
    _control["aileron"] = 0.0
    _control["rudder"] = 0.0
    _state = _make_initial_state()


def _out_of_bounds(state: dict) -> bool:
    alt = state.get("altitude", 0.0)
    if alt < ALTITUDE_MIN or alt > ALTITUDE_MAX:
        return True
    if abs(state.get("x", 0.0)) > POSITION_ABS_MAX or abs(state.get("y", 0.0)) > POSITION_ABS_MAX:
        return True
    return False


async def physics_loop():
    global _state
    dt = 1.0 / 60.0
    _state = _make_initial_state()
    while True:
        await asyncio.sleep(dt)
        try:
            _state = AircraftState(
                **{**vars(_state),
                   "throttle": _control["throttle"],
                   "elevator": _control["elevator"],
                   "aileron":  _control["aileron"],
                   "rudder":   _control["rudder"]}
            )
            _state = update_physics(_state, dt)
            state = state_to_telemetry(_state)
            if _out_of_bounds(state):
                reset_sim()
                state = state_to_telemetry(_state)
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
    reset_sim()
    return {"status": "reset"}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_connections.append(ws)
    # Send current state immediately so the client gets the first frame without
    # waiting up to 1/60 s for the next physics tick.
    try:
        if _state:
            await ws.send_text(json.dumps(state_to_telemetry(_state)))
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
