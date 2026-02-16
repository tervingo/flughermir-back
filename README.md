# Flughermir Backend

FastAPI server with WebSocket telemetry and JSBSim flight dynamics model (Phase 2).
Uses JSBSim for realistic aircraft physics (Cessna 172 model included).

## Setup

```bash
python -m venv venv
# Windows:
venv\Scripts\activate
# Then:
pip install -r requirements.txt
```

## Run

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

- Health: GET /health
- Reset: POST /reset — resets aircraft and controls to initial state (also used automatically when altitude or position go out of bounds)
- WebSocket: ws://localhost:8000/ws (send JSON with throttle, elevator, aileron, rudder; receive telemetry at ~60 Hz)

The physics loop runs at 60 Hz using JSBSim whether or not any client is connected. If altitude goes below -200 m or above 50 km, or position beyond ±100 km, the sim auto-resets.

## JSBSim

Uses JSBSim Python bindings (installed via `pip install JSBSim`). The default aircraft model is `c172x` (Cessna 172). To use a different model, change the aircraft name in `main.py` (e.g., `JSBSimWrapper("f15")`).

### Known Issues

JSBSim may have issues with:
- Controls not responding correctly (throttle not increasing speed)
- Excessive bouncing on ground
- Property access failures

The system automatically detects these issues and falls back to manual physics. You can also force manual physics by setting the environment variable:

```bash
export FORCE_MANUAL_PHYSICS=true
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The manual physics engine provides a simpler but functional flight model that works reliably.
