# Flughermir Backend

FastAPI server with WebSocket telemetry and minimal flight physics.

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

The physics loop runs at 60 Hz whether or not any client is connected. If altitude goes below -200 m or above 50 km, or position beyond ±100 km, the sim auto-resets.
