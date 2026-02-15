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

- Health: http://localhost:8000/health
- WebSocket: ws://localhost:8000/ws (send JSON with throttle, elevator, aileron, rudder; receive telemetry at ~60 Hz)
