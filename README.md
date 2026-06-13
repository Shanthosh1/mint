# MINT: MAVLink Intelligent Tuning Assistant

MINT is a desktop companion tool for pilots running the PX4 autopilot. It runs locally on your ground station laptop alongside QGroundControl to help you safely tune control loops and analyze flight logs in the field.

To make things easy, MINT runs a small telemetry proxy (`mavp2p`) in the background. You plug in your vehicle's telemetry link, and MINT splits it into separate streams for your Ground Control Station (GCS) and its own real-time analysis engines—meaning you don't have to fiddle with serial ports or lose your telemetry connection.

> [!WARNING]
> **Operational Safety Warning: Do Not Close MINT While Flying!**
> 
> Because MINT acts as the telemetry proxy routing data to QGroundControl, **closing MINT cleanly while your vehicle is in the air will cut the telemetry feed to QGC**. QGC will immediately report a connection loss, and the vehicle will trigger its configured **Data Link Failsafe** (typically executing Return-to-Launch, Land, or Hover depending on your PX4 parameters).
> 
> Always land and disarm your vehicle before closing the MINT application.

---

## Table of Contents
* [Supported Vehicles & Software](#-supported-vehicles--software)
* [Core Features](#-core-features)
* [How it Works](#-how-it-works)
* [The Safety Invariants](#️-the-safety-invariants)
* [Local Development](#️-local-development)
* [Building the Executable](#-building-the-executable)
* [Technical Reference & Operational Details](#-technical-reference--operational-details)
* [AI Credits & Disclaimer](#-ai-credits--disclaimer)
* [License](#-license)

---

## Supported Vehicles & Software
* **Autopilot**: PX4 firmware v1.14 or newer.
* **Airframes**: Multirotors, standard Fixed-Wing planes, Flying Wings/Deltas, and VTOLs.
* **Unsupported Vehicles**: Ground rovers, boats, submarines, balloons, and helicopters are explicitly blocked. Because their control structures differ so much from standard fixed-wing and multirotor rate cascades, applying MINT's advice to them would be unsafe.

---

## Core Features

* **Real-Time PID Rate Tuning**: Monitors live pilot inputs vs. the vehicle's actual response. MINT measures response speed (time constant $\tau$), tracking quality (correlation $r$), and overshoot to suggest concrete gain changes.
* **Offline ULog Analysis**: Upload your flight logs (even large files) to get a full post-flight report. It runs FFTs on your gyro data to identify vibration spikes and recommend exact notch/cutoff filter frequencies.
* **VTOL Support**: For VTOL aircraft, the log analyzer dynamically splits its analysis between multicopter (hover) and fixed-wing (forward flight) phases so you don't mix up your parameters.
* **EKF Diagnostics**: Keeps an eye on EKF status and pilot stick movements to diagnose vibration, sensor lag, or compass interference issues.
* **Safety First**: MINT will *never* write a parameter to your vehicle automatically. It only stages modifications as "proposals" in a staging area. You have to review and approve them before anything is sent to the vehicle.

---

## How it Works

```
                    ┌────────────────────────────────────────────────────────┐
                    │                     Ground Station                     │
                    │  mavp2p router (managed subprocess)                    │
                    │    ├── UDP 127.0.0.1:14550 ──▶ QGroundControl          │
                    │    ├── UDP 127.0.0.1:14540 ──▶ MAVSDK   (control plane)│
                    │    └── UDP 127.0.0.1:14541 ──▶ pymavlink (data plane)  │
                    │                                                        │
                    │  FastAPI Backend (asyncio + thread pools)              │
                    │    ├── TelemetryHub (Pub/Sub Event Bus)                │
                    │    ├── LivePidEngine · EkfMonitor · CascadeAnalyzer    │
                    │    ├── ULog Pipeline (pyulog + pandas + scipy)         │
                    │    └── REST / WebSocket APIs                           │
                    │                                                        │
                    │  React SPA (uPlot, Dark Glassmorphic UI)               │
                    └────────────────────────────────────────────────────────┘
```

---

## The Safety Invariants

MINT is built around a few non-negotiable safety rules to protect your vehicle:
1. **Strictly Manual Writes**: All recommendations are staged. No parameters are changed unless you click **Approve & Write**.
2. **Whitelist Protection**: You can only modify parameters that are registered in `backend/app/core/safety_registry.json` for your specific vehicle class.
3. **Pre-Write Verification**: When you write a parameter, MINT verifies it against the current vehicle state first. If the baseline changed on the vehicle since the recommendation was made, the write is blocked.
4. **Locked Airframe Mapping**: You cannot write or stage parameters until MINT successfully identifies the vehicle's `SYS_AUTOSTART` or `MAV_TYPE`.

---

## Local Development

### 1. Backend Setup (Python >= 3.10)
```bash
# Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run backend (runs on port 8400 by default)
python backend/main.py
```

### 2. Frontend Setup (Node.js >= 18)
In a separate terminal:
```bash
cd frontend
npm install
npm run dev
```

### 3. Telemetry Router Binary
MINT relies on `mavp2p` to route telemetry. Place a platform-specific binary of `mavp2p` in `resources/bin/` (see [resources/bin/README.md](resources/bin/README.md) for build instructions).

---

## Building the Executable

MINT is packaged as a single-folder executable (to avoid slow decompression of heavy SciPy/Pandas dependencies on every launch).

### Automated Builds
We've included scripts to handle everything (frontend build, virtual environment setup, packaging) in one go:
* **macOS / Linux**:
  ```bash
  ./build.sh
  ```
* **Windows (PowerShell)**:
  ```powershell
  ./build.ps1
  ```

### Manual Packaging
If you want to run the build steps yourself:
```bash
# Build the React frontend SPA
cd frontend && npm install && npm run build && cd ..

# Package with PyInstaller
.venv/bin/python -m PyInstaller mint.spec
# The output will be located in dist/mint/
```

We also have a GitHub Actions workflow configured in [.github/workflows/build.yml](.github/workflows/build.yml) that builds and uploads zipped binaries for Windows, macOS, and Linux on every release or push.

---

## Technical Reference & Operational Details

### 1. Telemetry Rate Limits
* **Maximum Telemetry Rate**: MINT can comfortably process telemetry rates up to **200 Hz** over the loopback network. 
* **Recommended Rate**: For physical telemetry radios (such as SiK radios), it is recommended to set your MAVLink stream rates (e.g. `SR0_*` parameters in PX4) between **50 Hz and 100 Hz** to avoid saturating the radio bandwidth while maintaining high-fidelity data resolution for real-time PID analysis.

### 2. Multi-Source Telemetry Routing
* **Collision Behavior**: MINT's backend and the underlying `mavp2p` router route telemetry packages based on MAVLink System ID (SYSID). 
* **Routing Logic**:
  * If two telemetry streams with **different SYSIDs** are received, MINT treats them as separate vehicles.
  * If two telemetry streams (e.g. two radios transmitting on the same network frequency/ports) send messages under the **same SYSID**, packets from both vehicles will interleave on the same channel. This will cause state thrashing, false EKF alerts, and PID analysis anomalies. **Always ensure each vehicle on the telemetry link has a unique SYSID.**

### 3. Running with SITL (Software-in-the-Loop)
To run MINT locally against a simulated PX4 SITL drone for development or testing:
1. **Launch SITL**: Start your PX4 SITL simulation. By default, it will broadcast a MAVLink stream on UDP port `14540` (and sometimes `14550`).
2. **Start MINT**: In the MINT UI, go to the connection configuration and specify the connection mode as **UDP Listen (Bind)** on port `14540`.
3. **Automatic Routing**: MINT's `mavp2p` proxy will capture the SITL stream, route it internally to MINT's analysis modules, and expose a forwarded output on `127.0.0.1:14550`.
4. **Connect QGC**: Open QGroundControl on the same computer. It will automatically connect to the forwarded stream on `14550`, allowing you to fly the simulated drone while MINT performs real-time PID and EKF analysis.

### 4. API Response & Error References
MINT's API uses standard HTTP response codes to communicate errors:
* **`413 Payload Too Large`**: Returned by the ULog upload pipeline (`/api/ulog`) if the uploaded `.ulg` file size exceeds the configured `ULOG_MAX_MIB` (default: 800 MiB).
* **`422 Unprocessable Entity`**: Returned during vehicle handshakes or log analysis if:
  * The autopilot stack is not PX4 (e.g., ArduPilot).
  * The firmware version is older than PX4 v1.14.
  * The airframe class resolves to an out-of-scope vehicle (rover, boat, submarine, balloon, airship).
* **`409 Conflict`**: Returned by system connection endpoints if you attempt to start a telemetry connection before the backend's `mavp2p` router subprocess is running.

---

## AI Credits & Disclaimer

This application was developed with the assistance of agentic AI coding tools. Agentic AI was utilized to design and build the entire React frontend, and to optimize the Python backend codebase, taking it from an initial proof-of-concept (POC) state into a robust, deployment-ready application.

---

## License

MINT is open-source software released under the [MIT License](LICENSE).
