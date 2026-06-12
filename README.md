# MINT — MAVLink Intelligent Tuning Assistant

MINT is a local, cross-platform ground station assistant designed to help field pilots tune PX4-based unmanned vehicles safely and analyze flight logs. It runs alongside ground control software (like QGroundControl) on the ground station laptop, dynamically splitting serial telemetry inputs into concurrent, independent streams for the pilot's GCS and its own real-time analysis engines.

---

## 🌟 Key Features

* **Real-time Rate-Loop Analysis**: Computes non-dimensional time constants ($\tau$), Pearson correlation coefficients ($r$), and Normalized Root-Mean-Square Error (NRMSE) metrics on live attitude target vs. response data.
* **EKF Diagnostics & Innovation Watchdog**: Monitors EKF status reports, tracking stick inputs and regimes to diagnose magnetic, vibration, and sensor phase-lag issues.
* **Offline ULog Analysis Pipeline**: Supports bulk uploads of flight logs (>100 MB), parses gyroscope FFT spectra, isolates vibration peaks, and provides coordinated notch/cutoff filter recommendations.
* **Human-In-The-Loop Safety Model**: Enforces double-validation on all parameter changes. MINT *never* writes a parameter automatically—every recommendation is staged as a proposal requiring explicit pilot approval.
* **Dynamic Connection Manager**: Runs a telemetry proxy subprocess to fan out serial or UDP MAVLink feeds to QGC (14550), MAVSDK control (14540), and raw data analyzers (14541) with auto-resolved port collision handling.

---

## 📐 Architecture

```
                    ┌────────────────────────────────────────────────────────┐
                    │                     Ground Station                     │
                    │  mavp2p router (managed subprocess)                    │
  Source            │    ├── UDP 127.0.0.1:14550 ──▶ QGroundControl          │
  ──────────▶       │    ├── UDP 127.0.0.1:14540 ──▶ MAVSDK   (control plane)│
  Serial / USB      │    └── UDP 127.0.0.1:14541 ──▶ pymavlink (data plane)  │
  UDP / TCP         │                                                        │
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

## 🛡️ Safety Model (Non-Negotiable Invariants)

1. **No Automated Writes**: MINT strictly generates *proposals*. Writes to the autopilot are only executed when the pilot explicitly clicks **Approve & Write**.
2. **Whitelist Registry**: Only parameters registered in `backend/app/core/safety_registry.json` for the auto-detected airframe type are allowed to be modified.
3. **Double Verification**: Proposed delta changes are validated at creation and re-validated against the live on-vehicle state at write-time (blocking the action if the baseline parameter was modified externally).
4. **Airframe Class Isolation**: Parameter proposals are disabled until the vehicle's `SYS_AUTOSTART` or `MAV_TYPE` is successfully read and mapped.

---

## 🛠️ Development Setup

### Backend Setup (Python >= 3.10)
```bash
# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run backend (dev mode - port 8400)
python backend/main.py
```

### Frontend Setup (Node.js >= 18)
In a separate terminal shell:
```bash
cd frontend
npm install
npm run dev
```

### Telemetry Router Binary
MINT bundles `mavp2p` as its telemetry proxy. Build or copy your platform-specific binary into `resources/bin/` first (see [resources/bin/README.md](resources/bin/README.md) for compiling/obtaining prebuilt binaries).

---

## 📦 Building the Executable

MINT is built as a single-folder distribution (`ONEDIR` target to avoid expensive decompression latency of SciPy/MAVSDK packages on every launch).

### Recommended (Automated)
Run the automated build script matching your operating system in the project root:

* **macOS / Linux**:
  ```bash
  ./build.sh
  ```
* **Windows (PowerShell)**:
  ```powershell
  ./build.ps1
  ```

### Manual Steps
If you prefer to run the steps manually:

```bash
# 1. Compile the React UI
cd frontend && npm install && npm run build && cd ..

# 2. Package the bundle
pyinstaller mint.spec
# Output folder will be created at → dist/mint/
```

### Automated CI/CD
MINT is configured with a GitHub Actions pipeline ([.github/workflows/build.yml](.github/workflows/build.yml)) to build, package, and upload zipped binaries for **Windows**, **macOS**, and **Linux** on every push.

---

## 🤖 AI Credits & Disclaimer

This application was developed with the assistance of agentic AI coding tools. AI assistance was primarily utilized to accelerate React frontend development and optimize overall coding standards and patterns across the codebase.

---

## 📄 License

MINT is released under the [MIT License](LICENSE).
