"""
MINT — MAVLink Intelligent Tuning Assistant.

Entry point for both `python backend/main.py` (development) and the
PyInstaller single-file bundle. Builds the FastAPI app, mounts the
compiled frontend, wires lifespan startup/shutdown of the background
engines, and opens the default browser pointed at the UI.
"""
from __future__ import annotations

import logging
import webbrowser
from contextlib import asynccontextmanager

import asyncio
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from app.advisors.param_advisor import ADVISOR
from app.analysis.cascade import CASCADE
from app.analysis.domains import ACTUATION
from app.analysis.ekf_monitor import EKF_MONITOR
from app.analysis.live_pid import LIVE_PID
from app.analysis.regime import REGIME
from app.analysis.stick_monitor import STICK_MONITOR
from app.analysis.vibration_live import VIB_GATE
from app.analysis.vtol_monitor import VTOL_MONITOR
from app.api import routes_params, routes_system, routes_ulog, ws
from app.core import config
from app.mavlink.connection import CONNECTION
from app.mavlink.router_manager import ROUTER

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)-24s %(levelname)-7s %(message)s",
)
log = logging.getLogger("mint")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Analysis engines subscribe to the hub immediately; they idle until
    # the router + vehicle connection start feeding events. The regime
    # classifier starts first — the others gate their verdicts on it.
    REGIME.start()
    LIVE_PID.start()
    EKF_MONITOR.start()
    STICK_MONITOR.start()
    ACTUATION.start()
    VTOL_MONITOR.start()
    CASCADE.start()
    VIB_GATE.start()
    async def _loop_latency_monitor():
        import time
        while True:
            t0 = time.monotonic()
            await asyncio.sleep(0.1)
            dt = time.monotonic() - t0 - 0.1
            if dt > 0.15:
                logging.getLogger("mint.perf").warning("FastAPI event loop blocked for %.1f ms", dt * 1000)

    latency_task = asyncio.create_task(_loop_latency_monitor(), name="latency-monitor")
    log.info("Analysis engines online. UI at http://%s:%s", config.HTTP_HOST, config.HTTP_PORT)
    yield
    latency_task.cancel()
    # Orderly teardown: vehicle link first, then the router subprocess.
    await CONNECTION.disconnect()
    await ROUTER.stop()
    REGIME.stop()
    LIVE_PID.stop()
    EKF_MONITOR.stop()
    ACTUATION.stop()
    VTOL_MONITOR.stop()
    CASCADE.stop()
    VIB_GATE.stop()
    ADVISOR.stop()


def create_app() -> FastAPI:
    app = FastAPI(title="MINT", version="0.1.0", lifespan=lifespan)

    # The UI is served same-origin in production; CORS is for `npm run dev`.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(routes_system.router)
    app.include_router(routes_params.router)
    app.include_router(routes_ulog.router)
    app.include_router(ws.router)

    @app.exception_handler(OSError)
    async def os_error_handler(_, exc: OSError):
        # Last-resort net for serial/socket trouble that escaped a route.
        return JSONResponse(status_code=500, content={
            "detail": f"OS error: {exc}. If this involves a serial port on "
                      f"Linux, check 'dialout' group membership.",
        })

    # Compiled SPA (frontend/dist) — present in the bundle, optional in dev.
    if config.FRONTEND_DIST.is_dir():
        app.mount("/", StaticFiles(directory=config.FRONTEND_DIST, html=True), name="ui")
    else:
        @app.get("/")
        def no_ui() -> dict:
            return {"detail": "Frontend not built. Run: cd frontend && npm install && npm run build"}

    return app


app = create_app()


if __name__ == "__main__":
    import os
    import threading
    import time

    if not os.environ.get("MINT_NO_BROWSER"):
        def open_browser():
            time.sleep(0.5)
            try:
                webbrowser.open(f"http://{config.HTTP_HOST}:{config.HTTP_PORT}")
            except Exception as e:
                log.warning("Failed to open browser automatically: %s", e)
        threading.Thread(target=open_browser, daemon=True).start()

    uvicorn.run(app, host=config.HTTP_HOST, port=config.HTTP_PORT, log_level="info")
