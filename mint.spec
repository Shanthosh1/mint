# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec — MINT cross-platform bundle.

Build (after `cd frontend && npm run build`):
    pyinstaller mint.spec
    -> dist/mint/            (distribute this folder, or zip it)
    -> dist/mint/mint        (the executable)

ONEDIR, deliberately: a onefile build must re-extract ~250 MB of
scipy/pandas/grpc/mavsdk_server to a temp dir on EVERY launch — measured
at 74 s on an M-series Mac, minutes on a low-spec field laptop. Onedir
unpacks once at build time and starts in seconds. Zip dist/mint/
for single-artifact distribution.

Bundles:
  * the FastAPI backend,
  * the compiled SPA (frontend/dist),
  * the platform mavp2p router binary (resources/bin),
  * the read-only safety registry JSON,
  * MAVSDK's mavsdk_server sidecar (no PyInstaller hook exists for it —
    omitting it makes vehicle connection fail only at runtime).
"""
import platform
from pathlib import Path

import mavsdk

ROOT = Path(SPECPATH)
MAVSDK_BIN = Path(mavsdk.__file__).parent / "bin"

datas = [
    (str(ROOT / "frontend" / "dist"), "frontend/dist"),
    (str(ROOT / "resources" / "bin"), "resources/bin"),
    # Frozen modules import as app.* (pathex=backend), so __file__-relative
    # resources must land under app/, NOT backend/app/.
    (str(ROOT / "backend" / "app" / "core" / "safety_registry.json"),
     "app/core"),
    # MAVSDK spawns this sidecar relative to its own package directory.
    (str(MAVSDK_BIN), "mavsdk/bin"),
]

hiddenimports = [
    "uvicorn.logging", "uvicorn.loops.auto", "uvicorn.protocols.http.auto",
    "uvicorn.protocols.websockets.auto", "uvicorn.lifespan.on",
    "pymavlink.dialects.v20.common",
    "scipy._cyutility",
]

excludes = ["tkinter", "matplotlib", "jinja2", "scipy.special._cdflib"]
if platform.system() != "Windows":
    excludes.extend([
        "serial.win32",
        "serial.tools.list_ports_windows",
        "win32ctypes",
    ])
if platform.system() != "Linux":
    excludes.extend([
        "serial.tools.list_ports_linux",
    ])
if platform.system() != "Darwin":
    excludes.extend([
        "serial.tools.list_ports_osx",
    ])

a = Analysis(
    [str(ROOT / "backend" / "main.py")],
    pathex=[str(ROOT / "backend")],
    datas=datas,
    hiddenimports=hiddenimports,
    excludes=excludes,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    exclude_binaries=True,       # onedir: binaries live in the COLLECT dir
    name="mint",
    console=True,                # field debugging: keep the log console
    upx=False,                   # UPX breaks scipy DLLs on Windows
    icon=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    name="mint",
    upx=False,
)
