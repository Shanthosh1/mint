"""
Lifecycle manager for the bundled mavp2p binary.

mavp2p (https://github.com/bluenviron/mavp2p) is a cross-platform Go
MAVLink router — chosen over mavlink-routerd, whose epoll-based core
does not run on Windows or macOS.

One MAVLink *source* is fanned out to up to two loopback UDP endpoints:

    source (serial / UDP / TCP)
        ├── 127.0.0.1:14550   QGroundControl
        └── 127.0.0.1:14541   pymavlink      (raw message firehose & control)

Supported source modes (ConnectionTarget.mode):
    serial       USB / radio modem            serial:<device>:<baud>
    udp_listen   vehicle/SITL sends to us     udps:<bind_ip>:<port>
    udp_connect  we connect to a UDP server   udpc:<host>:<port>
    tcp_connect  serial-over-ethernet, SITL   tcpc:<host>:<port>

When the source is a local UDP listener its port may collide with one of
the loopback fan-out ports (e.g. SITL's API stream on 14550 or 14541). Collisions
are resolved automatically: the QGC output is dropped when the source
already owns 14550 (QGC is then talking to the vehicle directly), and
the pymavlink output is shifted to alternate ports. The live
port assignment is exposed via `pymavlink_port` so the
connection layer always binds the right sockets.
"""
from __future__ import annotations

import asyncio
import logging
import subprocess
from dataclasses import dataclass, asdict, field
from typing import Literal, Optional

from ..core import config
from ..core.platform_utils import resolve_router_binary

log = logging.getLogger("mint.router")

ConnectionMode = Literal["serial", "udp_listen", "udp_connect", "tcp_connect"]

# Fallback ports used when a udp_listen source collides with a default.
_ALT_PYMAVLINK_PORT = 14543


@dataclass
class ConnectionTarget:
    mode: ConnectionMode
    # serial mode
    device: Optional[str] = None
    baud: int = config.DEFAULT_BAUD
    # network modes
    host: str = "0.0.0.0"       # bind address (udp_listen) or remote host
    port: int = 14550

    def source_endpoint(self) -> str:
        """mavp2p endpoint string for this source."""
        if self.mode == "serial":
            if not self.device:
                raise ValueError("serial mode requires a device")
            return f"serial:{self.device}:{self.baud}"
        if self.mode == "udp_listen":
            return f"udps:{self.host}:{self.port}"
        if self.mode == "udp_connect":
            return f"udpc:{self.host}:{self.port}"
        if self.mode == "tcp_connect":
            return f"tcpc:{self.host}:{self.port}"
        raise ValueError(f"Unknown connection mode {self.mode}")

    def describe(self) -> str:
        if self.mode == "serial":
            return f"{self.device} @ {self.baud} baud"
        labels = {"udp_listen": "UDP listen", "udp_connect": "UDP",
                  "tcp_connect": "TCP"}
        return f"{labels[self.mode]} {self.host}:{self.port}"

    def _listens_locally_on(self, port: int) -> bool:
        """True if this source binds `port` on an interface that includes
        loopback (any-address or explicit localhost)."""
        return (self.mode == "udp_listen" and self.port == port
                and self.host in ("0.0.0.0", "127.0.0.1", "localhost", "::"))


@dataclass
class RouterStatus:
    running: bool
    pid: Optional[int]
    source: Optional[str]            # human-readable source description
    mode: Optional[str]
    endpoints: list[str] = field(default_factory=list)
    qgc_forwarded: bool = True       # False when QGC talks to vehicle directly
    last_error: Optional[str] = None


class RouterManager:
    """Owns exactly one background mavp2p process."""

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None
        self._target: Optional[ConnectionTarget] = None
        self._last_error: Optional[str] = None
        self._lock = asyncio.Lock()
        # Active fan-out ports — re-resolved on every start().
        self.pymavlink_port: int = config.PYMAVLINK_UDP_PORT
        self._qgc_enabled: bool = True

    # -- public API ---------------------------------------------------------

    async def start(self, target: ConnectionTarget) -> RouterStatus:
        """Launch mavp2p for `target`. Idempotent while running."""
        async with self._lock:
            if self.is_running:
                return self.status()

            binary = resolve_router_binary()
            if binary is None:
                self._last_error = (
                    "mavp2p binary not found in resources/bin/. "
                    "See resources/bin/README.md for install instructions."
                )
                raise FileNotFoundError(self._last_error)

            self._resolve_ports(target)
            cmd = [
                str(binary),
                # PX4 streams telemetry unprompted; ArduPilot-style stream
                # requests would just add noise to the link.
                "--streamreq-disable",
                # Don't inject mavp2p's own GCS heartbeats — QGC and the
                # connection manager already provide them.
                "--hb-disable",
                target.source_endpoint(),
            ]
            if self._qgc_enabled:
                cmd.append(f"udpc:127.0.0.1:{config.QGC_UDP_PORT}")
            cmd.append(f"udpc:127.0.0.1:{self.pymavlink_port}")
            log.info("Launching router: %s", " ".join(cmd))

            try:
                self._proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    # Detach from our console on Windows so closing the app
                    # window doesn't orphan a console popup.
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            except PermissionError as exc:
                self._last_error = (
                    f"Permission denied opening {target.describe()}. On Linux "
                    f"add your user to the 'dialout' group: sudo usermod -aG "
                    f"dialout $USER (then log out/in). Underlying error: {exc}"
                )
                raise
            except OSError as exc:
                self._last_error = f"Failed to launch mavp2p: {exc}"
                raise

            # Give the process a moment to fail fast on bad source config
            # (missing serial device, port already bound, DNS failure ...).
            await asyncio.sleep(0.8)
            if self._proc.poll() is not None:
                stderr = (self._proc.stderr.read() or b"").decode(errors="replace")
                self._proc = None
                self._last_error = f"mavp2p exited immediately: {stderr.strip()}"
                raise RuntimeError(self._last_error)

            self._target, self._last_error = target, None
            return self.status()

    async def stop(self) -> RouterStatus:
        """Terminate the router process gracefully, escalating to kill."""
        async with self._lock:
            if self._proc is not None:
                self._proc.terminate()
                try:
                    await asyncio.get_running_loop().run_in_executor(
                        None, self._proc.wait, 5
                    )
                except subprocess.TimeoutExpired:
                    log.warning("Router did not terminate; killing")
                    self._proc.kill()
                self._proc = None
            self._target = None
            return self.status()

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def status(self) -> RouterStatus:
        endpoints = []
        if self.is_running:
            if self._qgc_enabled:
                endpoints.append(f"127.0.0.1:{config.QGC_UDP_PORT} (QGC)")
            endpoints.append(f"127.0.0.1:{self.pymavlink_port} (analyzer)")
        return RouterStatus(
            running=self.is_running,
            pid=self._proc.pid if self.is_running else None,
            source=self._target.describe() if self._target else None,
            mode=self._target.mode if self._target else None,
            endpoints=endpoints,
            qgc_forwarded=self._qgc_enabled,
            last_error=self._last_error,
        )

    def status_dict(self) -> dict:
        return asdict(self.status())

    # -- internals ----------------------------------------------------------

    def _resolve_ports(self, target: ConnectionTarget) -> None:
        """Pick fan-out ports that cannot collide with a local UDP source.

        Typical case: PX4 SITL's API stream targets 14550. Listening there
        means QGC is already receiving the vehicle stream directly,
        so forwarding to it would only create a feedback loop.
        """
        self._qgc_enabled = not target._listens_locally_on(config.QGC_UDP_PORT)
        self.pymavlink_port = (
            _ALT_PYMAVLINK_PORT
            if target._listens_locally_on(config.PYMAVLINK_UDP_PORT)
            else config.PYMAVLINK_UDP_PORT
        )


ROUTER = RouterManager()
