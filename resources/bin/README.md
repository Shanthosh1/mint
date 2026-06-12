# mavp2p binaries

MINT uses [mavp2p](https://github.com/bluenviron/mavp2p) (MIT license,
static Go binary) as its telemetry router. It was chosen over
mavlink-routerd, whose epoll-based core does not run on Windows or macOS.

Drop the platform-specific executable in this folder before building the
bundle:

| Platform | Filename     | Source                                                              |
|----------|--------------|---------------------------------------------------------------------|
| Linux    | `mavp2p`     | prebuilt: https://github.com/bluenviron/mavp2p/releases (amd64/arm) |
| Windows  | `mavp2p.exe` | prebuilt: same releases page (`windows_amd64.zip`)                  |
| macOS    | `mavp2p`     | no official prebuilt — build with Go (one command, see below)       |

macOS build (requires Go >= 1.21, e.g. `brew install go`):

```bash
go install github.com/bluenviron/mavp2p@latest
cp "$(go env GOPATH)/bin/mavp2p" resources/bin/
```

Only the binary matching the build host is required for a given bundle.
The app resolves the correct name at runtime via `platform.system()` and
surfaces a clear "router binary not installed" error (HTTP 424) if missing.

Launch command used by the app (see `backend/app/mavlink/router_manager.py`):

    mavp2p --streamreq-disable --hb-disable \
        serial:<device>:<baud> \
        udpc:127.0.0.1:14550 \   # → QGroundControl
        udpc:127.0.0.1:14540 \   # → MAVSDK-Python (params / commands)
        udpc:127.0.0.1:14541     # → pymavlink raw stream (analysis engines)

Flags: `--streamreq-disable` because PX4 streams telemetry without
ArduPilot-style data-stream requests; `--hb-disable` so mavp2p does not
inject its own GCS heartbeats alongside QGC's.
