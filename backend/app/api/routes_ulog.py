"""
Post-flight ULog upload + analysis endpoint.

The upload is spooled to disk in 1 MiB chunks (never fully buffered in
RAM) and the CPU-heavy pyulog/scipy pipeline runs in a worker thread so
the event loop — and live telemetry — stay responsive.
"""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, HTTPException, UploadFile

from ..analysis import ulog_pipeline
from ..analysis.ulog_pipeline import UlogTooLargeError

log = logging.getLogger("mint.api.ulog")
router = APIRouter(prefix="/api", tags=["ulog"])


@router.post("/analyze-ulog")
async def analyze_ulog(file: UploadFile) -> dict:
    if not (file.filename or "").lower().endswith(".ulg"):
        raise HTTPException(415, "Expected a PX4 .ulg log file")

    dest = ulog_pipeline.allocate_upload_path()
    try:
        size = await ulog_pipeline.spool_upload(file, dest)
        log.info("ULog received: %s (%.1f MiB)", file.filename, size / 2**20)

        # pyulog + FFT work is synchronous and CPU-bound -> thread pool.
        report = await asyncio.get_running_loop().run_in_executor(
            None, ulog_pipeline.analyze, dest
        )
        report["original_filename"] = file.filename
        report["size_bytes"] = size
        return report
    except UlogTooLargeError as exc:
        raise HTTPException(413, str(exc))
    except Exception as exc:
        log.exception("ULog analysis failed")
        raise HTTPException(422, f"Could not analyze log: {exc}")
    finally:
        dest.unlink(missing_ok=True)   # uploads are transient
