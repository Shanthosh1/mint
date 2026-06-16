"""
Post-flight ULog upload + analysis endpoint.

The upload is spooled to disk in 1 MiB chunks (never fully buffered in
RAM) and the CPU-heavy pyulog/scipy pipeline runs in a worker thread so
the event loop — and live telemetry — stay responsive.
"""
from __future__ import annotations

import asyncio
import logging
import hashlib
import json
from pathlib import Path

from fastapi import APIRouter, HTTPException, UploadFile

from ..analysis import ulog_pipeline
from ..analysis.ulog_pipeline import UlogTooLargeError, UnsupportedLogError

log = logging.getLogger("mint.api.ulog")
router = APIRouter(prefix="/api", tags=["ulog"])

# Cache directory on disk to persist reports across restarts
ULOG_CACHE_DIR = Path.home() / ".mint" / "cache"
ULOG_CACHE_DIR.mkdir(parents=True, exist_ok=True)


@router.post("/analyze-ulog")
async def analyze_ulog(file: UploadFile) -> dict:
    if not (file.filename or "").lower().endswith(".ulg"):
        raise HTTPException(415, "Expected a PX4 .ulg log file")

    dest = ulog_pipeline.allocate_upload_path()
    try:
        size = await ulog_pipeline.spool_upload(file, dest)

        # Calculate content SHA-256 hash
        sha = hashlib.sha256()
        with open(dest, "rb") as f:
            while chunk := f.read(65536):
                sha.update(chunk)
        file_hash = sha.hexdigest()

        # Check disk cache
        cache_file = ULOG_CACHE_DIR / f"{file_hash}.json"
        if cache_file.exists():
            log.info("Duplicate ULog upload detected (disk cache hit). Reusing cached report for file: %s (hash: %s)", 
                     file.filename, file_hash)
            try:
                with open(cache_file, "r") as f:
                    report = json.load(f)
                report["original_filename"] = file.filename
                report["size_bytes"] = size
                report["file_hash"] = file_hash
                return report
            except Exception as e:
                log.warning("Failed to read cached report from disk: %s. Re-analyzing...", e)

        log.info("ULog received (cache miss): %s (%.1f MiB, hash: %s). Starting analysis...", 
                 file.filename, size / 2**20, file_hash)

        # pyulog + FFT work is synchronous and CPU-bound -> thread pool.
        report = await asyncio.get_running_loop().run_in_executor(
            None, ulog_pipeline.analyze, dest
        )
        report["original_filename"] = file.filename
        report["size_bytes"] = size
        report["file_hash"] = file_hash

        # Cache the report on disk
        try:
            with open(cache_file, "w") as f:
                json.dump(report, f)
            log.info("ULog analysis completed and cached on disk. Hash: %s", file_hash)
        except Exception as e:
            log.warning("Failed to write report to disk cache: %s", e)

        return report
    except UlogTooLargeError as exc:
        raise HTTPException(413, str(exc))
    except UnsupportedLogError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        log.exception("ULog analysis failed")
        raise HTTPException(422, f"Could not analyze log: {exc}")
    finally:
        dest.unlink(missing_ok=True)   # uploads are transient


@router.get("/report/{file_hash}")
async def get_cached_report(file_hash: str) -> dict:
    """Retrieve a previously analyzed and cached ULog report by its file hash."""
    cache_file = ULOG_CACHE_DIR / f"{file_hash}.json"
    if not cache_file.exists():
        raise HTTPException(404, "Report not found in cache")
    try:
        with open(cache_file, "r") as f:
            report = json.load(f)
        report["file_hash"] = file_hash
        return report
    except Exception as e:
        log.error("Failed to read cached report %s from disk: %s", file_hash, e)
        raise HTTPException(500, f"Failed to read cached report: {e}")

