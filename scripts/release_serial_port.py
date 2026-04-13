# -*- coding: utf-8 -*-
"""
PlatformIO pre-upload script — release_serial_port.py

Asks a running erc2-link application to release the ERC2 serial port
before avrdude (or any other uploader) tries to open it.

erc2-link exposes a tiny HTTP control server on localhost:54321.
If the app is not running, this script does nothing and build continues.

Usage (platformio.ini):
    extra_scripts =
        pre:scripts/bootstrap.py
        pre:scripts/clear_libs_on_new_version.py
        upload_pre:scripts/release_serial_port.py
"""

import time
import urllib.error
import urllib.request

# Must match ControlServer.CONTROL_PORT in erc2-link
ERC2_LINK_CONTROL_URL = "http://127.0.0.1:54321"

# How long to wait for the app to actually close the port after our request
SETTLE_TIME_SEC = 0.5

# Timeout for the HTTP request itself
HTTP_TIMEOUT_SEC = 2.0


def _log(msg: str) -> None:
    print(f"\033[96m[ERC2-UPLOAD]\033[0m {msg}")


def release_serial() -> None:
    """
    Ask erc2-link to release the serial port.

    If the app is not running (connection refused) we skip silently —
    avrdude will be able to open the port directly in that case.
    """
    # Quick check: is erc2-link running at all?
    try:
        with urllib.request.urlopen(
            f"{ERC2_LINK_CONTROL_URL}/status", timeout=HTTP_TIMEOUT_SEC
        ):
            pass  # connected; continue
    except (urllib.error.URLError, OSError):
        _log("erc2-link not detected — skipping serial release.")
        return

    # Send the release request
    try:
        req = urllib.request.Request(
            f"{ERC2_LINK_CONTROL_URL}/release-serial",
            data=b"",   # non-empty triggers POST
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_SEC) as resp:
            _log(f"Serial port released by erc2-link (HTTP {resp.status}).")
    except Exception as exc:
        _log(f"Warning: could not release serial port: {exc}")
        return

    # Give the OS a moment to fully close the handle before avrdude opens it
    _log(f"Waiting {SETTLE_TIME_SEC}s for port to become available…")
    time.sleep(SETTLE_TIME_SEC)


# PlatformIO calls this script as a pre-upload extra_script.
# Both Import() and plain top-level code are executed.
Import("env")  # noqa: F821  — PlatformIO injects this

env.AddPreAction("upload", lambda target, source, env: release_serial())  # noqa: F821
