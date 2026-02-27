# To use developer mode, set DEVELOPER_MODE = True
# This will prevent files from getting overwritten
# Also, update platformio.ini to point to your local 
# ERC2 library directory. For example:
# lib_deps = 
#    symlink://../erc2firmwaresource/
DEVELOPER_MODE = False

import hashlib
import json
import os
import ssl
import sys
import time
import urllib.request
from pathlib import Path

# =========================
# CONFIG
# =========================

BASE_RAW_URL = "https://code.osu.edu/fehelectronics/eed_controller/erc2-template/-/raw/main/"

# manifest
MANIFEST_URL = BASE_RAW_URL + "scripts/manifest.json"

# Where to place downloaded files (relative to project root)
DEST_ROOT_REL = "."

# Path blocklist for safety: disallow updating files with these prefixes (relative paths)
DISALLOWED_PREFIXES = (
    ".pio", # PlatformIO build artifacts
    "src", # source code written by students
)

# Network timeouts / behavior
HTTP_TIMEOUT_SEC = 3.0
FAIL_OPEN_IF_OFFLINE = True   # if offline, continue build using local files
RETRY_COUNT = 2               # small retry for flaky WiFi
RETRY_BACKOFF_SEC = 0.25


# =========================
# HELPERS
# =========================

def log(msg: str):
    print(f"\033[96m[ERC2-BOOTSTRAP]\033[0m {msg}")

def _project_root() -> Path:
    """
    Best-effort find PlatformIO project root.
    Usually this script is under <root>/scripts, so parent is root.
    """
    try:
        here = Path(__file__).resolve()
        # scripts/bootstrap.py -> project root is parent of scripts/
        if here.parent.name == "scripts":
            return here.parent.parent
    except NameError:
        pass
    
    # fallback: current working dir
    return Path.cwd().resolve()


def _is_safe_relpath(rel: str) -> bool:
    # Normalize separators and remove leading "./"
    rel = rel.replace("\\", "/").lstrip("./")

    # Disallow absolute paths, parent traversal, or empty
    if not rel or rel.startswith("/") or rel.startswith("..") or "/../" in rel:
        return False

    # Block disallowed prefixes
    if DISALLOWED_PREFIXES:
        if any(rel.startswith(p) for p in DISALLOWED_PREFIXES):
            return False
    
    return True


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        # Read entire file (these are small text files) and normalize CRLF to LF
        content = f.read().replace(b"\r\n", b"\n")
        h.update(content)
    return h.hexdigest()


def _http_get_bytes(url: str) -> bytes:
    # Disable SSL verification for broader compatibility out-of-the-box
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    last_err = None
    for i in range(RETRY_COUNT + 1):
        try:
            req = urllib.request.Request(
                url,
                headers={"User-Agent": "erc2-bootstrap/1.0"},
                method="GET",
            )
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_SEC, context=ctx) as resp:
                return resp.read()
        except Exception as e:
            last_err = e
            if i < RETRY_COUNT:
                time.sleep(RETRY_BACKOFF_SEC * (2 ** i))
    raise last_err  # type: ignore


def _parse_manifest(manifest_text: str):
    """
    JSON format with array of file objects:
    {
      "files": [
        {"path": "...", "sha256": "..."},
        ...
      ]
    }
    """
    try:
        data = json.loads(manifest_text)
        rows = []
        if isinstance(data, dict) and "files" in data:
            for file_obj in data.get("files", []):
                if isinstance(file_obj, dict) and "path" in file_obj and "sha256" in file_obj:
                    rel_path = file_obj["path"].strip().replace("\\", "/")
                    sha256 = file_obj["sha256"].strip()
                    if rel_path and not rel_path.startswith("#"):
                        rows.append((rel_path, sha256))
        return rows
    except json.JSONDecodeError as e:
        log(f"JSON parse error: {e}")
        return []


def _write_atomic(path: Path, data: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


def sync_files():
    root = _project_root()
    dest_root = (root / DEST_ROOT_REL).resolve()

    if DEVELOPER_MODE:
        log("developer mode enabled, skipping sync.")
        return

    log(f"checking updates via {MANIFEST_URL}")

    try:
        manifest_bytes = _http_get_bytes(MANIFEST_URL)
    except Exception as e:
        log(f"manifest download failed: {e}")
        if FAIL_OPEN_IF_OFFLINE:
            log("continuing with local files (offline mode).")
            return
        raise

    rows = _parse_manifest(manifest_bytes.decode("utf-8", errors="replace"))
    if not rows:
        log("manifest empty; nothing to do.")
        return

    updated = 0
    skipped = 0

    for relpath, expected in rows:
        relpath_norm = relpath.lstrip("./").replace("\\", "/")

        if not _is_safe_relpath(relpath_norm):
            log(f"SKIP unsafe path in manifest: {relpath_norm}")
            skipped += 1
            continue

        local_path = (dest_root / relpath_norm).resolve()

        # Ensure local path is still within project root (extra safety)
        if not str(local_path).startswith(str(root)):
            log(f"SKIP path escapes project root: {relpath_norm}")
            skipped += 1
            continue

        remote_url = BASE_RAW_URL + relpath_norm

        needs_update = True
        if local_path.exists():
            # JSON manifest always provides sha256
            local_hash = _sha256_file(local_path)
            needs_update = (local_hash.lower() != expected.lower())

        if not needs_update:
            continue

        try:
            data = _http_get_bytes(remote_url)
            _write_atomic(local_path, data)
            updated += 1
            log(f"Updated: {relpath_norm}")
        except Exception as e:
            log(f"FAILED to update {relpath_norm}: {e}")
            if not FAIL_OPEN_IF_OFFLINE:
                raise

    if updated == 0 and skipped == 0:
        log("already up to date.")
    else:
        log(f"done (updated={updated}, skipped={skipped}).")

# Run immediately when PlatformIO imports this script
sync_files()