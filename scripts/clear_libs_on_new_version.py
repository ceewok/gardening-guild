# -*- coding: utf-8 -*-
import json
import os
import re
import shutil
import ssl
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional

Import("env")  # PlatformIO injects this in extra_scripts


PKG = "osu-eed/ERC2"
GUARD_ENV = "ERC2_AUTOUPDATE_ALREADY_RAN"
LIBDEPS_DIR = Path(".pio") / "libdeps" / "megaatmega2560" / "ERC2"


def log(msg: str):
    print(f"\033[95m[ERC2-LIB]\033[0m {msg}")


def run_pio(args, check=True):
    """
    Run PlatformIO in a cross-platform way using the current Python interpreter.
    This avoids relying on 'pio' being on PATH.
    """
    cmd = [sys.executable, "-m", "platformio"] + args
    log(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=check).returncode


def fetch_latest_version_from_registry(pkg: str) -> Optional[str]:
    """
    Uses PlatformIO Registry API v3 to search for the package and read its version.
    """
    try:
        # We search by owner if the package has an owner prefix, e.g. "osu-eed/ERC2"
        if "/" in pkg:
            owner, pkg_name = pkg.split("/", 1)
            query = f'owner:"{owner}" "{pkg_name}"'
        else:
            pkg_name = pkg
            query = f'"{pkg_name}"'
            
        url = "https://api.registry.platformio.org/v3/search?query=" + urllib.parse.quote(query)
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=10, context=ctx) as response:
            data = json.loads(response.read().decode("utf-8"))
            
        # The search returns a list of items. Find the one matching the package name.
        for item in data.get("items", []):
            if item.get("name", "").lower() == pkg_name.lower():
                version_info = item.get("version")
                if version_info and "name" in version_info:
                    return version_info["name"]
                    
    except Exception as e:
        log(f"Could not fetch version from Registry API ({e}). Skipping auto-update.")
        
    return None


def parse_installed_version_from_pkg_list(pkg: str, pioenv: str) -> Optional[str]:
    """
    Parses `pio pkg list -e <env>` output for a line like:
      eed-osu/ERC2 @ 1.0.4
    or
      ERC2 @ 1.3.4
    """
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "platformio", "pkg", "list", "-e", pioenv],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding='utf-8',
            errors='replace',
            check=False,
        )
    except Exception as e:
        log(f"Failed to run 'pio pkg list' ({e}).")
        return None

    out = proc.stdout or ""
    
    # Extract just the package name if it has an owner prefix
    pkg_name = pkg.split("/")[-1] if "/" in pkg else pkg

    # Typical formatting contains "name @ version"
    # We look for the package name (without owner) or with owner
    pattern = re.compile(rf"(?:{re.escape(pkg)}|{re.escape(pkg_name)})\s*@\s*([0-9]+(?:\.[0-9]+)*)")
    m = pattern.search(out)
    if m:
        return m.group(1)

    return None


def ver_tuple(v: str):
    # Simple semver-ish compare: "1.2.10" > "1.2.9"
    return tuple(int(x) for x in v.split("."))


def main():
    # Prevent infinite recursion when we restart `pio run`
    if os.environ.get(GUARD_ENV) == "1":
        log("Guard set; not running again.")
        return

    pioenv = env.get("PIOENV") or os.environ.get("PIOENV") or ""
    if not pioenv:
        # Fallback: PlatformIO normally sets PIOENV, but just in case
        pioenv = env.subst("$PIOENV")

    if not pioenv:
        log("Could not determine PIO environment; skipping auto-update.")
        return

    latest = fetch_latest_version_from_registry(PKG)
    if not latest:
        log("Could not determine latest version; skipping auto-update.")
        return

    installed = parse_installed_version_from_pkg_list(PKG, pioenv)
    if not installed:
        log(f"{PKG} not found in installed deps for env '{pioenv}'. Skipping.")
        return

    log(f"Installed {PKG} = {installed}, latest = {latest}")

    try:
        if ver_tuple(latest) <= ver_tuple(installed):
            log("Already up to date.")
            return
    except Exception:
        # If parsing fails, be conservative and do nothing
        log("Version compare failed; skipping auto-update.")
        return

    log(f"Newer version detected ({installed} -> {latest}). Updating...")

    # rm -rf .pio/libdeps
    if LIBDEPS_DIR.exists():
        shutil.rmtree(LIBDEPS_DIR, ignore_errors=True)

    # PlatformIO caches HTTP responses (package metadata + download URLs) in
    # ~/.platformio/.cache/http/. These cached entries can hold stale version
    # info even after a new release is published to the registry. We need to
    # purge any ERC2-related cache entries so PlatformIO resolves the new version.
    pkg_name = PKG.split("/")[-1].lower()
    pio_http_cache = Path.home() / ".platformio" / ".cache" / "http"
    if pio_http_cache.is_dir():
        for entry in pio_http_cache.iterdir():
            try:
                content = entry.read_text(encoding="utf-8", errors="ignore")
                if pkg_name in content.lower():
                    log(f"Clearing stale cache entry: {entry.name}")
                    entry.unlink(missing_ok=True)
            except Exception:
                pass

    # Also clear the downloads cache in case old tarballs are cached
    pio_dl_cache = Path.home() / ".platformio" / ".cache" / "downloads"
    if pio_dl_cache.is_dir():
        for entry in pio_dl_cache.iterdir():
            try:
                if pkg_name in entry.name.lower():
                    log(f"Clearing cached download: {entry.name}")
                    entry.unlink(missing_ok=True)
            except Exception:
                pass


main()