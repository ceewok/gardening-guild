import json
import os
import re
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path

Import("env")  # PlatformIO injects this in extra_scripts


PKG = "osu-eed/ERC2"
GUARD_ENV = "ERC2_AUTOUPDATE_ALREADY_RAN"
LIBDEPS_DIR = Path(".pio") / "libdeps"


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


def fetch_latest_version_from_registry(pkg: str) -> str | None:
    """
    Uses PlatformIO CLI to search for the package and read its version.
    """
    try:
        # We search by owner if the package has an owner prefix, e.g. "osu-eed/ERC2"
        if "/" in pkg:
            owner = pkg.split("/")[0]
            query = f"owner:{owner}"
        else:
            query = pkg
            
        proc = subprocess.run(
            [sys.executable, "-m", "platformio", "pkg", "search", query],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    except Exception as e:
        log(f"Could not run pio pkg search ({e}). Skipping auto-update.")
        return None

    out = proc.stdout or ""
    
    # Look for the package name followed by a line containing the version
    # Example output:
    # osu-eed/ERC2
    # Library • 1.3.4 • Published on Thu Feb 26 19:04:39 2026
    
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if line.strip().lower() == pkg.lower():
            # The next line should contain the version
            if i + 1 < len(lines):
                next_line = lines[i + 1]
                # Extract version, e.g., "Library • 1.3.4 • Published..."
                # We can use a regex to find the version number
                m = re.search(r"•\s*([0-9]+(?:\.[0-9]+)*)\s*•", next_line)
                if m:
                    return m.group(1)
                
                # Fallback regex if the format is slightly different
                m2 = re.search(r"([0-9]+\.[0-9]+\.[0-9]+)", next_line)
                if m2:
                    return m2.group(1)
                    
    return None


def parse_installed_version_from_pkg_list(pkg: str, pioenv: str) -> str | None:
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

    log("Newer version detected. Clearing libdeps to force re-download...")

    # 1) rm -rf .pio/libdeps
    if LIBDEPS_DIR.exists():
        shutil.rmtree(LIBDEPS_DIR, ignore_errors=True)

main()