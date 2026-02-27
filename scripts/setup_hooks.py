import os
import stat
from pathlib import Path

def main():
    root = Path(__file__).resolve().parent.parent
    hook_dir = root / ".git" / "hooks"
    
    if not hook_dir.exists():
        print(".git/hooks directory not found. Are you in a git repository?")
        return
        
    pre_commit_path = hook_dir / "pre-commit"
    
    hook_script = """#!/bin/sh
# Automatically run manifest updater
if command -v python3 >/dev/null 2>&1; then
    python3 scripts/update_manifest.py
else
    python scripts/update_manifest.py
fi
"""
    
    with pre_commit_path.open("w", newline="\n") as f:
        f.write(hook_script)
        
    # Ensure it's executable
    try:
        st = pre_commit_path.stat()
        pre_commit_path.chmod(st.st_mode | stat.S_IEXEC)
    except Exception:
        pass
        
    print(f"pre-commit hook installed at {pre_commit_path}")

if __name__ == "__main__":
    main()
